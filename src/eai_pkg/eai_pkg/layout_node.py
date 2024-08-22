import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from eai_interfaces.srv import RsDeproject
import message_filters
import json
import cv2
import numpy as np
from eai_interfaces.srv import LayoutAnalysis  # Assuming the service definition is in this package
# openvino.基于IR的推理器
from .openvino.inferers.infer_normal import NormalInferer
# openvino.推理解析及可视化
from .openvino.analyzers.analyzer_yolo import AnalyzerYolo


class LayoutNode(Node):
    def __init__(self):
        super().__init__('layout_node')
        self.bridge = CvBridge()

        # Subscribers using message_filters for time synchronization
        color_sub = message_filters.Subscriber(self, Image, '/rs_aligned_color')
        depth_sub = message_filters.Subscriber(self, Image, '/rs_aligned_depth')

        ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.sync_callback)

        # Service
        self.srv = self.create_service(LayoutAnalysis, 'layout_analysis', self.layout_analysis_callback)
        self.rs_deproject_client = self.create_client(RsDeproject, 'rs_deproject')

        # transform matrix v2
        # self.rotation_matrix = np.array([[0.9915888, 0.00678343, - 0.1292503],
        #                                  [0.09334349, - 0.72925641, 0.6778437],
        #                                  [-0.08965851, - 0.68420689, - 0.72375568]])
        # self.shift_matrix = np.array([32.70650206, - 114.97964346, 450.24596395])
        # transform matrix v3
        self.rotation_matrix = np.array(
            [[0.99432298, - 0.0244465, - 0.10355766],
             [0.05487418, - 0.71602738, 0.69591207],
             [-0.09116273, - 0.69764401, - 0.71062099]])
        self.shift_matrix = np.array(
            [17.05921779, - 123.92059465, 444.63119675])

        # Placeholder for the latest layout
        self.latest_layout_c_nxywhd = None
        self.latest_layout_rs_nxyd = None
        self.latest_layout_rs_nxyz = None
        self.latest_layout_arm_nxyz = None

        # openvino-yolo inferer
        self.inferer = NormalInferer(ir_model_path="blocks.xml", imgsz=(640, 640), device="CPU")
        self.classes = {0: 'red_block', 1: "purple_block", 2: 'green_block', 3: 'blue_block', 4: 'yellow_block'}

        # Init success
        self.get_logger().info(f'INFO --- layout_node set-up finished ^.^')

    def sync_callback(self, color_msg, depth_msg):
        # Convert ROS Image messages to CV images
        color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

        # Process images to get layout_c_nxywhd{'name':(x,y,w,h,mean_depth)}
        layout_c_nxywhd = self.get_layout_c_nxywhd(color_image, depth_image)
        self.latest_layout_c_nxywhd = layout_c_nxywhd
        # convert to realsense axis
        names = []
        c_x = []
        c_y = []
        c_d = []
        for name, c_xywhd in layout_c_nxywhd.items():
            names.append(name)
            c_x.append(int((c_xywhd[0] + c_xywhd[0] + c_xywhd[2]) / 2))
            c_y.append(int((c_xywhd[1] + c_xywhd[1] + c_xywhd[3]) / 2))
            c_d.append(int(c_xywhd[4]))
        # send arrays to realsense_node for analysis
        rs_req = RsDeproject.Request()
        rs_req.c_x = c_x
        rs_req.c_y = c_y
        rs_req.c_d = c_d
        future = self.rs_deproject_client.call_async(rs_req)
        future.add_done_callback(lambda fut: self.rs_deproject_received_callback(fut, names))
        # self.get_logger().info(f'INFO --- 0. Layout analysis and send to rs for deproject')

    def rs_deproject_received_callback(self, future, names):
        try:
            rs_response = future.result()
            rs_x = rs_response.rs_x
            rs_y = rs_response.rs_y
            rs_d = rs_response.rs_d
            # self.get_logger().info(f'INFO --- 1. Received rs_xyd: {rs_x}, {rs_y}, {rs_d}')

            layout_rs_nxyd = {}  # form layout_rs_nxyd
            for idx, name in enumerate(names):
                layout_rs_nxyd[name] = (rs_x[idx], rs_y[idx], rs_d[idx])
            self.latest_layout_rs_nxyd = layout_rs_nxyd
            # self.get_logger().info(f'INFO --- 2. Formed layout_rs_nxyd: {layout_rs_nxyd}')

            layout_rs_nxyz = {}  # form layout_rs_nxyz
            for name, rs_xyd in layout_rs_nxyd.items():
                rs_x, rs_y, rs_z = self.rs_xyd_2_rs_xyz(rs_xyd[0], rs_xyd[1], rs_xyd[2])
                layout_rs_nxyz[name] = (rs_x, rs_y, int(rs_z))
            self.latest_layout_rs_nxyz = layout_rs_nxyz
            # self.get_logger().info(f'INFO --- 3. Formed layout_rs_nxyz: {layout_rs_nxyz}')

            layout_arm_nxyz = {}  # form layout_arm_nxyz{'name':(arm_x,arm_y,arm_z)}
            for name, rs_xyz in layout_rs_nxyz.items():
                rs_coordinate = [rs_xyz[0], rs_xyz[1], rs_xyz[2]]
                arm_coordinate = np.dot(self.rotation_matrix, np.array(rs_coordinate)) + self.shift_matrix
                arm_x, arm_y, arm_z = arm_coordinate
                layout_arm_nxyz[name] = (int(arm_x), int(arm_y), int(arm_z))
            self.latest_layout_arm_nxyz = layout_arm_nxyz
            # self.get_logger().info(f'INFO --- 4. Formed layout_arm_nxyz: {layout_arm_nxyz}')

            # layout_board = np.ones((480, 640, 3), dtype=np.uint8) * 255  # 白色背景
            # for name in self.latest_layout_c_nxywhd.keys():
            #     cx, cy, w, h, _ = self.latest_layout_c_nxywhd[name]
            #     ax, ay, az = self.latest_layout_arm_nxyz[name]
            #     txt = name[:-6] + "," + str(ax) + "," + str(ay) + "," + str(az)
            #     cv2.putText(layout_board, txt, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            #     # cv2.rectangle(layout_board, (cx, cy), (cx + w, cy + h), (0, 255, 0), 2)
            # cv2.imshow("layout board (arm axis)", layout_board)
            # cv2.waitKey(5)

        except Exception as e:
            self.get_logger().error('ERROR --- rs_deproject_received_callback failed %r' % (e,))
            # self.get_logger().error(f'ERROR --- future.result() = {future.result()}')

    def layout_analysis_callback(self, request, response):
        if request.ask_for_layout:
            # Serialize the latest layout to JSON string
            if request.ask_for_layout == "c_nxywhd":
                response.layout = json.dumps(self.latest_layout_c_nxywhd)
            elif request.ask_for_layout == "rs_nxyd":
                response.layout = json.dumps(self.latest_layout_rs_nxyd)
            elif request.ask_for_layout == "rs_nxyz":
                response.layout = json.dumps(self.latest_layout_rs_nxyz)
            elif request.ask_for_layout == "arm_nxyz":
                response.layout = json.dumps(self.latest_layout_arm_nxyz)
            else:
                response.layout = "{}"
        else:
            response.layout = "{}"
        return response

    def get_layout_c_nxywhd(self, color_image, depth_image):
        # Implement your color block detection and depth calculation here
        # Placeholder for the function's logic, returning a dummy layout

        # layout = self.get_layout_color_blocks(color_image, depth_image)

        layout = self.get_layout_color_blocks_yolov5(color_image, depth_image)

        return layout

    def rs_xyd_2_rs_xyz(self, rs_x, rs_y, rs_dis):  # mm
        # 使用勾股定理计算z，这里假设rs_dis已经代表从相机到点的直线距离
        # 计算物体在相机坐标系中的实际z坐标
        rs_xyl2 = rs_x ** 2 + rs_y ** 2  # 计算x和y分量的平方和
        if rs_dis ** 2 - rs_xyl2 >= 0:
            rs_z = np.sqrt(rs_dis ** 2 - rs_xyl2)  # 根据勾股定理计算z值,must be >0
            return rs_x, rs_y, rs_z  # mm
        else:
            print("error: rs_dis**2 - rs_xyl2 < 0")
            return 0, 0, 0  # mm

    def get_layout_color_blocks_yolov5(self, color_image, depth_image):
        if color_image is None or depth_image is None:
            self.get_logger().warn(f'WARN --- color_image or depth_image is None')
            return {}
        # Define HSV color boundaries for color blocks

        layout = {}
        # print(f"color_image.shape() = {color_image.shape}")
        infer1 = self.inferer.infer_once(color_image)
        # openvino.analyze:解析推理结果并可视化
        analyzer_yolo = AnalyzerYolo(classes=self.classes, imgsz=(640, 640))
        pred = analyzer_yolo.infer_analyse(infer1)

        outputs = pred[0][:, :6]

        color_image = cv2.resize(color_image, (640, 640))
        # print(f"color_image.shape() = {color_image.shape}")

        if len(outputs[:, 4:] > 0):
            for i in outputs:
                prob = i[4]
                cls = int(i[5])
                prob = np.around(prob, decimals=2)
                if prob >= 0.4:
                    all_pred_boxes = i[:4]
                    for b in range(len(all_pred_boxes)):
                        x1 = int(all_pred_boxes[0])
                        y1 = int(all_pred_boxes[1] * 3 / 4)
                        x2 = int(all_pred_boxes[2])
                        y2 = int(all_pred_boxes[3] * 3 / 4)

                        color_image = cv2.resize(color_image, (640, 480))
                        # print(f"color_image.shape() = {color_image.shape}")

                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 1)
                        cv2.putText(color_image, self.classes[cls][:2] + '' + str(prob)[1:], (x1, y1),
                                    cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), 1, 4)

                        x, y, w, h = x1, y1, abs(x2 - x1), abs(y2 - y1)
                        depth_region = depth_image[y:y + h, x:x + w]
                        valid_depths = depth_region[(depth_region > 0) & ~np.isnan(depth_region)]
                        # valid_depths = depth_region[(depth_region > depth_threshold_low) & (depth_region < depth_threshold_high)]
                        mean_depth = np.mean(valid_depths) if len(valid_depths) > 0 else 0

                        layout[self.classes[cls]] = (x, y, w, h, mean_depth)

        else:
            color_image = cv2.resize(color_image, (640, 480))
        cv2.imshow("get_layout_color_blocks_yolov5", color_image)
        cv2.waitKey(10)
        return layout

    def get_layout_color_blocks(self, color_image, depth_image):
        if color_image is None or depth_image is None:
            self.get_logger().warn(f'WARN --- color_image or depth_image is None')
            return {}
        # Define HSV color boundaries for color blocks
        color_ranges = {
            'red_block': ((0, 100, 100), (20, 255, 255)),
            'yellow_block': ((20, 30, 30), (40, 255, 255)),
            'green_block': ((40, 50, 50), (90, 255, 255)),
            'blue_block': ((90, 50, 50), (140, 255, 255)),
            'purple_block': ((140, 50, 50), (200, 255, 255))
        }
        # Color block size threshold, color blocks smaller than this area will be ignored
        size_threshold_l = 300
        size_threshold_h = 1200
        layout = {}
        # Convert color images from BGR to HSV
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        for color_name, (lower, upper) in color_ranges.items():
            # Create Mask
            mask = cv2.inRange(hsv_image, lower, upper)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)  # 填充内部小洞
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)  # 去噪声
            # Searching for the outline of color blocks
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if size_threshold_l < cv2.contourArea(contour) < size_threshold_h:
                    # Calculate the minimum rectangular box for each color block
                    x, y, w, h = cv2.boundingRect(contour)
                    # cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    # Calculate the mean depth
                    depth_region = depth_image[y:y + h, x:x + w]
                    valid_depths = depth_region[(depth_region > 0) & ~np.isnan(depth_region)]
                    # valid_depths = depth_region[(depth_region > depth_threshold_low) & (depth_region < depth_threshold_high)]
                    mean_depth = np.mean(valid_depths) if len(valid_depths) > 0 else 0

                    cx = (x + x + w) // 2
                    cy = (y + y + h) // 2
                    # Put text
                    # cv2.putText(color_image, f"({color_name[:-6]},{cx},{cy},{mean_depth:.1f})", (x, y),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    #             (0, 255, 0), 2)

                    # Form layout ??? if multi blocks?
                    layout[color_name] = (x, y, w, h, mean_depth)

        # Display (optional)
        # cv2.imshow("color_image", color_image)
        # cv2.waitKey(1)

        return layout


def main(args=None):
    rclpy.init(args=args)
    node = LayoutNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
