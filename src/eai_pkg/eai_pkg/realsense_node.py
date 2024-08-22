import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from eai_interfaces.srv import RsDeproject, ColorImg
import json
import numpy as np
import cv2
import pyrealsense2 as rs


class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')  # Node name changed
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('camera.width', 640),
                                    ('camera.height', 480),
                                    ('camera.fps', 30)
                                ])
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Retrieve parameters
        width = self.get_parameter('camera.width').get_parameter_value().integer_value
        height = self.get_parameter('camera.height').get_parameter_value().integer_value
        fps = self.get_parameter('camera.fps').get_parameter_value().integer_value

        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.profile = self.pipeline.start(config)

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        self.bridge = CvBridge()

        # Publishers
        self.pub_color = self.create_publisher(Image, '/rs_aligned_color', 10)
        self.pub_depth = self.create_publisher(Image, '/rs_aligned_depth', 10)
        # self.latest_color_img = None
        # self.latest_depth_img = None

        # Service
        self.srv = self.create_service(RsDeproject, 'rs_deproject', self.deproject_callback)

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.color_img_service = self.create_service(ColorImg, 'color_img', self.color_img_callback)

        # Set-up finished
        self.get_logger().info(f'INFO --- realsense_node set-up finished ^.^')

    def deproject_callback(self, request, response):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()

        if not aligned_depth_frame:
            self.get_logger().error('ERROR --- No depth frame available')
            response.rs_x = []
            response.rs_y = []
            response.rs_d = []
            return response  # Assuming response has a way to indicate failure

        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        # depth_xy = aligned_depth_frame.get_distance(request.x, request.y)
        rq_c_x = request.c_x
        rq_c_y = request.c_y
        rq_c_d = request.c_d
        if len(rq_c_x) == len(rq_c_y) and len(rq_c_y) == len(rq_c_d):
            rs_x, rs_y, rs_d = [], [], []
            for idx in range(len(rq_c_x)):
                rs_x_, rs_y_, rs_d_ = rs.rs2_deproject_pixel_to_point(depth_intrin, [rq_c_x[idx], rq_c_y[idx]], rq_c_d[idx])
                rs_x.append(int(rs_x_))
                rs_y.append(int(rs_y_))
                rs_d.append(int(rs_d_))
            response.rs_x = rs_x
            response.rs_y = rs_y
            response.rs_d = rs_d
        else:
            response.rs_x = []
            response.rs_y = []
            response.rs_d = []
        # camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, [request.x, request.y], request.d)

        # self.get_logger().info(f'{type(camera_coordinate), type(camera_coordinate[0])}')
        # response.rs_coordinate = camera_coordinate
        # response.rs_coordinate = [float(coord) for coord in camera_coordinate]  # 强制转换为浮点数列表

        # json_rs_coordinate = json.dumps(camera_coordinate)
        # response.rs_coordinate = json_rs_coordinate
        # self.get_logger().info(
        #     f'INFO --- INPUT: rs_deproject c_x,y,d = {request.x},{request.y},{request.d} OUTPUT: return rs_x,y,d = {camera_coordinate}')

        return response

    def timer_callback(self):
        # get aligned frames
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            self.get_logger().warn('WARN --- No frames available')
            return

        # Convert images to numpy arrays
        depth_image = np.asanyarray(aligned_depth_frame.get_data(), dtype=np.uint16)
        color_image = np.asanyarray(color_frame.get_data())

        # self.latest_color_img = color_image
        # self.latest_depth_img = depth_image

        # Publish color and depth images
        color_msg = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
        color_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, '16UC1')
        depth_msg.header.stamp = self.get_clock().now().to_msg()

        self.pub_color.publish(color_msg)
        self.pub_depth.publish(depth_msg)

        # Display images(optional)
        # cv2.imshow("live", np.hstack(
        #     (color_image, cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.14), cv2.COLORMAP_JET))))
        # cv2.waitKey(1)

    def color_img_callback(self, request, response):
        # Construct response with the latest color image
        if request.color_img_req:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            # aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not color_frame:
                self.get_logger().warn('WARN --- No frames available')
                return
            
            color_image = np.asanyarray(color_frame.get_data())
            response.color_img_res = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
            return response

    def stop(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
