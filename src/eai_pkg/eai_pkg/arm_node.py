# arm_node，它主要功能是：
# 1.创建服务'basic_arm_cmd'的服务端，接收由center_node依次传来的BasicArmCmd。basic_arm_cmd(string，json序列化后的)，进行反序列化，得到一个列表basic_arm_cmd = ['action_type', ('obj_name', cx, cy)]。
# 2.根据cx,cy，向realsense_node的服务'rs_deproject'发送请求，也就是RsDeproject.x=cx，RsDeproject.y = cy。然后等待服务的回应，返回的是一个float32[] rs_coordinate.其中有三个数rs_x, rs_y, rs_dis = rs_coordinate。拿到这三个数后，传入并执行函数rs2arm()，这个函数由我来写，返回的是三个坐标arm_x, arm_y, arm_z。
# 3.创建一个TCP客户端，将得到的arm_x, arm_y, arm_z发送到端口host = '192.168.1.105‘,port = 12345。然后等待回应，如果返回的是1，就给basic_arm_cmd服务返回arm_finished=True。

import socket
import rclpy
from rclpy.node import Node
from eai_interfaces.srv import BasicArmCmd  # 假设服务定义在eai_interfaces包中
import time
import json
import numpy as np


class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')
        self.srv = self.create_service(BasicArmCmd, 'basic_arm_cmd', self.basic_arm_cmd_callback)
        self.tcp_client = None
        # # v1
        # self.rotation_matrix = np.array([[ 0.99397669, -0.00778223, -0.10931499],
        #                                  [ 0.0720509,  -0.70519207,  0.70534588],
        #                                  [-0.08257723, -0.70897361, -0.70038376]])
        # self.shift_matrix = np.array([23.61584224, -133.16565826,  440.36099586])
        # v2
        self.rotation_matrix = np.array([[0.9915888, 0.00678343, - 0.1292503],
                                         [0.09334349, - 0.72925641, 0.6778437],
                                         [-0.08965851, - 0.68420689, - 0.72375568]])
        self.shift_matrix = np.array([32.70650206, - 114.97964346, 450.24596395])

        self.create_tcp_client()
        self.get_logger().info('INFO --- arm_node set-up finished ^.^')

    def create_tcp_client(self):
        # 替换为实际的IP地址和端口
        host = '192.168.1.111'
        port = 12345
        self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        connected = False
        while not connected:
            try:
                self.tcp_client.connect((host, port))
                self.get_logger().info(f'INFO --- TCP connection ({host},{port})established')
                connected = True
            except socket.error as e:
                self.get_logger().error(f'ERROR --- Failed to connect to {host}:{port}, error: {e}')
                connected = False
                time.sleep(3)

    def basic_arm_cmd_callback(self, request, response):
        # 1.解析request
        basic_arm_cmd = json.loads(request.basic_arm_cmd)
        # send_to_arm_node_cmd_grab = ['grab', grab_obj_name, grab_obj_arm_xyz[0], grab_obj_arm_xyz[1], grab_obj_arm_xyz[2]]
        # send_to_arm_node_cmd_place = ['place', grab_obj_name, place_obj_arm_xyz[0], place_obj_arm_xyz[1], place_obj_arm_xyz[2]]
        # send_to_arm_node_cmd = json.dumps([send_to_arm_node_cmd_grab, send_to_arm_node_cmd_place])
        if len(basic_arm_cmd) == 2:
            grab_cmd = basic_arm_cmd[0]
            place_cmd = basic_arm_cmd[1]
        else:
            self.get_logger().error(f"ERROR --- basic_arm_cmd is not valid, len != 2")
            response.arm_finished = False
            return response
        # 创建TCP客户端发送arm_x, arm_y, arm_z等信息
        send_data = json.dumps([grab_cmd, place_cmd])
        self.get_logger().info(f'INFO --- send Jetson data= {send_data}')
        self.tcp_client.sendall(send_data.encode('utf-8'))
        received = self.tcp_client.recv(1024)
        self.get_logger().info(f'INFO --- Received from Jeston: {received}')

        if received.decode('utf-8') == '1':
            response.arm_finished = True
        else:
            response.arm_finished = False
        return response

    # def rs_xyd_2_rs_xyz(self, rs_x, rs_y, rs_dis):  # mm
    #     # 使用勾股定理计算z，这里假设rs_dis已经代表从相机到点的直线距离
    #     # 计算物体在相机坐标系中的实际z坐标
    #     rs_xyl2 = rs_x ** 2 + rs_y ** 2  # 计算x和y分量的平方和
    #     if rs_dis ** 2 - rs_xyl2 >= 0:
    #         rs_z = np.sqrt(rs_dis ** 2 - rs_xyl2)  # 根据勾股定理计算z值,must be >0
    #         return rs_x, rs_y, rs_z  # mm
    #     else:
    #         self.get_logger().error(f"ERROR --- rs_dis**2 - rs_xyl2 < 0")
    #         return 0, 0, 0  # mm
    #
    # def rs_xyz_2_arm_xyz(self, rs_x, rs_y, rs_z):
    #     rs_coordinate = [rs_x, rs_y, rs_z]
    #     arm_coordinate = np.dot(self.rotation_matrix, np.array(rs_coordinate)) + self.shift_matrix
    #     arm_x, arm_y, arm_z = arm_coordinate
    #     return arm_x, arm_y, arm_z


def main(args=None):
    rclpy.init(args=args)
    arm_node = ArmNode()
    try:
        rclpy.spin(arm_node)
    except KeyboardInterrupt:
        pass
    finally:
        arm_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from eai_interfaces.srv import BasicArmCmd, RsDeproject
# import json
# import socket

# class ArmNode(Node):
#     def __init__(self):
#         super().__init__('arm_node')
#         self.create_service(BasicArmCmd, 'basic_arm_cmd', self.basic_arm_cmd_callback)
#         self.rs_deproject_client = self.create_client(RsDeproject, 'rs_deproject')
#         self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         self.tcp_client.connect(('192.168.1.106', 12345))
#         self.get_logger().info('INFO --- arm_node set-up finished.')

#     def basic_arm_cmd_callback(self, request, response):
#         basic_arm_cmd = json.loads(request.basic_arm_cmd)
#         action_type, (obj_name, cx, cy) = basic_arm_cmd
#         self.get_logger().info(f'INFO --- arm_node received basic_arm_cmd = {basic_arm_cmd}')

#         # 向realsense_node发送rs_deproject请求
#         if not self.rs_deproject_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().error('ERROR --- rs_deproject service not available')
#             response.arm_finished = False
#             return response

#         rs_req = RsDeproject.Request()
#         rs_req.x = cx
#         rs_req.y = cy
#         future = self.rs_deproject_client.call_async(rs_req)
#         self.get_logger().info(f'INFO --- arm_node send rs_req = {rs_req}')
#         future.add_done_callback(lambda fut: self.rs_deproject_received_callback(fut, response, action_type, obj_name))

#     def rs_deproject_received_callback(self, future, response, action_type, obj_name):
#         self.get_logger().info(f'INFO --- rs_deproject_received_callback')
#         try:
#             rs_response = future.result()
#             rs_x, rs_y, rs_dis = rs_response.rs_coordinate
#             self.get_logger().info(f'INFO --- arm_node rs_response.rs_coordinate = {rs_response.rs_coordinate}')
#             arm_x, arm_y, arm_z = self.rs2arm(rs_x, rs_y, rs_dis)

#             # 向TCP服务器发送数据
#             send_data = json.dumps((action_type, obj_name, arm_x, arm_y, arm_z))
#             self.get_logger().info(f'INFO --- arm_node send_data = {send_data}')
#             self.tcp_client.sendall(send_data.encode('utf-8'))
#             received = self.tcp_client.recv(1024).decode('utf-8')

#             if received == '1':
#                 response.arm_finished = True
#             else:
#                 response.arm_finished = False
#         except Exception as e:
#             self.get_logger().error(f'ERROR --- Failed to process deproject or TCP command: {e}')
#             response.arm_finished = False
#         finally:
#             # 重要：确保在任何情况下都返回response对象
#             return response

#     def rs2arm(self, rs_x, rs_y, rs_dis):
#         # 这里应该是你的转换逻辑
#         return (0.1, 0.1, 0.1)  # 示例

# def main(args=None):
#     rclpy.init(args=args)
#     node = ArmNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         node.tcp_client.close()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# import socket
# import json
# from eai_interfaces.srv import BasicArmCmd, RsDeproject
# import time

# class ArmNode(Node):
#     def __init__(self):
#         super().__init__('arm_node')
#         self.srv = self.create_service(BasicArmCmd, 'basic_arm_cmd', self.basic_arm_cmd_callback)
#         self.rs_deproject_client = self.create_client(RsDeproject, 'rs_deproject')
#         self.tcp_client = None
#         self.create_tcp_client()

#     def create_tcp_client(self):
#         # 替换为实际的IP地址和端口
#         host = '192.168.1.106'
#         port = 12345
#         self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         connected = False
#         while not connected: 
#             try:
#                 self.tcp_client.connect((host, port))
#                 self.get_logger().info('INFO --- TCP connection established')
#                 connected = True
#             except socket.error as e:
#                 self.get_logger().error(f'ERROR --- Failed to connect to {host}:{port}, error: {e}')
#                 connected = False
#                 time.sleep(1)
#         self.get_logger().info('INFO --- arm_node set-up finished.')

#     def basic_arm_cmd_callback(self, request, response):
#         basic_arm_cmd = json.loads(request.basic_arm_cmd)
#         self.get_logger().info(f'INFO --- Received basic_arm_cmd = {basic_arm_cmd}')
#         action_type, (obj_name, cx, cy) = basic_arm_cmd
#         self.get_logger().info(f'INFO --- action_type, (obj_name, cx, cy) = {action_type}, ({obj_name},{cx},{cy})')

#         # Sending request to rs_deproject service
#         while not self.rs_deproject_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().warn('WARN --- waiting for rs_deproject service...')

#         rs_req = RsDeproject.Request()
#         rs_req.x = cx
#         rs_req.y = cy
#         self.get_logger().info(f'INFO --- Send RsDeproject = x: {cx},y: {cy}')
#         future = self.rs_deproject_client.call_async(rs_req)
#         future.add_done_callback(lambda fut: self.rs_deproject_received_callback(fut))
#         # # 3.发送解算请求到解算服务端
#         # future = self.calcu_client.call_async(req) 
#         # future.add_done_callback(lambda future: self.calcu_response_callback(future))

#         # rclpy.spin_until_future_complete(self, future)  # ???
#         # self.get_logger().info(f'***after rclpy.spin_until_future_complete(self, future) ')
#         # try:
#         #     rs_response = future.result()
#         # except Exception as e:
#         #     self.get_logger().error('ERROR --- Service call failed %r' % (e,))
#         #     response.arm_finished = False
#         #     return response

#         # # Assuming rs2arm function is defined elsewhere and imported
#         # json_rs_coordinate = rs_response.rs_coordinate
#         # rs_coordinate = json.loads(json_rs_coordinate)
#         # self.get_logger().info(f'INFO --- Received rs_coordinate: {rs_coordinate}')
#         # arm_x, arm_y, arm_z = self.rs2arm(*rs_coordinate)
#         # self.get_logger().info(f'INFO --- after rs2arm: arm_x/y/z = {arm_x}, {arm_y}, {arm_z}')
#         # # Creating a TCP client to send arm_x, arm_y, arm_z
#         # try:
#         #     send_data = json.dumps((action_type, obj_name, arm_x, arm_y, arm_z))
#         #     self.get_logger().info(f'INFO --- data send to Jeston = {send_data}')
#         #     self.tcp_client.sendall(send_data.encode('utf-8'))
#         #     received = self.tcp_client.recv(1024)
#         #     self.get_logger().info(f'INFO --- Received from Jeston: {received}')
#         #     if received.decode('utf-8') == '1':
#         #         response.arm_finished = True
#         #     else:
#         #         response.arm_finished = False
#         #         return response
#         # except Exception as e:
#         #     self.get_logger().error(f'ERROR --- Failed to send TCP command: {e}')
#         #     response.arm_finished = False
#         #     return response

#         # response.arm_finished = True
#         # return response

#     def rs_deproject_received_callback(self, future):
#         try:
#             rs_response = future.result()
#             rs_coordinate = rs_response.rs_coordinate
#             self.get_logger().info(f'INFO --- Received rs_coordinate: {rs_coordinate}')
#             # response.rs_coordinate = rs_coordinate  # 直接分配给响应的 rs_coordinate 字段
#             arm_x, arm_y, arm_z = self.rs2arm(rs_coordinate)
#             self.get_logger().info(f'INFO --- after rs2arm: arm_x/y/z = {arm_x}, {arm_y}, {arm_z}')

#             # 创建TCP客户端发送arm_x, arm_y, arm_z等信息
#             send_data = json.dumps(('action', 'obj_name', arm_x, arm_y, arm_z))
#             self.tcp_client.sendall(send_data.encode('utf-8'))
#             received = self.tcp_client.recv(1024)
#             self.get_logger().info(f'INFO --- Received from Jeston: {received}')

#             # if received.decode('utf-8') == '1':
#             #     response.arm_finished = True
#             # else:
#             #     response.arm_finished = False
#         except Exception as e:
#             self.get_logger().error(f'ERROR --- Service call failed or TCP command failed: {e}')
#             # response.arm_finished = False
#         finally:
#             # return response
#             return

#     def rs2arm(self, rs_coordinate):
#         rs_x, rs_y, rs_dis = rs_coordinate

#         # from rs_coordinate to arm_xyz
#         pass
#         return (0.05, 0.05, 0.05)
