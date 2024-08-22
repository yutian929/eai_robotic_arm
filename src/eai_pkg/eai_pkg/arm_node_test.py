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
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord
from pymycobot import PI_PORT, PI_BAUD  # 当使用树莓派版本的mycobot时，可以引用这两个变量进行MyCobot初始化

class ArmNode_test(Node):
    def __init__(self):
        super().__init__('arm_node_test')
        self.srv = self.create_service(BasicArmCmd, 'basic_arm_cmd_test', self.basic_arm_cmd_callback)
        self.get_logger().info('INFO --- arm_node set-up finished ^.^')


    def basic_arm_cmd_callback(self, request, response):
        # 1.解析request
        basic_arm_cmd = json.loads(request.basic_arm_cmd) 
        # action_type, obj_name, arm_x, arm_y, arm_z = basic_arm_cmd
        
        self.get_logger().info(f'INFO --- received basic_arm_cmd = {basic_arm_cmd}')
        # 执行
        # self.arm_exe()
        mc = MyCobot("/dev/ttyACM0", 115200)

        mc.send_angles([20, 30, 20, 0, 0, 0], 50)

        # 设置等待时间，确保机械臂已经到达指定位置
        time.sleep(2.5)


        response.arm_finished = False
        return response
    def arm_exe(self):
        # 2.执行
        
        mc = MyCobot("/dev/ttyACM0", 115200)

        mc.send_angles([20, 30, 0, 0, 0, 0], 50)

        # 设置等待时间，确保机械臂已经到达指定位置
        time.sleep(2.5)
        # 获取当前头部的坐标以及姿态
        # coords = mc.get_coords()
        # print(coords)

        # mc.send_coords( coords, 80, 1)

        # # 设置等待时间1.5秒
        # time.sleep(1.5)
        self.get_logger().info('INFO --- arm_exe finished ^.^')


def main(args=None):
    rclpy.init(args=args)
    arm_node = ArmNode_test()
    try:
        rclpy.spin(arm_node)
    except KeyboardInterrupt:
        pass
    finally:
        arm_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()