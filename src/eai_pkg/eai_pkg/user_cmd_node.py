'''
功能：用户输入指令
用法：“将红色方块放在绿色方块的左上角”
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from eai_interfaces.srv import TeachStatus
from eai_interfaces.srv import UserInput

class UserCmdNode(Node):
    def __init__(self):
        super().__init__('user_cmd_node')
        self.user_input_server = self.create_service(UserInput, 'user_input', self.user_input_callback)
        self.teach_status_server = self.create_service(TeachStatus, 'teach_status', self.user_teach_callback)
        self.sub = self.create_subscription(String, '/microphone_input', self.microphone_input_callback, 10)
        self.latest_microphone_input = ""
        self.teach_status = True
        self.get_logger().info('INFO --- user_cmd_node setup finished.')
        # self.user_cmd_input()

    def microphone_input_callback(self, msg):
        if msg.data == "":
            return
        else:
            self.latest_microphone_input = msg.data
            self.get_logger().info(f"INFO --- received microphone_input: {self.latest_microphone_input}")
            if "完了" in self.latest_microphone_input:
                
                self.teach_status = False
                self.get_logger().info(f"INFO --- 教学模式已{'开启' if self.teach_status else '关闭'}。")
                self.latest_microphone_input = ""

    def user_input_callback(self, request, response):
        if request.user_input_req:
            response.user_input_res = self.latest_microphone_input
            # self.get_logger().info(f"INFO --- received center_node_req: {self.latest_microphone_input}")
        else:
            self.get_logger().info(f"INFO --- 找了我却不要input，怎么不要了？")
        self.latest_microphone_input = ""
        return response

    def user_teach_callback(self, request, response):
        if request.teach_status_req:
            response.teach_status_res = self.teach_status
            self.get_logger().info(f"教学模式已{'开启' if self.teach_status else '关闭'}。")
            self.teach_status = True  # 使用self来访问和修改实例变量
        else:
            # self.teach_status = not self.teach_status
            self.get_logger().info(f"找了我却不要是否在教学状态，真奇怪")
        return response


def main(args=None):
    rclpy.init(args=args)
    user_cmd_node = UserCmdNode()
    rclpy.spin(user_cmd_node)
    user_cmd_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
