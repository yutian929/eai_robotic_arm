
import rclpy
from rclpy.node import Node
from eai_interfaces.srv import SelfLearning  # 确保这个路径与你的服务定义匹配

class SelfLearningNode(Node):
    def __init__(self):
        super().__init__('self_learning_client')
        self.client = self.create_client(SelfLearning, 'self_learning')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务不可用，正在等待...')
        self.req = SelfLearning.Request()

    def send_request(self, robot_actions):
        self.req.robot_actions = robot_actions
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    self_learning_client = SelfLearningNode()
    self_learning_client.send_request('''
layout={"orange_block":(150,250,0), "purple_block":(250,350,0)}, actions={grab('purple_block',150,250,0),place('purple_block',230,370,0)}
    ''')
    while rclpy.ok():
        rclpy.spin_once(self_learning_client)
        if self_learning_client.future.done():
            try:
                response = self_learning_client.future.result()
            except Exception as e:
                self_learning_client.get_logger().info('服务调用失败 %r' % (e,))
            else:
                self_learning_client.get_logger().info('服务响应: %r' % (response.robot_cmds,))
            break

    self_learning_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


