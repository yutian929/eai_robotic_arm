import sys
import rclpy
from rclpy.node import Node
import json
# from eai_interfaces.srv import AiCmd  # 确保这个路径与你的服务定义匹配
from eai_interfaces.srv import BasicArmCmd  # 确保这个路径与你的服务定义匹配
class AICmdClientAsync(Node):
    def __init__(self):
        super().__init__('ai_cmd_client')
        # self.client = self.create_client(AiCmd, 'ai_cmd')
        self.client = self.create_client(BasicArmCmd, 'basic_arm_cmd_test')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务不可用，正在等待...')
        self.req = BasicArmCmd.Request()

    def send_request(self, ai_prompt):
        # self.req.ai_prompt = ai_prompt
        self.req.basic_arm_cmd = ai_prompt
        
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    ai_cmd_client = AICmdClientAsync()
    basic_arm_cmd_json = json.dumps(['grab', 'blue', 10, 20, 3])
#     ai_cmd_client.send_request(
# '''input：layout={"red_block":(482,430,0), "green_block":(450,450,0)}, user_cmd="将红色物块放到绿色物块的右方"
# '''
# )
    ai_cmd_client.send_request(basic_arm_cmd_json)
    while rclpy.ok():
        rclpy.spin_once(ai_cmd_client)
        if ai_cmd_client.future.done():
            try:
                response = ai_cmd_client.future.result()
            except Exception as e:
                ai_cmd_client.get_logger().info('服务调用失败 %r' % (e,))
            else:
                # ai_cmd_client.get_logger().info('服务响应: %r' % (response.ai_basic_arm_cmds,))
                ai_cmd_client.get_logger().info('服务有响应' )
            break

    ai_cmd_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()