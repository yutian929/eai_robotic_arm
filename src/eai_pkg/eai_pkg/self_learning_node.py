'''
功能：解析robot_actions，恢复成robot_cmds
'''
import rclpy
from rclpy.node import Node
# 导入服务
from eai_interfaces.srv import SelfLearning
import json
from openai import OpenAI



# chat35 = OpenAI(api_key=api_key35)
# chat4 = OpenAI(api_key=api_key4)

class SelfLearningNode(Node):
    def __init__(self, nodename):
        super().__init__(nodename)
        # 创建服务
        self.srv = self.create_service(SelfLearning, 'self_learning', self.self_learning_callback)
        with open('self_learning_prompt.txt', 'r', encoding="utf-8") as file:
            self.self_learning_prompt = file.read()
            self.self_learning_prompt = str(self.self_learning_prompt)

        self.get_logger().info(f'INFO --- {nodename} set-up finished ^.^')

    def self_learning_callback(self, request, response):
        self.get_logger().info(f"INFO --- received robot_actions: {request.robot_actions}")
        model_response = chat35.chat.completions.create(
            model="gpt-3.5-turbo",  # 填写需要调用的模型名称
            messages=[
                    {"role": "system", "content": "你是一个有用的翻译助手，能够把机器人的动作翻译成人类可以听懂的自然语言。"},
                    {"role": "assistant", "content": self.self_learning_prompt},
                    {"role": "user", "content": request.robot_actions}
            ],
            # stream=True,
            # temperature = 0.5,
        )
        ai_result = model_response.choices[0].message.content
        self.get_logger().info(f"INFO --- OUTPUT: \n{ai_result}")
        response.robot_cmds = ai_result
        return response
    
        # 接收prompt
        chat_prompt = request.user_cmd
        self.get_logger().info(f"INFO --- chat prompt: {chat_prompt}")
        model_response = chat4.chat.completions.create(
            model="gpt-3.5-turbo",  # 填写需要调用的模型名称
            messages=[
                    {"role": "system", "content": "你是一个有意思的聊天对象"},
                    # {"role": "assistant", "content": self.basic_prompt_txt},
                    {"role": "user", "content": chat_prompt}
            ],
            # stream=True,
            # temperature = 0.5,
        )
        raw_result = model_response.choices[0].message.content
        # for chunk in model_response:
        #     if chunk.choices and chunk.choices[0]:
        #         if chunk.choices[0].delta and chunk.choices[0].delta.content:
        #             raw_result += chunk.choices[0].delta.content
        self.get_logger().info(f"INFO --- OUTPUT: \n{raw_result}")
        response.ai_chat_response = str(raw_result)
        return response
def main(args=None):
    rclpy.init(args=args)
    self_learning_node = SelfLearningNode('self_learning_node')
    rclpy.spin(self_learning_node)
    self_learning_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
