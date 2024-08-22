

'''
功能：从center_node接收ai_prompt = basic_prompt + environment + user_cmd
    传给LLM，得到结果ai_basic_cmds再返回服务给center_node
'''
import rclpy
from rclpy.node import Node
# 导入服务
from eai_interfaces.srv import AiCmd
from eai_interfaces.srv import AiChat
from eai_interfaces.srv import AiJudge
from eai_interfaces.srv import VlmExe
import json
import re
from openai import OpenAI
from dashscope import MultiModalConversation
# from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from http import HTTPStatus
import ast



# chat35 = OpenAI(api_key=api_key35)
# chat4 = OpenAI(api_key=api_key4)

def zyt_parse(input_str):
    result_list = []
    # 双指针依次查找左右[]
    left_index = 0
    right_index = 0
    left_has_been_used = False
    while right_index < len(input_str):
        if input_str[right_index] == '[':
            left_index = right_index
            left_has_been_used = False
        elif input_str[right_index] == ']' and not left_has_been_used:
            # 找到左右[]之间的字符串
            substring = input_str[left_index:right_index+1]
            # 使用ast.literal_eval安全地评估字符串
            sub_list = ast.literal_eval(substring)
            result_list.append(sub_list)
            left_has_been_used = True
        right_index += 1
    # print(result_list)
    return result_list

class AiNode(Node):
    def __init__(self, nodename):

        super().__init__(nodename)
        # 创建服务
        self.srv1 = self.create_service(AiJudge, 'ai_judge', self.judge_callback)
        self.srv2 = self.create_service(AiCmd, 'ai_cmd', self.cmd_callback)
        self.srv3 = self.create_service(AiChat, 'ai_chat', self.chat_callback)
        self.srv4 = self.create_service(VlmExe, 'vlm_exe', self.vlm_callback)

        with open('judge_prompt.txt', 'r', encoding="utf-8") as file:
            self.judge_prompt = file.read()
            self.judge_prompt = str(self.judge_prompt)

        with open('basic_prompt.txt', 'r', encoding="utf-8") as file:
            # self.basic_prompt = file.read().strip()
            self.basic_prompt_txt = file.read()
            self.basic_prompt_txt = str(self.basic_prompt_txt)
        
        with open('vlm_exe_prompt.txt', 'r', encoding="utf-8") as file:
            self.vlm_exe_prompt = file.read()
            self.vlm_exe_prompt = str(self.vlm_exe_prompt)

        self.get_logger().info(f'INFO --- {nodename} set-up finished ^.^')

    def judge_callback(self, request, response):
        self.get_logger().info(f"INFO --- received user_cmd: {request.user_cmd}")
        model_response = chat35.chat.completions.create(
            model="gpt-3.5-turbo",  # 填写需要调用的模型名称
            messages=[
                    {"role": "system", "content": self.judge_prompt},
                    # {"role": "assistant", "content": self.basic_prompt_txt},
                    {"role": "user", "content": request.user_cmd}
            ],
            # stream=True,
            temperature = 0,
        )
        ai_result = model_response.choices[0].message.content
        if 'learn' in ai_result.lower():
            response.ai_judge = str(ai_result)
        elif 'exe' in ai_result.lower():
            response.ai_judge = "exe"
        elif 'chat' in ai_result.lower():
            response.ai_judge = "chat"
        else:
            response.ai_judge = ai_result
        self.get_logger().info(f"INFO --- 判断结果: \n{ai_result}")
        return response
    
    def vlm_callback(self, request, response):
        cmd_prompt = request.user_cmd
        # cmd_prompt = "user_cmd:" + cmd_prompt
        print(cmd_prompt)
        layout = request.layout
        # layout = 'layout:' + layout 
        print(layout)
        with open('history_prompt.txt', 'r', encoding="utf-8") as file:
            self.history_prompt = file.read()
            self.history_prompt = str(self.history_prompt)
        # 接收图片
        cv_bridge = CvBridge()
        cv_img = cv_bridge.imgmsg_to_cv2(request.color_img, desired_encoding="passthrough")
        file_path = "vlm_color_image.jpg"
        cv2.imwrite(file_path, cv_img)

        str_what_need_to_send_to_vlm = "user_cmd="+cmd_prompt
        # str_what_need_to_send_to_vlm = '请把图片中所有小方块的整体布局，和每个小方块之间的相对位置详细地描述一下'
        self.get_logger().info(f"INFO --- vlm_call: {str_what_need_to_send_to_vlm}")
        vlm_result = self.vlm_call(str_what_need_to_send_to_vlm, file_path)

        self.get_logger().info(f"INFO --- vlm_call result: {vlm_result}")

        str_what_need_to_send_gpt = "layout="+layout + ", user_cmd="+cmd_prompt+", vlm_advice=" + vlm_result
        model_response = chat4.chat.completions.create(
            # model="gpt-4",  # 填写需要调用的模型名称
            model="gpt-3.5-turbo",  # 填写需要调用的模型名称
            messages=[
                    {"role": "system", "content": self.basic_prompt_txt},
                    {"role": "assistant", "content": self.history_prompt},
                    {"role": "user", "content": str_what_need_to_send_gpt}
            ],
            # stream=True,
            temperature = 0,
        )
        raw_result = model_response.choices[0].message.content
        self.get_logger().info(f"INFO --- raw_result: \n{raw_result}")
        if "ok" in raw_result.lower():
            response.vlm_advice = "ok"
        elif "wrong" in raw_result.lower():
            response.vlm_advice = "wrong"
        else:
            packed_result = zyt_parse(raw_result)
            # packed_result = eval(raw_result)
            packed_result = json.dumps(packed_result)
            response.vlm_advice = packed_result
        # response.vlm_advice = str(result)
        self.get_logger().info(f"INFO --- vlm_advice: {response.vlm_advice}")
        return response
 
    def chat_callback(self, request, response):
        # 接收prompt，还有图片-------------------------------------
        chat_prompt = request.user_cmd
        # 将sensorImage转换为OpenCV格式的图像
        cv_bridge = CvBridge()
        cv_img = cv_bridge.imgmsg_to_cv2(request.color_img, desired_encoding="passthrough")

        # 将OpenCV格式的图像保存到本地文件
        file_path = "chat_color_image.jpg"  # 可以根据需要修改保存的文件名和格式
        cv2.imwrite(file_path, cv_img)  
        self.get_logger().info(f"INFO --- chat prompt: {chat_prompt}")
        result = self.conversation_call(chat_prompt, file_path)
        response.ai_chat_response = str(result)
        return response
    
    def conversation_call(self, chat_prompt, file_path):
        user_cmd = chat_prompt
        local_file_path = 'file://' + file_path
        messages = [{
        'role': 'system',
        'content': [{
            'text': '你是一个聊天小助手，用户可能会结合图片跟你聊天，也可能不会，如果没有问起图片相关的内容，就不需要回复和图片有关的东西，聊聊天就行'
        }]}, 
        {
        'role':'user',
        'content': [
            {
                'image': local_file_path
            },
            {
                'text': user_cmd
            },
        ]}]
        response = MultiModalConversation.call(model='qwen-vl-max', messages=messages)
        # if response.status_code == HTTPStatus.OK:
            # print(response)
        # else:
            # print(response.code)  # The error code.
            # print(response.message)  # The error message.
        result = response.output.choices[0].message.content[0]['text']
        self.get_logger().info(f"INFO --- conversation_call result: {result}")
        return result

    def vlm_call(self, cmd_prompt, file_path):
        user_cmd = cmd_prompt
        user_cmd = "请你判断是否已经满足" + user_cmd + "，给出简单的推理过程"
        local_file_path = 'file://' + file_path
        messages = [
        {
        'role': 'system',
        'content': [{
            # 'text': self.vlm_exe_prompt
            'text': '你是一个有用的视觉模块'
            }]
        }, 
        {
        'role':'user',
        'content': [
            {
                'image': local_file_path
            },
            # 备忘：加入第二张照片
            # {
            #     'image': local_file_path
            # },
            {
                'text': user_cmd
            },
        ]}]
        response = MultiModalConversation.call(model='qwen-vl-max', messages=messages)
        # if response.status_code == HTTPStatus.OK:
            # print(response)
        # else:
            # print(response.code)  # The error code.
            # print(response.message)  # The error message.
        result = response.output.choices[0].message.content[0]['text']
        # self.get_logger().info(f"INFO --- vlm_call result: {result}")
        return result
    
    # 用这个函数来处理接收到的prompt
    def cmd_callback(self, request, response):
        # 接收prompt
        cmd_prompt = request.ai_prompt
        self.get_logger().info(f"INFO --- exe prompt: {cmd_prompt}")
        with open('history_prompt.txt', 'r', encoding="utf-8") as file:
            self.history_prompt = file.read()
            self.history_prompt = str(self.history_prompt)
        all_prompt = self.basic_prompt_txt + self.history_prompt
        model_response = chat4.chat.completions.create(
            model="gpt-3.5-turbo",  # 填写需要调用的模型名称
            # model="gpt-4",
            messages=[
                    {"role": "system", "content": "你是一个功能强大的机械臂，能够根据用户指令做出合适的动作。"},
                    {"role": "assistant", "content": self.basic_prompt_txt + self.history_prompt},
                    {"role": "user", "content": cmd_prompt}
            ],
            # stream=True,
            temperature = 0,
        )
        raw_result = model_response.choices[0].message.content
        # for chunk in model_response:
        #     if chunk.choices and chunk.choices[0]:
        #         if chunk.choices[0].delta and chunk.choices[0].delta.content:
        #             raw_result += chunk.choices[0].delta.content
        self.get_logger().info(f"INFO --- GPT result: \n{raw_result}")
        if 'wrong' in raw_result.lower():
            response.ai_basic_arm_cmds = json.dumps("wrong")
            return response
        elif 'ok' in raw_result.lower():
            response.ai_basic_arm_cmds = json.dumps("ok")
            return response
        else:
            packed_result = zyt_parse(raw_result)
            if len(packed_result) == 0:
                response.ai_basic_arm_cmds = json.dumps("wrong")
                return response
            packed_result = json.dumps(packed_result)
            response.ai_basic_arm_cmds = packed_result
            # self.get_logger().info(f"\nOUTPUT: ai_basic_arm_cmds = {response.ai_basic_arm_cmds}")
            return response
       
    
def main(args=None):
    rclpy.init(args=args)
    ai_node = AiNode('ai_node')
    rclpy.spin(ai_node)
    # 销毁节点
    ai_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

