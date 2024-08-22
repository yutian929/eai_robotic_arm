from dashscope import MultiModalConversation
import json

def call_with_local_file():
    """Sample of use local file.
       linux&mac file schema: file:///home/images/test.png
       windows file schema: file://D:/images/abc.png
    """
    local_file_path1 = 'file://vlm_color_image.jpg'
    local_file_path2 = 'file://image2.jpg'
    user_cmd = "请你把紫色方块拿到黄色方块的左边去。"
    messages = [{
        'role': 'system',
        'content': [{
            'text': '你是一个有用的视觉模块'
        }]}, 
        # {
        # 'role': 'assistant',
        # 'content': [{
        #     'text': '把图像中的筷子框选出来，回复box或者result_image'
        # }]},
        {
        'role':
        'user',
        'content': [
            {
                'image': local_file_path1
            },
            # {
            #     'image': local_file_path2
            # }, 
            {
                'text':  "请你判断用户命令" + user_cmd + "，两个物体的位置关系"
            },
        ]
    }]
    # response = MultiModalConversation.call(model=MultiModalConversation.Models.qwen_vl_chat_v1, messages=messages)
    response = MultiModalConversation.call(model='qwen-vl-max', messages=messages)
    print(response)
    text = response.output.choices[0].message.content[0]['text']
    # print(text)


if __name__ == '__main__':
    call_with_local_file()

# {"status_code": 200, "request_id": "ef4df1c6-0edb-9060-bcf6-58aac1fe99de", "code": "", "message": "", "output": 
#  {"text": null, "finish_reason": null, "choices": 
#   [
#       {"finish_reason": "stop", "message": 
#        {"role": "assistant", "content": 
#         [
#             {"box": "<ref>格子衬衫</ref><box>(30,6),(994,996)</box>"}, 
#             {"result_image": "http://dashscope-result-wlcb.oss-cn-wulanchabu.aliyuncs.com/1d/c0/20240331/fc6de009/ae5c9800-444f-4e00-b24c-29ac2ee11a93.jpg?Expires=1711939187&OSSAccessKeyId=LTAI5tGx7yvUcG32VzcwNgQ6&Signature=2wZpcVS5HjMZeaMZEhgbk4R2WPc%3D"}
#         ]
#         }
#         }
#     ]
# }, 
# "usage": {"input_tokens": 2562, "output_tokens": 22, "image_tokens": 2460}}