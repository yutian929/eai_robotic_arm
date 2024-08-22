import re

def parse_command_string(command_str):
    try:
        # 定义正则表达式来匹配操作、物体名称和坐标 [["grab", ["green_block", 8, 262, 0]], ["place", ["green_block", 73, 212, -1]], ["grab", ["blue_block", 85, 303, 10]], ["place", ["blue_block", 70, 236, 0]]]
        # pattern = re.compile(r'\["([^"]+)", "([^"]+)", (-?\d+), (-?\d+), (-?\d+)\]')
        # # 查找所有匹配项
        # matches = pattern.findall(command_str)
        # # 转换匹配项为指定格式
        # commands = []
        # for match in matches:
        #     # print(match)
        #     action = match[0]
        #     color = match[1]
        #     x = int(match[2])
        #     y = int(match[3])
        #     z = int(match[4])
        #     commands.append([action, color, x, y, z])
        # commands = ast.literal_eval(command_str)

        # 带前缀和后缀的字符串
        # 使用正则表达式匹配所有的[]
        matches = re.findall(r'\[(.*?)\]', command_str)

        # 初始化最终结果列表
        commands = []
        for match in matches:
            # 分割每个匹配到的内容，进一步处理
            items = match.split(',')
            # 去除每个项目的首尾空格和引号，并重新组装为列表
            cleaned_items = [item.strip().strip("'\"") for item in items]
            commands.append(cleaned_items)
        # 打印结果
        print(commands)
    except Exception as e:
        print(f"PARSE ERROR --- {e}")
        commands = []
    return commands

str_ = '''output: = [["grab", "orange_block"], ["place", "purple_block","left"], ["grab", "orange_block"], ["place", "purple_block","under"]] hahahaha'''
# print(parse_command_string(str_))

# import ast

# # 原始字符串
# s = '''[["grab", "blue_block"], ["place", "green_block", "right"]] '''

# # 使用ast.literal_eval安全地评估字符串
# result_list = ast.literal_eval(s)

# # 打印结果
# print(result_list)

import ast
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

print(zyt_parse(str_))