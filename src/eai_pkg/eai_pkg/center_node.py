import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from eai_interfaces.srv import AiJudge, LayoutAnalysis, AiCmd, BasicArmCmd, RsDeproject, AiChat, TeachStatus, UserInput, ColorImg, SpeakerInfo, VlmExe
from cv_bridge import CvBridge
import cv2
import json
import time


class CenterNode(Node):
    def __init__(self):
        super().__init__('center_node')
        self.user_input_client = self.create_client(UserInput, 'user_input')
        # self.create_subscription(String, '/user_cmd', self.user_cmd_received_callback, 10)
        self.ai_judge_client = self.create_client(AiJudge, 'ai_judge')
        self.layout_analysis_client = self.create_client(LayoutAnalysis, 'layout_analysis')
        self.ai_cmd_client = self.create_client(AiCmd, 'ai_cmd')
        self.basic_arm_cmd_client = self.create_client(BasicArmCmd, 'basic_arm_cmd')
        self.ai_chat_client = self.create_client(AiChat, 'ai_chat')
        self.teach_status_client = self.create_client(TeachStatus, 'teach_status')
        self.color_img_client = self.create_client(ColorImg, 'color_img')
        self.speaker_info_client = self.create_client(SpeakerInfo, 'speaker_info')
        self.vlm_exe_client = self.create_client(VlmExe, 'vlm_exe')
        self.status = ""

        self.layout_type = 'arm_nxyz'
        # "c_nxywhd"
        # "rs_nxyd"
        # "rs_nxyz"
        # 'arm_nxyz'
        self.former_layout = None
        self.latest_layout = None  # {'name':( defined by laout type)}
        self.layout_threshold = 12  # arm_xyz diff 10mm

        self.first_stable_layout = None
        self.last_stable_layout = None
        self.stable_layout_history = []
        self.stable_layout_cnt = 0

        self.bridge = CvBridge()
        self.latest_color_img = None
        self.vlm_exe_advice = None
        self.latest_c_nxywhd = None
        self.former_c_nxywhd = None

        self.user_cmd = None  # str
        self.user_former_cmd = None  # str

        self.basic_arm_cmds = None  # [['action_type', ('obj_name', x, y, z)],[]]
        self.basic_arm_cmd_id = 0

        self.get_logger().info('INFO --- center_node set-up finished ^.^')
        self.send_user_input_request()


    def send_user_input_request(self):
        req = UserInput.Request()
        req.user_input_req = True
        # self.get_logger().info('INFO --- STEP0: Req for user_input')
        while not self.user_input_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('WARN --- user_input service not available, waiting again...')
        time.sleep(0.5)
        future = self.user_input_client.call_async(req)
        future.add_done_callback(self.user_input_received_callback)

    def user_input_received_callback(self, future):  # 1.receive user_cmd
        self.user_cmd = future.result().user_input_res  # str
        # self.get_logger().info(f'INFO --- STEP1: Received user_cmd: {self.user_cmd}')
        if self.user_cmd:  # not None
            if self.user_former_cmd:  # existed user_former_cmd, need to judge whether same
                if self.user_cmd == self.user_former_cmd:
                    self.get_logger().info(f'INFO --- STEP1: User cmd not changed, do nothing.')
                    time.sleep(1)
                    self.send_user_input_request()
                else:
                    self.user_former_cmd = self.user_cmd  # set user_former_cmd for next judge
                    self.get_logger().info(f'INFO --- STEP1: User cmd changed, next step.')
                    self.send_ai_judge_request()
            else:
                self.user_former_cmd = self.user_cmd  # set user_former_cmd for next judge
                self.get_logger().info(f'INFO --- STEP1: Received new user_cmd: {self.user_cmd}')
                self.send_ai_judge_request()
        else:
            time.sleep(3)
            self.get_logger().info(f'INFO --- STEP1: Received None user_cmd, req again.')
            self.user_former_cmd = None  # reset user_former_cmd for next judge
            self.send_user_input_request()
    def send_ai_judge_request(self):  # 2.send req user_cmd to srv-ai_judge and wait for ai_judge
        while not self.ai_judge_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('WARN --- ai_judge service not available, waiting again...')
        req = AiJudge.Request()
        req.user_cmd = self.user_cmd
        self.get_logger().info(f'INFO --- STEP2: Send ai_judge request: {req.user_cmd}')
        future = self.ai_judge_client.call_async(req)
        future.add_done_callback(self.ai_judge_received_callback)

    def ai_judge_received_callback(self, future):  # 3.received ai_judge
        try:
            response = future.result()
            ai_judge = response.ai_judge  # str
            self.get_logger().info(f'INFO --- STEP3: Received ai_judge: {ai_judge}')
            if "exe" in ai_judge.lower():
                self.get_logger().info(f'INFO --- STEP4: Start exe cmds progress')
                self.status = "exe"
                self.exe_start()
            elif "chat" in ai_judge.lower():
                self.get_logger().info(f'INFO --- STEP4: Start happy chat progress')
                self.status = "chat"
                self.chat_start()
            elif "learn" in ai_judge.lower():
                self.get_logger().info(f'INFO --- STEP4: Start self learn progress')
                self.status = "learn"
                self.user_cmd = ai_judge[6:]
                self.learn_start()
            else:
                self.get_logger().info(f'INFO --- STEP4: No status fit, return')
                self.status = ""
                self.basic_flow_finished()
        except Exception as e:
            self.get_logger().error('ERROR --- Service call failed %r' % (e,))

    def basic_flow_finished(self):
        self.status = ""
        self.former_layout = None
        self.latest_layout = None  # {'name':( defined by laout type)}

        self.first_stable_layout = None
        self.last_stable_layout = None
        self.stable_layout_history = []
        self.stable_layout_cnt = 0

        self.basic_arm_cmds = None  # [['action_type', ('obj_name', x, y, z)],[]]
        self.basic_arm_cmd_id = 0
        self.vlm_exe_advice = None
        self.latest_color_img = None
        self.get_logger().info(f'INFO --- *** Basic flow finished, return ***')
        self.send_user_input_request()

    # ***************************************# self.status = "exe" #***************************************#
    def exe_start(self):  # exe0.declare basic info
        self.get_logger().info(f'INFO --- exe0. exe_start')
        self.exe_send_layout_analysis_request()

    def exe_send_layout_analysis_request(self):  # exe1.send layout req
        while not self.layout_analysis_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('WARN --- layout_analysis service not available, waiting again...')
        req = LayoutAnalysis.Request()
        req.ask_for_layout = self.layout_type
        self.get_logger().info(f'INFO --- exe1. ask Layout analysis ask for special type: {self.layout_type}')
        future = self.layout_analysis_client.call_async(req)
        future.add_done_callback(self.exe_layout_analysis_received_callback)

    def exe_layout_analysis_received_callback(self, future):  # exe2.received layout
        try:
            response = future.result()
            layout = response.layout
            layout = json.loads(layout)  # layout now is a dict = {'name', (cx, cy, depth)...}
            self.latest_layout = layout  # 3D
            self.get_logger().info(f'INFO --- exe2: received Layout analysis: {layout}')
            if layout:
                if self.exe_is_layout_stable():
                    layout_json = json.dumps(self.latest_layout)
                    self.exe_send_ai_cmd_request(layout_json)
                else:
                    self.get_logger().warn("WARN --- layout isn't stable.")
                    self.exe_send_layout_analysis_request()
            else:
                self.get_logger().info('INFO --- No layout found.')
                self.exe_send_layout_analysis_request()

        except Exception as e:
            self.get_logger().error('ERROR --- Service call failed, maybe unstable %r' % (e,))

    def exe_is_layout_stable(self):
        if not self.former_layout:
            self.former_layout = self.latest_layout
            return False
        # Step 1: Check if the keys of both layouts are identical
        if set(self.latest_layout.keys()) != set(self.former_layout.keys()):
            self.former_layout = self.latest_layout
            return False  # Layouts are unstable due to key differences
        # Step 2: Check if the values for each key are within the allowed range
        for key in self.latest_layout.keys():
            # Extracting coordinates for each block in both layouts
            x_latest, y_latest, z_latest = self.latest_layout[key]
            x_former, y_former, z_former = self.former_layout[key]

            # Calculating the differences
            diff_x = abs(x_latest - x_former)
            diff_y = abs(y_latest - y_former)
            diff_z = abs(z_latest - z_former)

            # Checking if any difference exceeds the allowed threshold
            if diff_x > self.layout_threshold or diff_y > self.layout_threshold or diff_z > self.layout_threshold:
                self.former_layout = self.latest_layout
                return False  # Layouts are unstable due to coordinate differences
        self.former_layout = self.latest_layout
        return True  # Layouts are stable

    def exe_send_ai_cmd_request(self, layout):  # exe3. send ai_cmd req
        ai_prompt = '\n' + 'input: layout=' + layout + ', user_cmd=' + self.user_cmd + '。'
        # layout = json.loads(layout)  # layout now is a dict = {'name', (cx, cy, depth)...}
        # self.latest_layout = layout  # 3D
        self.get_logger().info(f'INFO --- exe3: form and send ai_prompt.')
        # layout_2D = {key: val[:2] for key, val in layout.items()}
        # layout = json.dumps(layout)
        # ai_prompt = self.basic_prompt + '\n' + 'input: layout=' + layout_2D + ', user_cmd=' + self.user_cmd + '。'

        while not self.ai_cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('WARN --- ai_cmd service not available, waiting again...')

        req = AiCmd.Request()
        req.ai_prompt = ai_prompt
        future = self.ai_cmd_client.call_async(req)
        future.add_done_callback(self.exe_ai_basic_arm_cmds_received_callback)

    def exe_ai_basic_arm_cmds_received_callback(self, future):  # exe4. received ai_basic_arm_cmds(json-str)
        try:
            response = future.result()
            # self.get_logger().info(f'INFO ---*STEP5: json_ai_basic_cmds = {response.ai_basic_arm_cmds}, {type(response.ai_basic_arm_cmds.lower())}')
            if 'ok' in response.ai_basic_arm_cmds.lower():
                self.get_logger().info(f'INFO --- exe4. AI response = ok, exe basic flow finished.')
                self.basic_flow_finished()

            elif 'wrong' in response.ai_basic_arm_cmds.lower():
                self.get_logger().info(f'INFO --- exe4. AI response = wrong, exe basic flow finished.')
                self.basic_flow_finished()

            else:
                basic_arm_cmds = json.loads(response.ai_basic_arm_cmds)  # basic_arm_cmds is a list[list]
                self.basic_arm_cmds = basic_arm_cmds  # for arm_node to execute basic_arm_cmds one by one
                self.get_logger().info(f'INFO --- exe4. Received ai basic arm cmds  = {basic_arm_cmds}, go on')
                self.exe_optimize_arm_commands(basic_arm_cmds)
        except Exception as e:
            self.get_logger().error('ERROR --- Service call failed %r' % (e,))

    def exe_optimize_arm_commands(self, basic_arm_cmds):  # 后续结合Depth图像进行优化前后左右要保证至少差20mm
        self.exe_execute_arm_commands(basic_arm_cmds)

    def exe_execute_arm_commands(self, basic_arm_cmds):  # exe5.exe_execute_arm_commands one by one, send basic_arm_cmd
        if basic_arm_cmds:  # (str-json)
            self.basic_arm_cmd_id += 2
            basic_arm_cmd_grab = self.basic_arm_cmds[0]  # self.basic_arm_cmds is a list = [['grab', 'grab_obj_name'], ['place', 'relative_obj_name', 'front/back/left/right/on/under']...]
            basic_arm_cmd_place = self.basic_arm_cmds[1]
            self.get_logger().info(
                f'INFO --- exe5: Execute basic_arm_cmd: {basic_arm_cmd_grab}{basic_arm_cmd_place} ID: {self.basic_arm_cmd_id}')
            
            if basic_arm_cmd_grab[0] == 'grab':
                grab_obj_name = basic_arm_cmd_grab[1]
            else:
                self.get_logger().error(f"basic_arm_cmd_grab[0] != 'grab'")
                self.basic_flow_finished()
            if basic_arm_cmd_place[0] == 'place':
                relative_obj_name = basic_arm_cmd_place[1]
                fblrou = basic_arm_cmd_place[2]
            else:
                self.get_logger().error(f"basic_arm_cmd_place[0] != 'place'")
                self.basic_flow_finished()
            
            grab_obj_arm_xyz = self.latest_layout[grab_obj_name]
            relative_obj_arm_xyz = self.latest_layout[relative_obj_name]
            place_obj_arm_xyz = relative_obj_arm_xyz
            objs_interval = 40
            if fblrou == 'front':
                place_obj_arm_xyz[1] += objs_interval
            elif fblrou == 'back':
                place_obj_arm_xyz[1] -= objs_interval
            elif fblrou == 'left':
                place_obj_arm_xyz[0] -= objs_interval
            elif fblrou == 'right':
                place_obj_arm_xyz[0] += objs_interval
            elif fblrou == 'on':
                place_obj_arm_xyz[2] += objs_interval
            elif fblrou == 'under':
                place_obj_arm_xyz[2] -= objs_interval
            else:
                self.get_logger().warn('fblrou is not valid')
            
            
            send_to_arm_node_cmd_grab = ['grab', grab_obj_name, grab_obj_arm_xyz[0], grab_obj_arm_xyz[1], grab_obj_arm_xyz[2]]
            send_to_arm_node_cmd_place = ['place', grab_obj_name, place_obj_arm_xyz[0], place_obj_arm_xyz[1], place_obj_arm_xyz[2]]
            
            send_to_arm_node_cmd = json.dumps([send_to_arm_node_cmd_grab, send_to_arm_node_cmd_place])
            while not self.basic_arm_cmd_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('WARN --- basic_arm_cmd service not available, waiting again...')
            req = BasicArmCmd.Request()
            req.basic_arm_cmd = send_to_arm_node_cmd
            future = self.basic_arm_cmd_client.call_async(req)
            future.add_done_callback(self.exe_basic_arm_cmd_received_callback)
        else:
            # self.basic_arm_cmd_id = 0
            # self.exe_send_layout_analysis_request()  # exe7. recurrent
            self.basic_flow_finished()  # exe8. finished

    def exe_basic_arm_cmd_received_callback(self, future):  # exe6. jeston exe
        try:
            response = future.result()
            if response.arm_finished:
                self.get_logger().info(f'INFO --- exe6: Arm finished basic_arm_cmd ID: {self.basic_arm_cmd_id}, now we need to get latest layout')
                self.basic_arm_cmds = self.basic_arm_cmds[2:]
                self.exe_arm_cmd_get_latest_layout()
            else:
                self.get_logger().info('INFO --- Arm not finished')
        except Exception as e:
            self.get_logger().error('ERROR --- Service call failed %r' % (e,))
    
    def exe_arm_cmd_get_latest_layout(self):
        while not self.layout_analysis_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('WARN --- layout_analysis service not available, waiting again...')
        req = LayoutAnalysis.Request()
        req.ask_for_layout = self.layout_type
        self.get_logger().info(f'INFO --- exe7. arm_cmd ask Layout analysis ask for special type: {self.layout_type}')
        time.sleep(0.5)  # wait for layout being stable
        future = self.layout_analysis_client.call_async(req)
        future.add_done_callback(self.exe_arm_cmd_received_latest_layout_callback)

    def exe_arm_cmd_received_latest_layout_callback(self, future):  # exe2.received layout
        try:
            response = future.result()
            layout = response.layout
            layout = json.loads(layout)  # layout now is a dict = {'name', (cx, cy, depth)...}
            self.latest_layout = layout  # 3D
            self.get_logger().info(f'INFO --- exe8: received Layout analysis: {layout}')
            if layout:
                if self.exe_is_layout_stable():
                    # layout_json = json.dumps(self.latest_layout)
                    # self.exe_send_ai_cmd_request(layout_json)
                    # latest layout has been renewed
                    self.exe_execute_arm_commands(self.basic_arm_cmds)
                else:
                    self.get_logger().warn("WARN --- layout isn't stable.")
                    self.exe_arm_cmd_get_latest_layout()
            else:
                self.get_logger().info('INFO --- No layout found.')
                self.exe_arm_cmd_get_latest_layout()

        except Exception as e:
            self.get_logger().error('ERROR --- exe_arm_cmd_received_latest_layout_callback Service call failed, maybe unstable %r' % (e,))


    # ***************************************# self.status = "chat" #***************************************#
    def chat_start(self):
        self.get_logger().info(f'INFO --- chat0. chat_start')
        self.chat_get_color_img()
        # self.chat_send_ai_user_cmd_request()

    def chat_get_color_img(self):
        self.get_logger().info(f'INFO --- chat1.1 get color_img')
        req = ColorImg.Request()
        req.color_img_req = True
        future = self.color_img_client.call_async(req)
        future.add_done_callback(self.chat_color_img_received_callback)

    # def chat_color_img_received_callback(self, future):
    #     response = future.result()
    #     self.latest_color_img = response.color_img_res
    #     # self.get_logger().info(f'INFO --- chat1.2 received color_img')
    #     self.get_logger().info(f'INFO --- chat1.2 received color_img type = {type(response.color_img_res)}')
    #     self.chat_send_ai_user_cmd_request()

    def chat_color_img_received_callback(self, future):
        try:
            response = future.result()
            # 使用CvBridge将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(response.color_img_res, "bgr8")
            # 显示图像
            cv2.imshow("chat_color_img", cv_image)
            cv2.waitKey(30)  # 等待至少1毫秒，以便图像显示
            self.latest_color_img = cv_image
            # 如果你希望图像窗口保持打开，直到用户按键，则可以使用 cv2.waitKey(0)
            self.chat_send_ai_user_cmd_request()
            
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    # def chat_send_ai_user_cmd_request(self):
    #     self.get_logger().info(f'INFO --- chat1.2 send user_cmd {self.user_cmd}')
    #     req = AiChat.Request()
    #     req.user_cmd = self.user_cmd
    #     req.color_img = self.latest_color_img
    #     future = self.ai_chat_client.call_async(req)
    #     future.add_done_callback(self.chat_ai_user_cmd_received_callback)

    def chat_send_ai_user_cmd_request(self):
        self.get_logger().info(f'INFO --- chat1.2 send user_cmd {self.user_cmd}')
        req = AiChat.Request()
        req.user_cmd = self.user_cmd
        try:
            # 假设self.latest_color_img已经是OpenCV格式的图像
            # 将OpenCV格式的图像转换回sensor_msgs/Image格式
            req.color_img = self.bridge.cv2_to_imgmsg(self.latest_color_img, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert OpenCV image to ROS image message: {e}')
            return  # 转换失败，直接返回

        future = self.ai_chat_client.call_async(req)
        future.add_done_callback(self.chat_ai_user_cmd_received_callback)

    def chat_ai_user_cmd_received_callback(self, future):
        response = future.result()
        ai_chat_response = response.ai_chat_response
        # ai_chat_response = ai_chat_response.replace(' ', '').replace('\n', '').replace('\r', '').replace('\t', '')
        self.get_logger().info(f'INFO --- chat2. received ai_chat_response = {ai_chat_response}')
        self.chat_send_speaker_info(ai_chat_response)  # tts
    
    def chat_send_speaker_info(self, ai_chat_response):
        self.get_logger().info(f'INFO --- chat3 send speaker_info')
        req = SpeakerInfo.Request()
        req.speaker_info_req = ai_chat_response
        future = self.speaker_info_client.call_async(req)
        future.add_done_callback(self.chat_speaker_info_received_callback)
    
    def chat_speaker_info_received_callback(self, future):
        response = future.result()
        if response.speaker_info_res:
            self.basic_flow_finished()

    # ***************************************# self.status = "learn" #***************************************#
    def learn_start(self):
        self.get_logger().info(f'INFO --- learn0. learn_start, task = {self.user_cmd}')
        self.learn_send_layout_analysis_request()

    def learn_send_layout_analysis_request(self):
        while not self.layout_analysis_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('WARN --- layout_analysis service not available, waiting again...')
        req = LayoutAnalysis.Request()
        req.ask_for_layout = self.layout_type
        self.get_logger().info(f'INFO --- learn1. send layout_analysis req')
        time.sleep(0.2)
        future = self.layout_analysis_client.call_async(req)
        future.add_done_callback(self.learn_layout_analysis_received_callback)

    def learn_layout_analysis_received_callback(self, future):  # exe2.received layout
        try:
            response = future.result()
            layout = response.layout
            layout = json.loads(layout)
            self.latest_layout = layout  # 3D
            # self.get_logger().info(f'INFO --- learn2: received Layout analysis: {layout}')
            if layout:
                if self.learn_is_layout_stable():
                    self.stable_layout_cnt += 1
                    self.get_logger().info(
                        f'INFO --- learn2: received stable_layout_cnt = {self.stable_layout_cnt}, stable layout analysis: {layout}')

                    # learn_v1
                    if not self.first_stable_layout:
                        self.first_stable_layout = self.latest_layout
                    else:
                        self.last_stable_layout = self.latest_layout

                    # learn_v2
                    self.stable_layout_history.append(self.latest_layout)

                    if self.stable_layout_cnt % 10 == 0:
                        self.learn_send_user_teach_finished_request()
                    else:
                        self.learn_send_layout_analysis_request()
                else:
                    self.get_logger().warn("WARN --- layout isn't stable.")
                    self.learn_send_layout_analysis_request()
            else:
                self.get_logger().info('INFO --- No layout found.')
                self.learn_send_layout_analysis_request()

        except Exception as e:
            self.get_logger().error('ERROR --- Service call failed, maybe unstable %r' % (e,))

    def learn_send_user_teach_finished_request(self):
        req = TeachStatus.Request()
        req.teach_status_req = True
        self.get_logger().info(f'INFO --- learn3: ask for whether user teaching finished')
        future = self.teach_status_client.call_async(req)
        future.add_done_callback(self.learn_teach_status_received_callback)

    def learn_teach_status_received_callback(self, future):
        response = future.result()
        self.get_logger().info(f'INFO --- learn4: received tech_status: {response.teach_status_res}')
        if not response.teach_status_res:
            self.get_logger().info(f'INFO --- learn5: user teaching finished, self_learning begin')
            # self.learn_self_learn_v1()
            self.learn_self_learn_v3()
        else:
            self.get_logger().info(f'INFO --- learn5: user teaching not finished, send layout_analysis req')
            self.learn_send_layout_analysis_request()

    def learn_self_learn_v3(self):
        # 确保有足够的稳定布局历史记录来分析动作顺序
        def determine_relative_position(moved_object_coords, stationary_object_coords):
            x_diff = moved_object_coords[0] - stationary_object_coords[0]
            y_diff = moved_object_coords[1] - stationary_object_coords[1]
            z_diff = moved_object_coords[2] - stationary_object_coords[2]

            if abs(z_diff) > max(abs(x_diff), abs(y_diff)):
                return 'on' if z_diff > 0 else 'under'
            elif abs(x_diff) > abs(y_diff):
                return 'right' if x_diff > 0 else 'left'
            else:
                return 'front' if y_diff > 0 else 'back'
            
        if len(self.stable_layout_history) > 1:
            self.get_logger().info(f'INFO --- learn6.1: self learning begin with {len(self.stable_layout_history)} stable layouts')

            # 初始化一个空列表来存储推断出的动作指令
            inferred_actions = []

            # 遍历稳定布局历史记录中的每一对连续布局
            for i in range(len(self.stable_layout_history) - 1):
                current_layout = self.stable_layout_history[i]
                next_layout = self.stable_layout_history[i + 1]

                # 对于当前布局中的每个物体，检查它是否在下一个布局中位置有变化
                for object_name, current_coords in current_layout.items():
                    if object_name in next_layout:
                        next_coords = next_layout[object_name]

                        # 计算位置差异
                        diff = [abs(cc - nc) for cc, nc in zip(current_coords, next_coords)]

                        # 如果任一坐标差异超过阈值，生成相应的动作指令
                        if any(d > self.layout_threshold for d in diff):  # 现在发现object_name, next_coords跟当前的current_layout有差异
                            grab_action = ['grab', object_name]
                            nearest_object_name, nearest_object_coords = None, None
                            min_distance = float('inf')
                            # 在current_layout中寻找next_coords最近的物体，也就是放在原来的哪个旁边，也可能是自己
                            for cur_object_name, cur_coords in current_layout.items():
                                distance = ((next_coords[0] - cur_coords[0])**2 + (next_coords[1] - cur_coords[1])**2 + (next_coords[2] - cur_coords[2])**2)**0.5
                                if distance < min_distance:
                                    nearest_object_name = cur_object_name
                                    nearest_object_coords = cur_coords
                                    min_distance = distance
                            # 找到了最近的物体和位置，下面判断前后左右上下
                            if nearest_object_name:
                                relative_pos = determine_relative_position(next_coords, nearest_object_coords)
                            else:
                                self.get_logger().warn(f'WARN --- learn6.2: no nearest object found for {object_name}')
                            
                            palce_action = ['place', nearest_object_name, relative_pos]
                            inferred_actions.append(grab_action)
                            inferred_actions.append(palce_action)
                    # else:  # current doesn't have but next have, we should compare it with the first stable layout
                    #     if object_name in self.first_stable_layout:
                    #         first_coords = self.first_stable_layout[object_name]
                    #         diff = [abs(fc - lc) for fc, lc in zip(first_coords, current_coords)]
                    #         if any(d > self.layout_threshold for d in diff):
                    #             grab_action = ['grab', object_name, first_coords[0], first_coords[1], first_coords[2]]
                    #             palce_action = ['place', object_name, current_coords[0], current_coords[1], current_coords[2]]
                    #             inferred_actions.append(grab_action)
                    #             inferred_actions.append(palce_action)

            # self.get_logger().info(f'INFO --- learn6.1: inferred actions No.1 = {inferred_actions}')

            # 使用列表推导式提取所有的object_name
            have_been_changed_object_names = []
            for action in inferred_actions:
                if action[0] == 'grab':
                    have_been_changed_object_names.append(action[1])

            # 使用set()函数将列表转换为集合，自动去除重复项
            # have_been_changed_object_names = set(have_been_changed_object_names)
            self.get_logger().info(f'INFO --- learn6.2 have_been_changed_object_names = {have_been_changed_object_names}')
            self.get_logger().info(f'INFO --- learn7: interval inferred actions = {inferred_actions}')

            if self.first_stable_layout and self.last_stable_layout:
                # 对于first_stable_layout中的每个物体，检查是否需要移动
                for object_name, first_coords in self.first_stable_layout.items():
                    if object_name in have_been_changed_object_names:
                        continue
                    if object_name in self.last_stable_layout:  # first中有，last中也有，而且没动过
                        last_coords = self.last_stable_layout[object_name]
                        # 计算坐标差异
                        diff = [abs(fc - lc) for fc, lc in zip(first_coords, last_coords)]

                        # 如果任一坐标差异超过阈值，生成grab和place命令
                        if any(d > self.layout_threshold for d in diff):
                            grab_action = ['grab', object_name]
                            nearest_object_name, nearest_object_coords = None, None
                            min_distance = float('inf')
                            next_coords = self.last_stable_layout[object_name]
                            # 在first_layout中寻找last_coords最近的物体，也就是放在原来的哪个旁边，也可能是自己
                            for cur_object_name, cur_coords in current_layout.items():
                                distance = ((next_coords[0] - cur_coords[0])**2 + (next_coords[1] - cur_coords[1])**2 + (next_coords[2] - cur_coords[2])**2)**0.5
                                if distance < min_distance:
                                    nearest_object_name = cur_object_name
                                    nearest_object_coords = cur_coords
                                    min_distance = distance
                            # 找到了最近的物体和位置，下面判断前后左右上下
                            if nearest_object_name:
                                relative_pos = determine_relative_position(next_coords, nearest_object_coords)
                            else:
                                self.get_logger().warn(f'WARN --- learn6.2: no nearest object found for {object_name}')
                            
                            palce_action = ['place', nearest_object_name, relative_pos]
                            inferred_actions.append(grab_action)
                            inferred_actions.append(palce_action)
            self.get_logger().info(f'INFO --- learn7: inferred actions = {inferred_actions}')

            self.learn_write_history(inferred_actions)
        else:
            self.get_logger().info(
                'INFO --- learn6: Not enough stable layouts for learning, sending layout analysis request')
            self.learn_send_layout_analysis_request()

    def learn_write_history(self, exe_basic_arm_cmds):
        input_ = json.dumps(self.first_stable_layout)
        user_cmd_ = self.user_cmd
        output_ = json.dumps(exe_basic_arm_cmds)
        history_ = "input: layout=" + input_ + ", user_cmd=" + user_cmd_ + ". output: " + output_
        self.get_logger().info(f'INFO --- learn8: generate history = {history_}')
        with open("history_prompt.txt", "a") as file:
            file.write("\n" + history_)
        self.basic_flow_finished()

    def learn_is_layout_stable(self):
        if not self.former_layout:
            self.former_layout = self.latest_layout
            return False
        # Step 1: Check if the keys of both layouts are identical
        if set(self.latest_layout.keys()) != set(self.former_layout.keys()):
            self.former_layout = self.latest_layout
            return False  # Layouts are unstable due to key differences
        # Step 2: Check if the values for each key are within the allowed range
        for key in self.latest_layout.keys():
            # Extracting coordinates for each block in both layouts
            x_latest, y_latest, z_latest = self.latest_layout[key]
            x_former, y_former, z_former = self.former_layout[key]

            # Calculating the differences
            diff_x = abs(x_latest - x_former)
            diff_y = abs(y_latest - y_former)
            diff_z = abs(z_latest - z_former)

            # Checking if any difference exceeds the allowed threshold
            if diff_x > self.layout_threshold or diff_y > self.layout_threshold or diff_z > self.layout_threshold:
                self.former_layout = self.latest_layout
                return False  # Layouts are unstable due to coordinate differences
        self.former_layout = self.latest_layout
        return True  # Layouts are stable
    
    # def learn_self_learn_v1(self):
    #     if self.first_stable_layout and self.last_stable_layout:
    #         self.get_logger().info(f'INFO --- learn6: self learning begin')
    #         exe_basic_arm_cmds = []
    #         # 对于first_stable_layout中的每个物体，检查是否需要移动
    #         for object_name, first_coords in self.first_stable_layout.items():
    #             if object_name in self.last_stable_layout:
    #                 last_coords = self.last_stable_layout[object_name]
    #                 # 计算坐标差异
    #                 diff = [abs(fc - lc) for fc, lc in zip(first_coords, last_coords)]

    #                 # 如果任一坐标差异超过阈值，生成grab和place命令
    #                 if any(d > self.layout_threshold for d in diff):
    #                     grab_cmd = ['grab', (object_name, first_coords[0], first_coords[1], first_coords[2])]
    #                     place_cmd = ['place', (object_name, last_coords[0], last_coords[1], last_coords[2])]
    #                     exe_basic_arm_cmds.append(grab_cmd)
    #                     exe_basic_arm_cmds.append(place_cmd)

    #         self.get_logger().info(f'INFO --- learn7: learned basic arm cmds = {exe_basic_arm_cmds}')
    #         pass
    #         self.learn_write_history(exe_basic_arm_cmds)
    #     else:
    #         self.get_logger().info(f'INFO --- learn6: lack stable layout, send layout_analysis req')
    #         self.learn_send_layout_analysis_request()

    # def learn_self_learn_v2(self):
    #     # 确保有足够的稳定布局历史记录来分析动作顺序
    #     if len(self.stable_layout_history) > 1:
    #         self.get_logger().info(
    #             f'INFO --- learn6: self learning begin with {len(self.stable_layout_history)} stable layouts')

    #         # 初始化一个空列表来存储推断出的动作指令
    #         inferred_actions = []

    #         # 遍历稳定布局历史记录中的每一对连续布局
    #         for i in range(len(self.stable_layout_history) - 1):
    #             current_layout = self.stable_layout_history[i]
    #             next_layout = self.stable_layout_history[i + 1]

    #             # 对于当前布局中的每个物体，检查它是否在下一个布局中位置有变化
    #             for object_name, current_coords in current_layout.items():
    #                 if object_name in next_layout:
    #                     next_coords = next_layout[object_name]

    #                     # 计算位置差异
    #                     diff = [abs(cc - nc) for cc, nc in zip(current_coords, next_coords)]

    #                     # 如果任一坐标差异超过阈值，生成相应的动作指令
    #                     if any(d > self.layout_threshold for d in diff):
    #                         grab_action = ['grab',
    #                                        (object_name, current_coords[0], current_coords[1], current_coords[2])]
    #                         palce_action = ['place', (object_name, next_coords[0], next_coords[1], next_coords[2])]
    #                         inferred_actions.append(grab_action)
    #                         inferred_actions.append(palce_action)

    #         self.get_logger().info(f'INFO --- learn7: inferred actions = {inferred_actions}')
    #         self.learn_write_history(inferred_actions)
    #     else:
    #         self.get_logger().info(
    #             'INFO --- learn6: Not enough stable layouts for learning, sending layout analysis request')
    #         self.learn_send_layout_analysis_request()


def main(args=None):
    rclpy.init(args=args)
    center_node = CenterNode()
    try:
        rclpy.spin(center_node)
    except KeyboardInterrupt:
        pass
    finally:
        center_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
