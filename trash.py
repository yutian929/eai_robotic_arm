

# import pyaudio
# import webrtcvad
# import numpy as np
# import wave
#
# def capture_sound_period(path_audio_data):
#         speech_started = False
#         frames = []
#         vad = webrtcvad.Vad()
#         vad.set_mode(1)  # 1/2/3
#         p = pyaudio.PyAudio()
#         stream = p.open(format=pyaudio.paInt16,
#                              channels=1,
#                              rate=16000,
#                              input=True,
#                              frames_per_buffer=960)
#
#         while True:
#             audio_data = stream.read(960)  # 读取音频数据，帧大小通常为960（20毫秒）
#             # 在读取音频数据后进行格式转换
#             audio_data_np = np.frombuffer(audio_data, dtype=np.int16)
#             # 使用VAD检测声音活动
#             if vad.is_speech(audio_data_np, sample_rate=16000):  # 有声音活动
#                 if not speech_started:  # 首次开始
#                     print("开始捕获声音活动...")
#                     speech_started = True
#                     # print("声音活动开始")
#                     frames.append(audio_data)
#                     for i in range(0, int(16000 / 960)):  # 至少录制1s
#                         audio_data = stream.read(960)
#                         frames.append(audio_data)
#                 else:  # 之前就已经开始了
#                     # print("声音活动继续")
#                     frames.append(audio_data)
#                     for i in range(0, int(16000 / 960 * 0.25)):  # 至少录制0.25s
#                         audio_data = stream.read(960)
#                         frames.append(audio_data)
#             else:  # 没有声音活动
#                 if speech_started:  # 之前已经开始了
#                     print("声音活动结束...")
#                     wf = wave.open(path_audio_data, 'wb')
#                     wf.setnchannels(1)
#                     wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
#                     wf.setframerate(16000)
#                     wf.writeframes(b''.join(frames))
#                     wf.close()
#                     break
#         stream.stop_stream()
#         stream.close()
#         p.terminate()
#
# capture_sound_period('test_record.wav')

# class SoundRecognizer():
#     def __init__(self, model_name="tiny.en", sense_mode=3):
#         self.model = whisper.load_model(model_name)
#         self.vad = webrtcvad.Vad()
#         self.vad.set_mode(sense_mode)
#         self.p = pyaudio.PyAudio()
#         self.stream = None

#     def turn_on(self):
#         self.stream = self.p.open(format=pyaudio.paInt16,
#                              channels=1,
#                              rate=16000,
#                              input=True,
#                              frames_per_buffer=960)
#         # print("开始捕获声音活动...")

#     def turn_off(self):
#         self.stream.stop_stream()
#         self.stream.close()
#         self.p.terminate()

#     def recognize_once(self, path_audio_data: str):
#         result = self.model.transcribe(path_audio_data, fp16=False)
#         return result['text']

#     def capture_sound_period(self, path_audio_data):
#         speech_started = False
#         frames = []
#         while True:
#             audio_data = self.stream.read(960)  # 读取音频数据，帧大小通常为960（20毫秒）
#             # 在读取音频数据后进行格式转换
#             audio_data_np = np.frombuffer(audio_data, dtype=np.int16)
#             # 使用VAD检测声音活动
#             if self.vad.is_speech(audio_data_np, sample_rate=16000):  # 有声音活动
#                 if not speech_started:  # 首次开始
#                     speech_started = True
#                     # print("声音活动开始")
#                     frames.append(audio_data)
#                     for i in range(0, int(16000 / 960)):  # 至少录制1s
#                         audio_data = self.stream.read(960)
#                         frames.append(audio_data)
#                 else:  # 之前就已经开始了
#                     # print("声音活动继续")
#                     frames.append(audio_data)
#                     for i in range(0, int(16000 / 960 * 0.25)):  # 至少录制0.25s
#                         audio_data = self.stream.read(960)
#                         frames.append(audio_data)
#             else:  # 没有声音活动
#                 if speech_started:  # 之前已经开始了
#                     # print("声音活动结束")
#                     wf = wave.open(path_audio_data, 'wb')
#                     wf.setnchannels(1)
#                     wf.setsampwidth(self.p.get_sample_size(pyaudio.paInt16))
#                     wf.setframerate(16000)
#                     wf.writeframes(b''.join(frames))
#                     wf.close()
#                     break

    # def exe_start(self):  # exe0.declare basic info
    #     self.get_logger().info(f'INFO --- exe0. exe_start')
    #     self.exe_get_color_img()

    # def exe_get_color_img(self):
    #     while not self.color_img_client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().warn('WARN --- color_img service not available, waiting again...')
    #     req = ColorImg.Request()
    #     req.color_img_req = True
    #     self.get_logger().info(f'INFO --- exe1. ask for color_img')
    #     future = self.color_img_client.call_async(req)
    #     future.add_done_callback(self.exe_color_img_received_callback)

    # def exe_color_img_received_callback(self, future):
    #     try:
    #         response = future.result()
    #         # 使用CvBridge将ROS图像消息转换为OpenCV图像
    #         cv_image = self.bridge.imgmsg_to_cv2(response.color_img_res, "bgr8")
    #         # # 显示图像
    #         # cv2.imshow("exe_color_img", cv_image)
    #         # cv2.waitKey(30)  # 等待至少1毫秒，以便图像显示
    #         self.latest_color_img = cv_image
    #         self.exe_send_layout_analysis_request()
            
    #     except Exception as e:
    #         self.get_logger().error(f'Failed to convert image: {e}')
        
    # def exe_send_layout_analysis_request(self):  # exe1.send layout req
    #     while not self.layout_analysis_client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().warn('WARN --- layout_analysis service not available, waiting again...')
    #     req = LayoutAnalysis.Request()
    #     req.ask_for_layout = self.layout_type
    #     self.get_logger().info(f'INFO --- exe2. ask Layout analysis ask for special type: {self.layout_type}')
    #     time.sleep(0.3)  # wait for stable layout
    #     future = self.layout_analysis_client.call_async(req)
    #     future.add_done_callback(self.exe_layout_analysis_received_callback)

    # def exe_layout_analysis_received_callback(self, future):  # exe2.received layout
    #     try:
    #         response = future.result()
    #         layout = response.layout
    #         layout = json.loads(layout)  # layout now is a dict = {'name', (ARM_x, ARM_y, ARM_z)...}
    #         self.latest_layout = layout  # 3D
    #         self.get_logger().info(f'INFO --- exe3: received Layout analysis: {layout}')
    #         if layout:
    #             if self.exe_is_layout_stable():
    #                 # layout_json = json.dumps(self.latest_layout)
    #                 # self.exe_send_vlm_request(layout_json)
    #                 self.exe_send_req_to_get_c_nxywhd()  # 除了拿arm_xyz下的，还要拿c_nxywhd去裁剪图片
    #             else:
    #                 self.get_logger().warn("WARN --- layout isn't stable.")
    #                 self.exe_get_color_img()
    #         else:
    #             self.get_logger().info('INFO --- No layout found.')
    #             self.exe_get_color_img()

    #     except Exception as e:
    #         self.get_logger().error('ERROR --- Service call failed, maybe unstable %r' % (e,))
    
    # def exe_send_req_to_get_c_nxywhd(self):
    #     while not self.layout_analysis_client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().warn('WARN --- layout_analysis service not available, waiting again...')
    #     req = LayoutAnalysis.Request()
    #     req.ask_for_layout = 'c_nxywhd'
    #     self.get_logger().info(f'INFO --- exe2. ask Layout analysis ask for special type: {self.layout_type}')
    #     time.sleep(0.3)  # wait for stable layout
    #     future = self.layout_analysis_client.call_async(req)
    #     future.add_done_callback(self.exe_c_nxywhd_received_callback)
    
    # def exe_c_nxywhd_received_callback(self, future):
    #     try:
    #         response = future.result()
    #         layout = response.layout
    #         layout = json.loads(layout)  # layout now is a dict = {'name', (c_x, c_y, c_w, c_h, c_d)...}
    #         self.latest_c_nxywhd = layout  # 3D
    #         self.get_logger().info(f'INFO --- exe3: received Layout analysis: {layout}')  # layout_c_nxywhd{'name':(x,y,w,h,mean_depth)}
    #         if layout:
    #             if self.exe_is_c_nxywhd_stable():
    #                 self.exe_send_vlm_request()  # 此刻万事俱备
    #             else:
    #                 self.get_logger().warn("WARN --- c_nxywhd isn't stable.")
    #                 self.exe_send_req_to_get_c_nxywhd()
    #         else:
    #             self.get_logger().info('INFO --- No layout found.')
    #             self.exe_send_req_to_get_c_nxywhd()

    #     except Exception as e:
    #         self.get_logger().error('ERROR --- exe_c_nxywhd_received_callback, Service call failed, maybe unstable %r' % (e,))
    
    # def exe_is_c_nxywhd_stable(self):
    #     if not self.former_c_nxywhd:
    #         self.former_c_nxywhd = self.latest_c_nxywhd
    #         return False
    #     # Step 1: Check if the keys of both layouts are identical
    #     if set(self.latest_c_nxywhd.keys()) != set(self.former_c_nxywhd.keys()):
    #         self.former_c_nxywhd = self.latest_c_nxywhd
    #         return False  # Layouts are unstable due to key differences
    #     # Step 2: Check if the values for each key are within the allowed range
    #     for key in self.latest_c_nxywhd.keys():
    #         # Extracting coordinates for each block in both layouts
    #         x_latest, y_latest, w_latest, h_latest, d_latest = self.latest_c_nxywhd[key]
    #         x_former, y_former, w_former,  h_former, d_former = self.former_c_nxywhd[key]

    #         # Calculating the differences
    #         diff_x = abs(x_latest - x_former)
    #         diff_y = abs(y_latest - y_former)
    #         diff_w = abs(w_latest - w_former)
    #         diff_h = abs(h_latest - h_former)
    #         diff_d = abs(d_latest - d_former)

    #         # Checking if any difference exceeds the allowed threshold
    #         if diff_x > self.layout_threshold or diff_y > self.layout_threshold or diff_w > self.layout_threshold or diff_h > self.layout_threshold or diff_d > self.layout_threshold:
    #             self.former_c_nxywhd = self.latest_c_nxywhd
    #             return False  # Layouts are unstable due to coordinate differences
    #     self.former_c_nxywhd = self.latest_c_nxywhd
    #     return True  # Layouts are stable

    # def exe_is_layout_stable(self):
    #     if not self.former_layout:
    #         self.former_layout = self.latest_layout
    #         return False
    #     # Step 1: Check if the keys of both layouts are identical
    #     if set(self.latest_layout.keys()) != set(self.former_layout.keys()):
    #         self.former_layout = self.latest_layout
    #         return False  # Layouts are unstable due to key differences
    #     # Step 2: Check if the values for each key are within the allowed range
    #     for key in self.latest_layout.keys():
    #         # Extracting coordinates for each block in both layouts
    #         x_latest, y_latest, z_latest = self.latest_layout[key]
    #         x_former, y_former, z_former = self.former_layout[key]

    #         # Calculating the differences
    #         diff_x = abs(x_latest - x_former)
    #         diff_y = abs(y_latest - y_former)
    #         diff_z = abs(z_latest - z_former)

    #         # Checking if any difference exceeds the allowed threshold
    #         if diff_x > self.layout_threshold or diff_y > self.layout_threshold or diff_z > self.layout_threshold:
    #             self.former_layout = self.latest_layout
    #             return False  # Layouts are unstable due to coordinate differences
    #     self.former_layout = self.latest_layout
    #     return True  # Layouts are stable

    # def exe_send_vlm_request(self):
    #     req = VlmExe.Request()
    #     try:
    #         # 根据layout确定真正的要发送的区域
    #         layout_c_nxywhd = self.latest_c_nxywhd
    #         color_img = self.latest_color_img
    #         x_min, y_min = float('inf'), float('inf')
    #         x_max, y_max = float('-inf'), float('-inf')

    #         for name, (x, y, w, h, d) in layout_c_nxywhd.items():
    #             x_min = min(x_min, x)
    #             y_min = min(y_min, y)
    #             x_max = max(x_max, x + w)
    #             y_max = max(y_max, y + h)

    #         # 最小矩形的左上角坐标(x_min, y_min)和宽度、高度(width, height)
    #         width = min(x_max - x_min + 20, color_img.shape[1])
    #         height = min(y_max - y_min + 20, color_img.shape[0])

    #         exe_color_img = color_img[y_min:y_min + height, x_min:x_min + width]

    #         req.color_img = self.bridge.cv2_to_imgmsg(exe_color_img, "bgr8")
    #         req.layout = json.dumps(self.latest_layout)
    #         req.user_cmd = self.user_cmd
    #         self.get_logger().info(f'INFO --- exe4: send vlm latest_color_img and layout_json and user_cmd')
    #         # 显示图像
    #         cv2.imshow("exe_color_img", exe_color_img)
    #         cv2.waitKey(30)  # 等待至少1毫秒，以便图像显示
    #     except Exception as e:
    #         self.get_logger().error(f'Failed to form VlmExe.req: {e}')
    #     future = self.vlm_exe_client.call_async(req)
    #     future.add_done_callback(self.exe_vlm_exe_received_callback)

    # # def exe_vlm_exe_received_callback(self, future):
    # #     try:
    # #         response = future.result()
    # #         self.get_logger().info(f'INFO --- exe5: received VlmExe.res: {response.vlm_advice}')
    # #         if response.vlm_advice:
    # #             self.vlm_exe_advice = response.vlm_advice
    # #             self.exe_send_ai_cmd_request()
    # #         else:
    # #             self.get_logger().info('INFO --- No vlm_exe_advice.')
    # #             self.exe_send_vlm_request()
    # #     except Exception as e:
    # #         self.get_logger().error('ERROR --- Service call failed, maybe unstable %r' % (e,))

    # # def exe_send_ai_cmd_request(self):  # exe3. send ai_cmd req
    # #     layout = json.dumps(self.latest_layout)
    # #     vlm_advice = self.vlm_exe_advice
    # #     ai_prompt = '\n' + 'input: layout=' + layout + ', user_cmd=' + self.user_cmd + ', vlm_advice=' + vlm_advice + '。'

    # #     self.get_logger().info(f'INFO --- exe3: form and send ai_prompt.')

    # #     while not self.ai_cmd_client.wait_for_service(timeout_sec=1.0):
    # #         self.get_logger().warn('WARN --- ai_cmd service not available, waiting again...')

    # #     req = AiCmd.Request()
    # #     req.ai_prompt = ai_prompt
    # #     future = self.ai_cmd_client.call_async(req)
    # #     future.add_done_callback(self.exe_ai_basic_arm_cmds_received_callback)
        

    # # def exe_ai_basic_arm_cmds_received_callback(self, future):  # exe4. received ai_basic_arm_cmds(json-str)
    # #     try:
    # #         response = future.result()
    # #         # self.get_logger().info(f'INFO ---*STEP5: json_ai_basic_cmds = {response.ai_basic_arm_cmds}, {type(response.ai_basic_arm_cmds.lower())}')
    # #         if 'ok' in response.ai_basic_arm_cmds.lower():
    # #             self.get_logger().info(f'INFO --- exe4. AI response = ok, exe basic flow finished.')
    # #             self.basic_flow_finished()

    # #         elif 'wrong' in response.ai_basic_arm_cmds.lower():
    # #             self.get_logger().info(f'INFO --- exe4. AI response = wrong, exe basic flow finished.')
    # #             self.basic_flow_finished()

    # #         else:
    # #             basic_arm_cmds = json.loads(response.ai_basic_arm_cmds)  # basic_arm_cmds is a list[list]
    # #             self.basic_arm_cmds = basic_arm_cmds  # for arm_node to execute basic_arm_cmds one by one
    # #             self.get_logger().info(f'INFO --- exe4. Received ai basic arm cmds  = {basic_arm_cmds}, go on')
    # #             self.exe_basic_arm_cmds_optimize(basic_arm_cmds)
    # #     except Exception as e:
    # #         self.get_logger().error('ERROR --- Service call failed %r' % (e,))
        

    # def exe_vlm_exe_received_callback(self, future):  # 现在只要和vlm进行通讯就好了，不用和原本的ai_cmd服务进行交接
    #     try:
    #         response = future.result()
    #         exe_ai_return = response.vlm_advice
    #         self.get_logger().info(f'INFO --- exe5: received VlmExe.vlm_advice = {exe_ai_return}')

    #         if 'ok' in exe_ai_return.lower():
    #             self.get_logger().info(f'INFO --- exe5. AI response = ok, exe basic flow finished.')
    #             self.basic_flow_finished()

    #         elif 'wrong' in exe_ai_return.lower():
    #             self.get_logger().info(f'INFO --- exe5. AI response = wrong, exe basic flow finished.')
    #             self.basic_flow_finished()

    #         else:
    #             basic_arm_cmds = json.loads(exe_ai_return)  # basic_arm_cmds is a list[list]
    #             self.basic_arm_cmds = basic_arm_cmds  # for arm_node to execute basic_arm_cmds one by one
    #             self.get_logger().info(f'INFO --- exe5. AI generate basic arm cmds  = {basic_arm_cmds}, go on')
    #             self.exe_basic_arm_cmds_optimize(basic_arm_cmds)
    #     except Exception as e:
    #         self.get_logger().error('ERROR --- exe_vlm_exe_received_callback,  Service call failed %r' % (e,))

    
    # def exe_basic_arm_cmds_optimize(self, basic_arm_cmds):  # 后续结合Depth图像进行优化前后左右要保证至少差20mm
    #     pass
    #     self.exe_execute_arm_commands(basic_arm_cmds)

    # def exe_execute_arm_commands(self, basic_arm_cmds):  # exe5.exe_execute_arm_commands one by one, send basic_arm_cmd
    #     if basic_arm_cmds:  # (str-json)
    #         self.basic_arm_cmd_id += 1
    #         basic_arm_cmd = self.basic_arm_cmds[0]  # basic_arm_cmd is a list = ['grab', 'red_block', cx, cy]
    #         self.get_logger().info(
    #             f'INFO --- exe5: Execute basic_arm_cmd: {basic_arm_cmd} ID: {self.basic_arm_cmd_id}')
    #         try:
    #             action_type, obj_name, arm_x, arm_y, arm_z = basic_arm_cmd
    #         except:
    #             action_type, (obj_name, arm_x, arm_y, arm_z) = basic_arm_cmd  # for arm_node to rotate arm
    #         basic_arm_cmd_3D = json.dumps((action_type, obj_name, arm_x, arm_y, arm_z))
    #         while not self.basic_arm_cmd_client.wait_for_service(timeout_sec=1.0):
    #             self.get_logger().warn('WARN --- basic_arm_cmd service not available, waiting again...')
    #         req = BasicArmCmd.Request()
    #         req.basic_arm_cmd = basic_arm_cmd_3D
    #         future = self.basic_arm_cmd_client.call_async(req)
    #         future.add_done_callback(self.exe_basic_arm_cmd_received_callback)
    #     else:
    #         self.basic_arm_cmd_id = 0
    #         self.exe_start()  # exe7. recurrent

    # def exe_basic_arm_cmd_received_callback(self, future):  # exe6. jeston exe
    #     try:
    #         response = future.result()
    #         if response.arm_finished:
    #             self.get_logger().info(f'INFO --- exe6: Arm finished basic_arm_cmd ID: {self.basic_arm_cmd_id}')
    #             self.basic_arm_cmds = self.basic_arm_cmds[1:]
    #             self.exe_execute_arm_commands(self.basic_arm_cmds)
    #         else:
    #             self.get_logger().info('INFO --- Arm not finished')
    #     except Exception as e:
    #         self.get_logger().error('ERROR --- Service call failed %r' % (e,))