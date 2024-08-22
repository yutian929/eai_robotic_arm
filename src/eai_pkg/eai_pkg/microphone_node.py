'''
功能：用户输入指令
用法：“将红色方块放在绿色方块的左上角”
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from eai_interfaces.srv import SpeakerInfo
import pyaudio
import wave
import random
import numpy as np
import sys
import json
import time
import pyaudio
import webrtcvad
import numpy as np
import wave

IS_PY3 = sys.version_info.major == 3

if IS_PY3:
    from urllib.request import urlopen
    from urllib.request import Request
    from urllib.error import URLError
    from urllib.parse import urlencode

    timer = time.perf_counter
else:
    import urllib2
    from urllib2 import urlopen
    from urllib2 import Request
    from urllib2 import URLError
    from urllib import urlencode

    if sys.platform == "win32":
        timer = time.clock
    else:
        # On most other platforms the best timer is time.time()
        timer = time.time

class DemoError(Exception):
    pass

class MicrophoneNode(Node):
    def __init__(self):
        super().__init__('microphone_node')
        self.asrpro_subscriptions = self.create_subscription(String, '/asrpro_serial', self.microphone_callback, 10)
        self.record = False
        self.publisher = self.create_publisher(String, '/microphone_input', 10)
        self.speaker_srv = self.create_client(SpeakerInfo, 'speaker_info')
        self.get_logger().info('INFO --- microphone_node setup finished.')

        self.API_KEY = ''
        self.SECRET_KEY = ''

        # 需要识别的文件
        self.AUDIO_FILE = 'record.wav'  # 只支持 pcm/wav/amr 格式，极速版额外支持m4a 格式
        # 文件格式
        self.FORMAT = self.AUDIO_FILE[-3:];  # 文件后缀只支持 pcm/wav/amr 格式，极速版额外支持m4a 格式

        self.CUID = '123456PYTHON';
        # 采样率
        self.RATE = 16000;  # 固定值

        # 普通版

        self.DEV_PID = 1537;  # 1537 表示识别普通话，使用输入法模型。根据文档填写PID，选择语言及识别模型
        self.ASR_URL = 'http://vop.baidu.com/server_api'
        self.SCOPE = 'audio_voice_assistant_get'  # 有此scope表示有asr能力，没有请在网页里勾选，非常旧的应用可能没有

        self.TOKEN_URL = 'http://aip.baidubce.com/oauth/2.0/token'

        self.token = self.fetch_token()

    def fetch_token(self):
        params = {'grant_type': 'client_credentials',
                'client_id': self.API_KEY,
                'client_secret': self.SECRET_KEY}
        post_data = urlencode(params)
        if (IS_PY3):
            post_data = post_data.encode('utf-8')
        req = Request(self.TOKEN_URL, post_data)
        try:
            f = urlopen(req)
            result_str = f.read()
        except URLError as err:
            # print('token http response http code : ' + str(err.code))
            result_str = err.read()
        if (IS_PY3):
            result_str = result_str.decode()

        # print(result_str)
        result = json.loads(result_str)
        # print(result)
        if ('access_token' in result.keys() and 'scope' in result.keys()):
            if self.SCOPE and (not self.SCOPE in result['scope'].split(' ')):  # SCOPE = False 忽略检查
                raise DemoError('scope is not correct')
            # print('SUCCESS WITH TOKEN: %s ; EXPIRES IN SECONDS: %s' % (result['access_token'], result['expires_in']))
            return result['access_token']
        else:
            raise DemoError('MAYBE API_KEY or SECRET_KEY not correct: access_token or scope not found in token response')


    def microphone_callback(self, msg):
        # self.get_logger().info(f"INFO --- microphone_node received = {msg.data}")
        if "awake" in msg.data:
            self.record = True
            request = SpeakerInfo.Request()
            request.speaker_info_req = random.choice(["我在！", "主人我在！", "我在这里！"])  # 随机选择一句话
            self.get_logger().info(f"INFO --- microphone_node request = {request.speaker_info_req}")
            future = self.speaker_srv.call_async(request)
            future.add_done_callback(self.callback_done)

    def callback_done(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info(f"ERROR --- microphone_node exception: {e}")
        else:
            self.get_logger().info(f"INFO --- speaker_node response: {response.speaker_info_res}")
            self.record_audio()

    def record_audio(self):
        CHUNK = 1024
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        # THRESHOLD = 3000
        # RECORD_SECONDS = 5
        NOISE_SAMPLES = 20  # 用于测量噪音的样本数量
        MARGIN = 1.5  # 动态阈值与环境噪音能量的比例边际
        WAVE_OUTPUT_FILENAME = "record.wav"
        p = pyaudio.PyAudio()

        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)

        self.get_logger().info("Measuring noise...")
        noise_frames = [stream.read(CHUNK) for _ in range(NOISE_SAMPLES)]
        noise_energy = np.mean([np.linalg.norm(np.frombuffer(frame, dtype=np.int16)) / CHUNK for frame in noise_frames])
        THRESHOLD = noise_energy * MARGIN
        self.get_logger().info(f"Dynamic threshold set to: {THRESHOLD}")

        self.get_logger().info("* recording")

        frames = []
        silent_count = 0  # 用于计数连续的静音帧数
        start_recording = False
        while True:
            data = stream.read(CHUNK)

            # 将数据转换为numpy数组，计算音频信号的能量
            # audio_data = np.frombuffer(data, dtype=np.int16)
            # energy = np.sum(audio_data ** 2) / len(audio_data)
            # 使用直接的方法计算能量
            energy = np.linalg.norm(np.frombuffer(data, dtype=np.int16)) / CHUNK
            # self.get_logger().info(f"INFO --- energy = {energy}")
            # 如果能量低于阈值，则增加静音计数
            if energy < THRESHOLD and start_recording:
                silent_count += 1
                frames.append(data)
            elif energy < THRESHOLD and not start_recording:
                continue
            elif energy > THRESHOLD and not start_recording:
                start_recording = True
                self.get_logger().info(f"start_energy = {energy}")
                frames.append(data)
            elif energy > THRESHOLD and start_recording:
                frames.append(data)
                silent_count = max(0, silent_count-1)  # 静音计数

            # 如果连续的静音帧数超过一定阈值，就停止录音
            if silent_count > 30:  # 10帧的静音
                break

        self.get_logger().info("* done recording")

        stream.stop_stream()
        stream.close()
        p.terminate()

        wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
        self.record = False
        self.speech_recognition(WAVE_OUTPUT_FILENAME)
    
    def speech_recognition(self, AUDIO_FILE):
        speech_data = []
        with open(AUDIO_FILE, 'rb') as speech_file:
            speech_data = speech_file.read()
        length = len(speech_data)
        if length == 0:
            raise DemoError('file %s length read 0 bytes' % AUDIO_FILE)

        params = {'cuid': self.CUID, 'token': self.token, 'dev_pid': self.DEV_PID}
        #测试自训练平台需要打开以下信息
        #params = {'cuid': CUID, 'token': token, 'dev_pid': DEV_PID, 'lm_id' : LM_ID}
        params_query = urlencode(params);

        headers = {
            'Content-Type': 'audio/' + self.FORMAT + '; rate=' + str(self.RATE),
            'Content-Length': length
        }

        url = self.ASR_URL + "?" + params_query
        # print("url is", url);
        # print("header is", headers)
        # print post_data
        req = Request(url, speech_data, headers)
        try:
            begin = timer()
            f = urlopen(req)
            result_str = f.read()
            # print("Request time cost %f" % (timer() - begin))
        except  URLError as err:
            # print('asr http response http code : ' + str(err.code))
            result_str = err.read()

        if (IS_PY3):
            result_str = str(result_str, 'utf-8')
        print(result_str)
        # with open("result.txt", "w") as of:
        #     of.write(result_str)
        if result_str:
            result = json.loads(result_str)
            if result['err_no'] == 0:
                result = result['result'][0]
                self.get_logger().info(f"INFO --- microphone_node recognized = {result}")
                self.send_command(result)
            else:
                self.get_logger().info(f"ERROR --- microphone_node error: {result['err_msg']}")
        # result = json.loads(result_str)
        # result = result['result'][0]

        # self.get_logger().info(f"INFO --- microphone_node recognized = {result}")
        # self.send_command(result)

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        # self.get_logger().info(f"INFO --- microphone_node published = {command}")

def main(args=None):
    rclpy.init(args=args)
    user_cmd_node = MicrophoneNode()
    rclpy.spin(user_cmd_node)
    user_cmd_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()