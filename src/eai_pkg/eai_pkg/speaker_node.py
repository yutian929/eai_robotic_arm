# coding=utf-8
#!/usr/bin/env python3

import os
import sys
import json
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from eai_interfaces.srv import SpeakerInfo

import pyaudio
import wave

IS_PY3 = sys.version_info.major == 3
if IS_PY3:
    from urllib.request import urlopen
    from urllib.request import Request
    from urllib.error import URLError
    from urllib.parse import urlencode
    from urllib.parse import quote_plus
else:
    import urllib2
    from urllib import quote_plus
    from urllib2 import urlopen
    from urllib2 import Request
    from urllib2 import URLError
    from urllib import urlencode

class DemoError(Exception):
    pass

class SpeakerNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.service = self.create_service(SpeakerInfo, 'speaker_info', self.speaker_info_callback)
        
        self.API_KEY = ''
        self.SECRET_KEY = ''
        # 发音人选择, 基础音库：0为度小美，1为度小宇，3为度逍遥，4为度丫丫，
        # 精品音库：5为度小娇，103为度米朵，106为度博文，110为度小童，111为度小萌，默认为度小美 
        self.PER = 3
        # 语速，取值0-15，默认为5中语速
        self.SPD = 5
        # 音调，取值0-15，默认为5中语调
        self.PIT = 5
        # 音量，取值0-9，默认为5中音量
        self.VOL = 5
        # 下载的文件格式, 3：mp3(default) 4： pcm-16k 5： pcm-8k 6. wav
        self.AUE = 6

        FORMATS = {3: "mp3", 4: "pcm", 5: "pcm", 6: "wav"}
        self.FORMAT = FORMATS[self.AUE]

        self.CUID = "123456PYTHON"

        self.TTS_URL = 'http://tsn.baidu.com/text2audio'

        self.TOKEN_URL = 'http://aip.baidubce.com/oauth/2.0/token'
        self.SCOPE = 'audio_tts_post'  # 有此scope表示有tts能力，没有请在网页里勾选

        self.token = self.fetch_token()

        self.video_file_path = 'play.wav'
        self.get_logger().info(f'INFO --- speaker_node setup finished.')

    def fetch_token(self):
        # print("fetch token begin")
        params = {'grant_type': 'client_credentials',
                'client_id': self.API_KEY,
                'client_secret': self.SECRET_KEY}
        post_data = urlencode(params)
        if (IS_PY3):
            post_data = post_data.encode('utf-8')
        req = Request(self.TOKEN_URL, post_data)
        try:
            f = urlopen(req, timeout=5)
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
            if not self.SCOPE in result['scope'].split(' '):
                raise DemoError('scope is not correct')
            # print('SUCCESS WITH TOKEN: %s ; EXPIRES IN SECONDS: %s' % (result['access_token'], result['expires_in']))
            return result['access_token']
        else:
            raise DemoError('MAYBE API_KEY or SECRET_KEY not correct: access_token or scope not found in token response')

    def speaker_info_callback(self, request, response):
        text = request.speaker_info_req
        self.get_logger().info(f'INFO --- speaker_info_callback: {text}')
        # if text == "我在！":
        #     self.play_audio('/home/eai/ros2_ws/eai4_ws/videos/wozai.wav')
        # else:
        self.get_logger().info(f'INFO --- generating new wav audio.')
        self.generate_audio(text)
        self.play_audio(self.video_file_path)
        response.speaker_info_res = True
        return response
    


    def generate_audio(self, TEXT):
        # Your TTS logic goes here, similar to the play_wav function in your original script
        tex = quote_plus(TEXT)  # 此处TEXT需要两次urlencode
        # print(tex)
        params = {'tok': self.token, 'tex': tex, 'per': self.PER, 'spd': self.SPD, 'pit': self.PIT, 'vol': self.VOL, 'aue': self.AUE, 'cuid': self.CUID,
                'lan': 'zh', 'ctp': 1}  # lan ctp 固定参数

        data = urlencode(params)
        # print('test on Web Browser' + self.TTS_URL + '?' + data)

        req = Request(self.TTS_URL, data.encode('utf-8'))
        has_error = False
        try:
            f = urlopen(req)
            result_str = f.read()

            headers = dict((name.lower(), value) for name, value in f.headers.items())

            has_error = ('content-type' not in headers.keys() or headers['content-type'].find('audio/') < 0)
        except  URLError as err:
            print('asr http response http code : ' + str(err.code))
            result_str = err.read()
            has_error = True

        save_file = "error.txt" if has_error else self.video_file_path
        with open(save_file, 'wb') as of:
            of.write(result_str)

        if has_error:
            if (IS_PY3):
                result_str = str(result_str, 'utf-8')
            # print("tts api  error:" + result_str)

        # print("result saved as :" + save_file)
    def play_audio(self, video_file_path):
        chunk_size = 1024
        wf = wave.open(video_file_path, 'rb')
        p = pyaudio.PyAudio()
        stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                        channels=wf.getnchannels(),
                        rate=wf.getframerate(),
                        output=True)
        data = wf.readframes(chunk_size)
        while len(data) > 0:
            stream.write(data)
            data = wf.readframes(chunk_size)
        stream.stop_stream()
        stream.close()
        p.terminate()

def main(args=None):
    rclpy.init(args=args)
    speaker_node = SpeakerNode('speaker_node')
    rclpy.spin(speaker_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

