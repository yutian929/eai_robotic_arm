import pyaudio
import wave
import numpy as np
import time
import  random
class MicrophoneNode:
    def __init__(self):
        self.record = False

    def microphone_callback(self, msg):
        print(f"INFO --- microphone_node received = {msg}")
        if "awake" in msg:
            self.record = True
            print("INFO --- microphone_node response: " + random.choice(["我在！", "主人我在噢!", "嗯呢！"]))  # 随机选择一句话
            self.record_audio()

    def record_audio(self):
        CHUNK = 1024
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        THRESHOLD = 2000  # 设置一个阈值，用于判断是否停止录音
        WAVE_OUTPUT_FILENAME = "temp.wav"
        p = pyaudio.PyAudio()

        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)

        print("* recording")

        frames = []
        silent_count = 0  # 用于计数连续的静音帧数

        while True:
            data = stream.read(CHUNK)
            frames.append(data)

            # 将数据转换为numpy数组，计算音频信号的能量
            audio_data = np.frombuffer(data, dtype=np.int16)
            energy = np.sum(audio_data ** 2) / len(audio_data)

            # 如果能量低于阈值，则增加静音计数
            if energy < THRESHOLD:
                silent_count += 1
            else:
                silent_count = 0  # 重置静音计数

            # 如果连续的静音帧数超过一定阈值，就停止录音
            if silent_count > 10:  # 10帧的静音
                break

        print("* done recording")

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

    def speech_recognition(self, audio_file):
        # 实现语音识别的逻辑
        pass

    def send_command(self, command):
        print(f"INFO --- microphone_node published = {command}")

def main():
    user_cmd_node = MicrophoneNode()
    while True:
        # 模拟从话题接收消息
        user_input = input("请输入指令: ")
        user_cmd_node.microphone_callback(user_input)
        time.sleep(1)  # 每隔1秒检查一次

if __name__ == '__main__':
    main()
