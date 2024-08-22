from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord
from pymycobot import PI_PORT, PI_BAUD  # 当使用树莓派版本的mycobot时，可以引用这两个变量进行MyCobot初始化
import time

# MyCobot 类初始化需要两个参数：
#   第一个是串口字符串， 如：
#       linux： "/dev/ttyUSB0"
#       windows: "COM3"
#   第二个是波特率：
#       M5版本为： 115200
#   如:
#       mycobot-M5:
#           linux:
#              mc = MyCobot("/dev/ttyUSB0", 115200)
#           windows:
#              mc = MyCobot("COM3", 115200)
#       mycobot-raspi:
#           mc = MyCobot(PI_PORT, PI_BAUD)
#
# 初始化一个MyCobot对象
# 下面为 windows版本创建对象代码
mc = MyCobot("/dev/ttyACM2", 115200)

mc.send_angles([0, 0, 0, 0, 0, 0], 50)

# 设置等待时间，确保机械臂已经到达指定位置
time.sleep(2.5)
# 获取当前头部的坐标以及姿态
coords = mc.get_coords()
print(coords)

mc.send_coords( coords, 80, 1)

# 设置等待时间1.5秒
time.sleep(1.5)
