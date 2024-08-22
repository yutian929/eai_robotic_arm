import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class AsrproNode(Node):
    def __init__(self):
        super().__init__('asrpro_node')
        self.publisher_ = self.create_publisher(String, 'asrpro_serial', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)  # 串口号和波特率根据实际情况进行更改
        self.get_logger().info('INFO --- asrpro_node setup finished.')

    def serial_read(self):
        if self.serial_port.in_waiting > 0:
            received_data = self.serial_port.readline().decode().strip()
            self.get_logger().info(f'INFO --- asrpro_node received: {received_data}')
            if received_data == 'awake':
                msg = String()
                msg.data = 'I am awake!'
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    asrpro_node = AsrproNode()
    try:
        while rclpy.ok():
            asrpro_node.serial_read()
            rclpy.spin_once(asrpro_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    asrpro_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
