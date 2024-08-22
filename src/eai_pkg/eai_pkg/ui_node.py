import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import websockets

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')
        self.subscription = self.create_subscription(String,'/ros_to_frontend_topic',self.listener_callback,10)

    def listener_callback(self, msg):
        self.get_logger_info(f"INFO --- received message: {msg.data}")
        # 当接收到ROS消息时，将消息发送到前端页面
        asyncio.get_event_loop().run_until_complete(self.send_to_frontend(msg.data))

    async def send_to_frontend(self, data):
        async with websockets.connect('ws://localhost:8765') as websocket:
            await websocket.send(data)


def main(args=None):
    rclpy.init(args=args)

    ui_node = UINode()

    rclpy.spin(ui_node)

    ui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
