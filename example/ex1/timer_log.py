# 01_timer_log.py
import rclpy
from rclpy.node import Node

class Heartbeat(Node):
    def __init__(self):
        super().__init__('heartbeat')
        self.count = 0
        self.timer = self.create_timer(0.5, self.tick)  # 2Hz

    def tick(self):
        self.count += 1
        self.get_logger().info(f'beat {self.count}')

rclpy.init(); rclpy.spin(Heartbeat()); rclpy.shutdown()
