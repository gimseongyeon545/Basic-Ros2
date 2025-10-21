'''
[heartbeat_pub] --( /mini/heartbeat : std_msgs/String )--> [status_sub]
        │                                                   ▲
        └─ uses Timer(hz) & Parameters(text,hz) ────────────┘
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StatusSub(Node):
  def __init__(self):
    super().__init__('status_sub')
    self.StatusSub = self.create_subscription(String, '/mini/heartbeat', self.cb, 10)
    self.timer = self.create_timer(0.5, self.cb)
    self.cnt = 0

  def cb(self, msg):
    self.cnt += 1
    self.get_logger().info(f'heartbeat sub cnt: {self.cnt} || data = {msg.data}')

rclpy.init()
rclpy.spin(StatusSub())
rclpy.shutdown()
