"""
[heartbeat_pub] --( /mini/heartbeat : std_msgs/String )--> [status_sub]
        │                                                   ▲
        └─ uses Timer(hz) & Parameters(text,hz) ────────────┘
        
[ros2 param]* ──(set/get)──> [heartbeat_pub]   *CLI로 hz,text 동적 변경
"""

from rclpy.node import Node
import rclpy
from std_msgs.msg import String


class HeartBeatpub(Node):
  def __init__(self):
    super().__init__('heartbeat_pub')
    self.HBpub = self.create_publisher(String, '/mini/heartbeat', 10)
    self.timer = self.create_timer(0.5, self.cb)
    self.cnt = 0
    self.declare_paramter('text', 'hi')
    self.declare_paramter('hz', 2.0)
    self.hz = self.get_paramter('hz')
        
  def cb(self):
    self.cnt += 1
    msg = String()
    txt = self.get_paramter('text')
    msg.data = txt
    self.HBpub.publish(msg)
    self.get_logger().info(f'heartbeat pub cnt: {self.cnt}')  

rclpy.init()
rclpy.spin(HeartBeatpub())
rclpy.shutdown()
