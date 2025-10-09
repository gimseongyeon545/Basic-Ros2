# [heartbeat_pub] ──(String /mini/heartbeat)──▶ [status_sub]
# [heartbeat_pub]

from rclpy.node import Node
import rclpy
from std_msgs.msg import String


class HeartBeatpub(Node):
  def __init__(self):
    super().__init__('heartbeat_pub')
    self.HBpub = self.create_publisher(String, '/mini/heartbeat', 10)
    self.timer = self.create_timer(0.5, self.cb)
    self.cnt = 0
    
  def cb(self):
    self.cnt += 1
    msg = String()
    msg.data = 'heartbeat msg'
    self.HBpub.publish(msg)
    self.get_logger().info(f'heartbeat pub cnt: {cnt}')  

rclpy.init()
rclpy.spin(HeartBeatpub)
rclpy.shutdown()
