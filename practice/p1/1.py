# [heartbeat_pub] ──(String /mini/heartbeat)──▶ [status_sub]
# [heartbeat_pub]

import rclpy.node as Node
import rclpy

class HeartBeatpub(Node):
  def __init__(self):
    super().__init__('heartbeat_pub')
    self.HBpub = self.create_publisher(String, '/mini/heartbeat', 10)
    self.timer = self.create_timer(0.5, self.cb)
    self.cnt = 0
    
  def cb(self, msg):
    self.cnt += 1
    self.HBpub.publish(msg)
    self.get_logger().info(f'heartbeat pub cnt: {cnt}')

rclpy.init()
rclpy.spin(HeartBeatpub)
rclpy.shutdown()
