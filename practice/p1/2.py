# [heartbeat_pub] ──(String /mini/heartbeat)──▶ [status_sub]
# [status_sub]

import rclpy
import rclpy.node as Node

class StatusSub(Node):
  def __init__(self):
    super().__init__('status_sub')
    self.StatusSub = self.create_subscription(String, '/mini/hearbeat', self.cb, 10)
    self.timer = self.creat_timer(0.5, self.cb)
    self.cnt = 0

  def cb(self, msg):
    self.cnt += 1
    self.get_logger().info(f'heartbeat sub cnt: {cnt}')

rclpy.init()
rclpy.spin(StatusSub)
rclpy.shutdown()
