import rclpy


class timer(Node):
  def __init__(self):
    super().__init__('timer')
    self.cnt = 0
  def get_timer(self):
    self.timer = self.create_timer(0.5, self.cb)
    self.cnt += 1

  def cb(self, msg):
    self.log = self.get_logger().info(f'timer: {self.cnt}')

rclpy.init()
rclpy.spin(timer)
rclpy.shutdown()
