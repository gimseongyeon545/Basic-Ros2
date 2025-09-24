1. `from rclpy.node import Node`
2. `self.create_timer(0.5, self.tick)`
   - rclpy.node.Node 의 `create_timer(timer_period_sec, callback, callback_group=None, clock=None)` 함수
     - https://docs.ros2.org/foxy/api/rclpy/api/node.html
3. `self.get_logger().info(f'beat {self.count}')`
   - rclpy.node.Node 의 `get_logger()` 함수
     - https://docs.ros2.org/foxy/api/rclpy/api/node.html
4. `rclpy.init(); rclpy.spin(Heartbeat()); rclpy.shutdown()`
   - https://docs.ros2.org/foxy/api/rclpy/api/init_shutdown.html
   - `rclpy.init(*, args=None, context=None)`
   - `rclpy.spin(node, executor=None)`
   - `rclpy.shutdown(*, context=None)`
