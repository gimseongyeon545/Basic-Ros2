# timer_log.py

1. `from rclpy.node import Node`
2. `super().__init__('heartbeat')`
   - Node 클래스의 init 함수에서 node_name 을 반드시 받도록 정의됨
      - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L127
4. `self.create_timer(0.5, self.tick)`
   - rclpy.node.Node 의 `create_timer(timer_period_sec, callback, callback_group=None, clock=None)` 함수
     - https://docs.ros2.org/foxy/api/rclpy/api/node.html
5. `self.get_logger().info(f'beat {self.count}')`
   - rclpy.node.Node 의 `get_logger()` 함수
      - https://docs.ros2.org/foxy/api/rclpy/api/node.html
  - 반환: RcutilsLogger class
    - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L385
  - RcutilsLogger class
    - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/impl/rcutils_logger.py#L418
6. `rclpy.init(); rclpy.spin(Heartbeat()); rclpy.shutdown()`
   - https://docs.ros2.org/foxy/api/rclpy/api/init_shutdown.html
   - `rclpy.init(*, args=None, context=None)`
   - `rclpy.spin(node, executor=None)`
   - `rclpy.shutdown(*, context=None)`
