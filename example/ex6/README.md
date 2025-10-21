# tf2_static_broadcaster.py
1. `from geometry_msgs.msg import TransformStamped`
   - https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TransformStamped.html
   - `t = TransformStamped()`
   - `t.header.frame_id = 'base_link'`
     - https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html
   - `t.child_frame_id  = 'camera_link'`
   - `t.header.stamp = now`
     - https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html
   - https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Transform.html
     - `t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (0.30, 0.0, 0.50)`
     - `t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q`
  
2. `from tf2_ros import StaticTransformBroadcaster`
   - https://github.com/ros2/geometry2/blob/rolling/tf2_ros_py/tf2_ros/static_transform_broadcaster.py#L41
   - `self.br = StaticTransformBroadcaster(self)`
     - def __init__(self, node: Node, qos: Optional[Union[QoSProfile, int]] = None) -> None:
   - `self.br.sendTransform(t)`
     - https://github.com/ros2/geometry2/blob/rolling/tf2_ros_py/tf2_ros/static_transform_broadcaster.py#L65
       > `def sendTransform(self, transform: Union[TransformStamped, List[TransformStamped]]) -> None:`
     - geometry_msgs/msg/TransformStamped 타입의 객체를 TF Broadcaster 가 /tf 토픽을 통해 publlish
       - https://github.com/ros2/geometry2/blob/rolling/tf2_ros_py/tf2_ros/static_transform_broadcaster.py#L84
3. `now = self.get_clock().now().to_msg()`
   - https://github.com/ros2/rclpy/blob/master/rclpy/rclpy/node.py#L365
     - Clock class 객체 반환
   - `.now` func
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/clock.py#L188
     - Time class 객체 반환
   - `.to_msg` func
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/time.py#L158
     - 반환: `return builtin_interfaces.msg.Time(sec=seconds, nanosec=nanoseconds)`
