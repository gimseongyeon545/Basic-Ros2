# depth_pixel_to_point.py
1. `from sensor_msgs.msg import Image, CameraInfo`
   - https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
   - https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
2. `from cv_bridge import CvBridge`
   - `self.bridge = CvBridge()`
   - `depth = self.bridge.imgmsg_to_cv2(msg)`
   - `v, u = depth.shape[0]//2, depth.shape[1]//2`
   - `z = depth[v, u] / 1000.0`
3. sub, pub
   - `self.create_subscription(CameraInfo, '/color/camera_info', self.caminfo, 1)`
     - http://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L1662
       > `create_subscription(msg_type, topic, callback, qos_profile, *, callback_group=None, event_callbacks=None, raw=False)`
     - msg 가 도착할 때까지 `self.caminfo` callback 을 실행
       - callback 호출이 실행될 때, 인자는 `spin()` 시 executor 에서 담당
         ```
         - `spin`: https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L353
         - `spin_once`: https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L941
         - `_spin_once_impl`: https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L919
         - `wait_for_ready_callbacks`: https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L875
         - `_wait_for_ready_callbacks`: https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L641
         - `handler = self._make_handler(sub, node, self._take_subscription)`
           - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L839
         - `take_subscription`: https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L478
           - `msg_info = sub.handle.take_message(sub.msg_type, sub.raw)` ->
             - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L482
           - `await await_or_execute(sub.callback, *msg_tuple)` -> 
             - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L492
         ```
     - `self.K = np.array(msg.k).reshape(3,3)`
       - 
       - `fx, fy, cx, cy = self.K[0,0], self.K[1,1], self.K[0,2], self.K[1,2]`
   - `self.create_subscription(Image, '/aligned_depth_to_color/image_raw', self.depth_cb, 10)`
4. `self.get_logger().info(f'center 3D = ({x:.3f}, {y:.3f}, {z:.3f}) m')`
   - `x = (u - cx) * z / fx`
   - `y = (v - cy) * z / fy`
