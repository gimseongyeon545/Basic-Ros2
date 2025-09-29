# depth_pixel_to_point.py
1. `from sensor_msgs.msg import Image, CameraInfo`
   - https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html
   - https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html
2. `from cv_bridge import CvBridge`
   - `self.bridge = CvBridge()`
     - https://github.com/ros-perception/vision_opencv/blob/rolling/cv_bridge/python/cv_bridge/core.py#L47
   - `depth = self.bridge.imgmsg_to_cv2(msg)`
     - https://github.com/ros-perception/vision_opencv/blob/rolling/cv_bridge/python/cv_bridge/core.py#L147
     - 반환: `res = cvtColor2(im, 'bgr8', desired_encoding)`
   - `v, u = depth.shape[0]//2, depth.shape[1]//2`
     - 픽셀 x, y 좌표 = u, v
   - `z = depth[v, u] / 1000.0`
     - 픽셀 (u, v) 에서의 depth 값
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
           - `msg_info = sub.handle.take_message(sub.msg_type, sub.raw)` -> (CameraInfo(), MessageInfo())
             - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L482
           - `msg_tuple: Union[Tuple[Msg], Tuple[Msg, MessageInfo]] = (msg_info[0], )` -> (CameraInfo(), )
             - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L487
           - `await await_or_execute(sub.callback, *msg_tuple)` -> await_or_execute(self.caminfo, CameraInfo())
             - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L492
             - await_or_execute func: `callback(*args)` -> caminfo(CameraInfo())
               - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L134
         ```
     - `self.K = np.array(msg.k).reshape(3,3)`
       - `msg.k`: msg 가 CameraInfo type 이고, k field 를 이용하여 3*3 카메라 내재행렬 (cameraintrinsic matrix) 을 self.K 에 저장
         - 내재 행렬: 픽셀 좌표, 카메라 좌표 변환
           - `fx, fy, cx, cy = self.K[0,0], self.K[1,1], self.K[0,2], self.K[1,2]`
             > <img width="150" height="90" alt="image" src="https://github.com/user-attachments/assets/ddedbf59-a762-4c71-870e-41eca7b440ce" />
   - `self.create_subscription(Image, '/aligned_depth_to_color/image_raw', self.depth_cb, 10)`
     - 위와 동일
4. `self.get_logger().info(f'center 3D = ({x:.3f}, {y:.3f}, {z:.3f}) m')`
   - `x = (u - cx) * z / fx`
   - `y = (v - cy) * z / fy`
   - x, y, z: 픽셀 (u, v) 이 가리키는 camera TF 기준 3D 점의 좌표
