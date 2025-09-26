# param_goal_publisher.py
1. `from geometry_msgs.msg import PoseStamped`
2. `declare_parameters()`
   - rclpy.node.Node 의 `declare_parameters(namespace, parameters, ignore_override=False)`
     > parameters (List[Union[Tuple[str], Tuple[str, Any], Tuple[str, Any, ParameterDescriptor]]]) – List of tuples with parameters to declare.
     - https://docs.ros2.org/foxy/api/rclpy/api/node.html#rclpy.node.Node.declare_parameters
   - `('target_xyz', [0.40, 0.20, 0.12]),
            ('target_rpy', [0.0, math.pi, 0.0]),  # 수직 하향
            ('frame_id', 'base_link')`
     - xyz & rpy
       - xyz: base frame (ex: base_link) 에서 본 end-effector position
       - rpy: xyz position 에 있는 end-effector 의 orientation (Roll(φ)/Pitch(θ)/Yaw(ψ) = x/y/z axis)
         > <img width="300" height="200" alt="image" src="https://github.com/user-attachments/assets/5d364bc4-fd58-4858-b5bc-8fff08ee5a29" />
         > <img width="300" height="250" alt="image" src="https://github.com/user-attachments/assets/d4b1fb45-33c1-4eea-94e1-6046b6c17496" />

     - frame_id: 
3. `self.create_publisher(PoseStamped, '/target_pose', 10)`
   - https://docs.ros2.org/foxy/api/rclpy/api/node.html#rclpy.node.Node.create_publisher
   - `self.pub.publish(pose)`
4. `self.get_parameter('target_xyz').value` & `self.get_parameter('target_rpy').value`
   - https://docs.ros2.org/foxy/api/rclpy/api/node.html#rclpy.node.Node.get_parameter
     > `get_parameter(name)`
   - 반환: Parameter class 객체
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L715
   - `.value` : Parameter class 객체의 value 속성 (`@property`)
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/parameter.py#L254
5. `pose = PoseStamped()`
   - `pose.header.frame_id = self.get_parameter('frame_id').value`
   - `pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q`
6. `import tf_transformations as tft`
   - https://github.com/DLu/tf_transformations/blob/main/tf_transformations/__init__.py
   - `tft.quaternion_from_euler(*rpy)`
     - https://github.com/DLu/tf_transformations/blob/main/tf_transformations/__init__.py#L745
     - 반환: `_reorder_output_quaternion(
        transforms3d.euler.euler2quat(ai, aj, ak, axes=axes)
    )`
       - `_reorder_output_quaternion()` func
          - https://github.com/DLu/tf_transformations/blob/main/tf_transformations/__init__.py#L739
            > return x, y, z, w
