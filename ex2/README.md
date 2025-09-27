# param_goal_publisher.py
1. `from geometry_msgs.msg import PoseStamped`
   - https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html
   - stamp(시점) 와 `frame_id` 에서의 pose 를 담은 message

</br>

2. `declare_parameters()`
   - rclpy.node.Node 의 `declare_parameters(namespace, parameters, ignore_override=False)`
     > parameters (List[Union[Tuple[str], Tuple[str, Any], Tuple[str, Any, ParameterDescriptor]]]) – List of tuples with parameters to declare.
     - https://docs.ros2.org/foxy/api/rclpy/api/node.html#rclpy.node.Node.declare_parameters
       - 반환: `return [self._parameters[parameter.name] for parameter in parameter_list]` -> `name`, `value` 를 속성으로 하는 Parameter class 객체의 list 를 반환
          - 반환 타입: `List[Parameter[Any]]`
          - `self._parameters` (`self._parameters: Dict[str, Parameter[Any]] = {}`)
            - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L188
          - ex) `[ Parameter(name='target_xyz', value=[0.40,0.20,0.12], ...),
  Parameter(name='target_rpy', value=[0.0,3.14159,0.0], ...),
                `
   - ```
     ('target_xyz', [0.40, 0.20, 0.12]),
            ('target_rpy', [0.0, math.pi, 0.0]),  # 수직 하향
            ('frame_id', 'base_link')
     ```
     - xyz & rpy
       - xyz: base frame (ex: base_link) 에서 본 end-effector position
       - rpy: xyz position 에 있는 end-effector 의 orientation (Roll(φ)/Pitch(θ)/Yaw(ψ) = x/y/z axis)
         > <img width="300" height="250" alt="image" src="https://github.com/user-attachments/assets/5d364bc4-fd58-4858-b5bc-8fff08ee5a29" />
         
         > <img width="300" height="250" alt="image" src="https://github.com/user-attachments/assets/d4b1fb45-33c1-4eea-94e1-6046b6c17496" />
            - **base link 의 TF 도 표시한 그림 필요**
            - X (Red): roll, Y (Green): pitch, Z (Blue): yaw
            - [0.0, math.pi, 0.0]: 수직 하향
              - base link TF (불변) 대로 end-effector 의 TF 를 설정한 후 절대 rpy 만큼 rotation

</br>

3. `self.pub = self.create_publisher(PoseStamped, '/target_pose', 10)`
   - https://docs.ros2.org/foxy/api/rclpy/api/node.html#rclpy.node.Node.create_publisher
     > `create_publisher(msg_type, topic, qos_profile, *, callback_group=None, event_callbacks=None)`
        - `/target_pose`: 토픽명
           - `PoseStamped` type + `/target_pose` 라는 이름(토픽)의 message를 publish 하는 publisher 생성
        - qos_profile: Quality of Service 의 depth (KEEP_LAST) (+ RELIABLE/VOLATILE (가능한 한 모든 메세지 전달 보장 / 과거 메세지 받지 않음))
        - 반환: Publisher 객체
          - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L1560
   - `self.pub.publish(pose)`
     - Publish class 의 publish func
        - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/impl/_rclpy_pybind11.pyi#L249
     - `create_publisher` 인자로 넣은 `msg_type` 의 객체(`pose`)를 `.publish`

</br>

4. `xyz = self.get_parameter('target_xyz').value` & `rpy = self.get_parameter('target_rpy').value`
   > `get_parameter(name)`
      - https://docs.ros2.org/foxy/api/rclpy/api/node.html#rclpy.node.Node.get_parameter
      - 반환: `return self._parameters[name]` & `Parameter[Any]:` (Parameter 객체)
        - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L734
        - `self._parameters` (`self._parameters: Dict[str, Parameter[Any]] = {}`)
          - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L188
          - Dict 이므로 `get_paramer` 에서 인자 name 을 key 로 각 `declare_parameters` 에서 정의한 각 객체 자체를 불러오기
   - `.value` : Parameter class 객체의 value 속성 (`@property`)
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/parameter.py#L254
       > `return self._value`
          - Parameter class `__init__` 에서 정의: `self._value = value` & `def __init__(self, name: str, type_: Optional[Parameter.Type] = None, value=None) -> None:`
             - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/parameter.py#L230

</br>

5. `pose = PoseStamped()`
   - `pose.header.frame_id = self.get_parameter('frame_id').value`
     - https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html
   - `pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q`
     - https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html
       - Point: https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html
       - Quaternion: https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Quaternion.html
    - `pose` 라는 객체를 생성 후 하위 attribute, field (메타데이터) 지정

</br>

6. `import tf_transformations as tft`
   - https://github.com/DLu/tf_transformations/blob/main/tf_transformations/__init__.py
   - `tft.quaternion_from_euler(*rpy)`
     > `def quaternion_from_euler(ai, aj, ak, axes='sxyz'):`
     - https://github.com/DLu/tf_transformations/blob/main/tf_transformations/__init__.py#L745
     - 반환: `_reorder_output_quaternion(
        transforms3d.euler.euler2quat(ai, aj, ak, axes=axes)
    )`
       - `_reorder_output_quaternion()` func
          - https://github.com/DLu/tf_transformations/blob/main/tf_transformations/__init__.py#L739
            > return x, y, z, w
     - `*rpy`: * 은 `declare_parameters` 에서 value 를 list 로 정했으므로 unpacking 해서 `quaternion_from_euler` 함수의 인자로
   - quaternion & euler
     - euler: rpy
       > <img width="722" height="112" alt="image" src="https://github.com/user-attachments/assets/1e0e6696-fe66-43d3-b168-40825140376c" />
     - quaternion: q = (x, y, z, w)
       > <img width="347" height="32" alt="image" src="https://github.com/user-attachments/assets/aac561db-95bd-4b58-a570-7a4ee005ab69" />
     - euler to quaternion
       > <img width="266" height="143" alt="image" src="https://github.com/user-attachments/assets/51b68443-ed8b-4cae-a3bb-05d682780c0c" />
     - quaternion to euler
       > <img width="257" height="73" alt="image" src="https://github.com/user-attachments/assets/5955a626-402e-4720-b7ab-68f7c548a264" />


       
