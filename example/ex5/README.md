# tf2_listener_pose.py
1. `from tf2_ros import Buffer, TransformListener`
   - `self.buf = Buffer()`
     - https://github.com/ros2/geometry2/blob/rolling/tf2_ros_py/tf2_ros/buffer.py#L54
       > `def __init__(
        self,
        cache_time: Optional[Duration] = None,
        node: Optional[Node] = None
    )`
     - buffer 생성
   - `self.tl = TransformListener(self.buf, self)`
     - https://github.com/ros2/geometry2/blob/rolling/tf2_ros_py/tf2_ros/transform_listener.py#L49
       > `def __init__(self, buffer: Buffer, node: Node, *, spin_thread: bool = False, qos: Optional[Union[QoSProfile, int]] = None, static_qos: Optional[Union[QoSProfile, int]] = None, tf_topic: str = DEFAULT_TF_TOPIC, tf_static_topic: str = DEFAULT_STATIC_TF_TOPIC, static_only: bool = False) -> None:`
     - /tf, /tf_static topic (tf2_msgs/msg/TFMessage 메세지 타입) 변환을 TF brodcaster 들이 구독하고 TransformListener 가 받아서 생성했던 buffer 채워짐
       - /tf: 동적 변환 스트림
       - /tf_static: 정적 변환
       - 두 topic 을 통해 변환 msg 들어옴
     - `out = self.buf.transform(msg, 'base_link', timeout=Duration(seconds=0.2))`
       - buffer 에 쌓인 TF chain(frame 들의 변환으로 연결된 경로) 을 사용하여 PoseStamped (위치+자세) 변환
         - camera 기준 PoseStamped 를 robot 기준 PoseStamepd 로 변환
       - buffer class 는 BufferInterface class 를 상속
         - `class Buffer(tf2.BufferCore, BufferInterface):`
         - BufferInterface class 의 transform 함수
           - https://github.com/ros2/geometry2/blob/rolling/tf2_ros_py/tf2_ros/buffer_interface.py#L75
           - https://github.com/ros2/geometry2/blob/rolling/tf2_ros_py/tf2_ros/buffer_interface.py#L63
         
     - `self.pub.publish(out)`
       - pub 가 변환된 메세지를 publish 하여 메세지 내보냄
2. `self.sub = self.create_subscription(PoseStamped, '/pose_in_camera', self.cb, 10) # 환경에 맞게`
     - http://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L1662
       > `create_subscription(msg_type, topic, callback, qos_profile, *, callback_group=None, event_callbacks=None, raw=False)`
   - `self.pub = self.create_publisher(PoseStamped, '/pose_in_base', 10) # 환경에 맞게`
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L1560
       > `create_publisher(msg_type, topic, qos_profile, *, callback_group=None, event_callbacks=None)`
   - topic 명은 `ros2 topic list` 로 확인 후 변경
3. [최종 과정 및 도식화]
    ```
    self.buf = Buffer()
    self.tl = TransformListener(self.buf, self)
    -> buffer 생성 및 TransformListener 는 buf 에 /tf, /tf_static 토픽을 통해 들어오는 TransformStamped(프레임 변환) msg를 받아 Buffer에 저장
    
    self.sub = self.create_subscription(PoseStamped, '/pose_in_camera', self.cb, 10)
    self.pub = self.create_publisher(PoseStamped, '/pose_in_base', 10)
    -> sub, pub 생성
    -> sub 는 /pose_in_camera 라는 토픽을 구독중 (self.cb 는 callback 으로, 구독해둔 topic 을 통해 메세지를 받으면 호출됨 / 실제 camera 에서 오는 msg 를 /pose_in_camera 토픽을 통해 받으면 cb 함수 실행
    
    out = self.buf.transform(msg, 'base_link', timeout=Duration(seconds=0.2))
    -> buf 에 담아놓은 TransformStamped msg 를 보고 입력 PoseStamped msg 를 base link 기준의 PoseStamped 로 변환
    
    self.pub.publish(out)
    -> transform 한 결과를 publish 하고 /pose_in_base 라는 토픽으로 메세지 내보냄
    -> publish 가 메세지를 주면 /pose_in_base topic 을 구독 중인 robot node 의 subscriber 가 처리
    
    
    
     [Camera Node]
          │
          │  publishes
          │  Topic: /pose_in_camera
          │  Msg  : geometry_msgs/msg/PoseStamped
          ▼
    ┌───────────────────────────────┐
    │   Sub/Pub Node (변환 노드)    │
    │   - SUB: /pose_in_camera      │
    │   - PUB: /pose_in_base        │
    │   - 내부: buf.transform(...)   │
    └───────────────────────────────┘
          │
          │  publishes
          │  Topic: /pose_in_base
          │  Msg  : geometry_msgs/msg/PoseStamped
          ▼
     [Robot Node]
       subscribes /pose_in_base
    ```
4. `import tf2_geometry_msgs`
   - transform 함수에서 들어온 메세지 타입을 변환msg로 매핑하는 부분
     - https://github.com/ros2/geometry2/blob/rolling/tf2_ros_py/tf2_ros/buffer_interface.py#L97
       > `do_transform = self.registration.get(type(object_stamped))`
       - 등록해둔 변환 함수 꺼냄
         - 등록된 변환함수는 tf2_geometry_msgs 의 함수들
           - https://github.com/ros2/geometry2/blob/rolling/tf2_geometry_msgs/src/tf2_geometry_msgs/tf2_geometry_msgs.py#L373
5. `from rclpy.duration import Duration` & `Duration(seconds=0.2)`
   - https://github.com/ros2/rclpy/blob/master/rclpy/rclpy/duration.py#L22
     > `def __init__(self, *, seconds: Union[int, float] = 0, nanoseconds: int = 0):`
