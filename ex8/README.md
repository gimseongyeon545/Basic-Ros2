# rgbd_sync.py
1. `from message_filters import Subscriber, ApproximateTimeSynchronizer`
   - https://github.com/ros2/message_filters/blob/rolling/src/message_filters/__init__.py#L69
   - https://github.com/ros2/message_filters/blob/rolling/src/message_filters/__init__.py#L245
2. `self.sub_c  = Subscriber(self, Image, '/color/image_raw')
        self.sub_d  = Subscriber(self, Image, '/aligned_depth_to_color/image_raw')
        self.sub_ci = Subscriber(self, CameraInfo, '/color/camera_info')`
   - https://github.com/ros2/message_filters/blob/rolling/src/message_filters/__init__.py#L78
     > `def __init__(self, *args, **kwargs):`
     > `self.sub = self.node.create_subscription(*args[1:], self.callback, **kwargs)`
       - args 들 (topic 들) 받아서 create_subscription 으로 한 번에 sub 생성

4. `self.sync = ApproximateTimeSynchronizer([self.sub_c, self.sub_d, self.sub_ci], queue_size=10, slop=0.05)`
   - https://github.com/ros2/message_filters/blob/rolling/src/message_filters/__init__.py#L263
   - 각 sub 들에서 들어오는 각 msg 들의 time stamp 간 차이가 `slop` 이하 이면 topic 별 buffer 에 각 msg 저장
     > `def __init__(self, fs, queue_size, slop,
                 queue_offset=False,
                 allow_headerless=False,
                 sync_arrival_time=False):`
   - `self.sync.registerCallback(self.cb)`
     - https://github.com/ros2/message_filters/blob/rolling/src/message_filters/__init__.py#L52
     - 공통 call back 등록
     - 등록된 call back 함수를 실행하는 과정
       ```
       1. `registerCallback` 함수에서 `self.callbacks[conn] = (cb, args)`
         - https://github.com/ros2/message_filters/blob/rolling/src/message_filters/__init__.py#L61
       2. Subscriber 객체 생성 시 `self.sub = self.node.create_subscription(*args[1:], self.callback, **kwargs)`
         - https://github.com/ros2/message_filters/blob/rolling/src/message_filters/__init__.py#L83
         - callback func: https://github.com/ros2/message_filters/blob/rolling/src/message_filters/__init__.py#L85
         - signalMessage func: https://github.com/ros2/message_filters/blob/rolling/src/message_filters/__init__.py#L64
           - call back 함수에 인자 넣어서 호출
             > `for (cb, args) in self.callbacks.values(): cb(*(msg + args))`
       ```
   - `self.get_logger().info(f'sync OK t={color.header.stamp.sec}.{color.header.stamp.nanosec:09d}')`
