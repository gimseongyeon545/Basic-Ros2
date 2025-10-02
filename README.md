# PROJECT 1
1. Project Content
   - 임의의 목표 지점을 주면 ee의 목표 지점으로 할당하여, IK 를 계산하여 pick-place plannig 을 실행
   - moveit 으로 planning 계획하여 Simulation & Physical Robot 실행
     - args 로 only-plan 이 아닐 경우만 Physical Robot 실행
2. Package Configuration
   - Package Name
     - `gen3_lite_pickplace`
       ```
       # 0) 작업 폴더
       mkdir -p ~/ks/src
       cd ~/ks/src
        
       # 1) rclpy용 패키지 생성
       ros2 pkg create gen3_lite_pickplace \
         --build-type ament_python \
         --dependencies rclpy geometry_msgs moveit_msgs control_msgs tf2_ros
        
       # 2) 워크스페이스 빌드
       cd ~/ks
       colcon build
       source install/setup.bash
       ```

     - ws_kortex
       - https://github.com/Kinovarobotics/ros2_kortex
       - clone
         ```
         # 0) ks 워크스페이스
         cd ~/ks/src
          
         # 1) ros2_kortex 클론 (Humble 브랜치)
         git clone -b humble https://github.com/Kinovarobotics/ros2_kortex.git

         # 2) 의존성 설치 (워크스페이스 루트에서)
         cd ~/ks
         rosdep update
         rosdep install --from-paths src --ignore-src -r -y
        
         # 3) 빌드 & 환경설정
         colcon build
         source install/setup.bash
         ```
3. Code Description
   - /home/ubuntu_labtop_02/ks/src/gen3_lite_pickplace/setup.py
     ```
     entry_points={
         'console_scripts': [
            'goal = gen3_lite_pickplace:main',
            'plan = gen3_lite_pickplace:main;,
           
          ],
     },
     ```
   [1] `goal.py`
   - `def declare_parameters(
        self,
        namespace: str,
        parameters: Sequence[Union[
            Tuple[str],
            Tuple[str, ParameterInput],
            Tuple[str, ParameterInput, ParameterDescriptor]]],
        ignore_override: bool = False
    ) -> List[Parameter[Any]]:`
       - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L435
   - `def create_publisher(
        self,
        msg_type: Type[MsgT],
        topic: str,
        qos_profile: Union[QoSProfile, int],
        *,
        callback_group: Optional[CallbackGroup] = None,
        event_callbacks: Optional[PublisherEventCallbacks] = None,
        qos_overriding_options: Optional[QoSOverridingOptions] = None,
        publisher_class: Type[Publisher[MsgT]] = Publisher,
    ) -> Publisher[MsgT]:`
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L1560
   - `def create_timer(
        self,
        timer_period_sec: float,
        callback: Optional[TimerCallbackType],
        callback_group: Optional[CallbackGroup] = None,
        clock: Optional[Clock] = None,
        autostart: bool = True,
    ) -> Timer:`
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L1826
   - `def get_parameter(self, name: str) -> Parameter[Any]:`
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L716
   - PoseStamped
     - https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html
     - header
       - https://docs.ros2.org/foxy/api/std_msgs/msg/Header.html
     - pose
       - https://docs.ros2.org/foxy/api/geometry_msgs/msg/Pose.html
   - `def quaternion_from_euler(ai, aj, ak, axes='sxyz'):`
     - https://github.com/DLu/tf_transformations/blob/main/tf_transformations/__init__.py#L745
   - `def publish(self, msg: Union[MsgT, bytes]) -> None:`
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/publisher.py#L62
   - `def get_clock(self) -> Clock:`
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L382
   - `def get_logger(self) -> RcutilsLogger:`
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L386
     - .info()
       - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/impl/rcutils_logger.py#L418
   - `rclpy.init()`
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/__init__.py#L119
    
   [2] `plan.py`
5. Command
   - [0] 공통
      ```
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      export ROS_DOMAIN_ID=1
      
      cd ~/ks
      colcon build
      source install/setup.bash
      ```
   - [1] kortex bringup
      ```
      ros2 launch kortex_bringup gen3_lite.launch.py \
        robot_ip:=192.168.1.10
      ```
   - [2] goal node
      ```
      ros2 run gen3_lite_pickplace goal
      ```
   - [3] plan node
      ```
      ros2 run gen3_lite_pickplace plan
      ```
   
6. Result
   - Simulation (with rviz2)
   - Video (Physical Robot)
