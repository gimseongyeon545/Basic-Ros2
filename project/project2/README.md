# PROJECT 2
1. Project Contents
   - Project 1 과 동일하되, 물체 위치를 직접 주지 않고, 카메라로 찍어서 파악하기기

</br>

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
</br>

3. setup.py
   - /home/ubuntu_labtop_02/ks/src/gen3_lite_pickplace/setup.py
     ```
     entry_points={
        'console_scripts': [
           'plan = gen3_lite_pickplace.plan:main',
           'depth = gen3_lite_pickplace.depth:main',
           
        ],
     },
     ```
   
</br>
   
4. Command
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
   - [2] movegroup
     ```
     ros2 launch kinova_gen3_lite_moveit_config move_group.launch.py\
        publish_robot_description:=true
     ```
   - [3] 임의 위치로 이동
     ```
     ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
     control_msgs/action/FollowJointTrajectory \
     "{trajectory: {
       joint_names: [joint_1,joint_2,joint_3,joint_4,joint_5,joint_6],
       points: [{positions: [1.4208193447, 0.0704470024, 1.8731641801, -2.0, -0.4543651094, 0.0349651746],
       time_from_start: {sec: 10}}]}}"
     ```
   - [4] pick node ik 계산 가능 여부 확인인
     ```
     ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "
		ik_request:
		  group_name: arm
		  ik_link_name: end_effector_link   # 네 환경의 tip 링크명으로
		  avoid_collisions: true
		  timeout: {sec: 1, nanosec: 0}
		  pose_stamped:
		    header: {frame_id: base_link}
		    pose:
		      position: {x: 0.40, y: 0.00, z: 0.157}
		      orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}
		  robot_state:
		    joint_state: {name: [], position: []}  # 빈 seed면 MoveIt이 알아서 현재 상태 사용
		"	
     ```
   - [5] plan node
      ```
      ros2 run gen3_lite_pickplace plan
      ```
   - [6] pick, place topic once 보내기
     ```
     # pick (툴 아래, 약간 높게)
     ros2 topic pub --once /pick_pose geometry_msgs/PoseStamped "
		header: {frame_id: base_link}
		pose:
  		position: {x: 0.40, y: 0.00, z: 0.157}
  		orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}
		"
     ```
     ```
     # place (비슷한 높이, y 쪽으로 약간 이동)
	 ros2 topic pub --once /place_pose geometry_msgs/PoseStamped "
	 header: {frame_id: base_link}
	 pose:
	   position: {x: 0.45, y: -0.15, z: 0.171}
	   orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}
	 "
	 ```

- 산출물을 위한 realsense camera node 올리기
  ```
  ros2 run realsense2_camera realsense2_camera_node --ros-args \
	-p enable_color:=true -p enable_depth:=true \
	-p pointcloud.enable:=true -p align_depth.enable:=true \
	--log-level debug
  ```

</br>
   
5. Result
   - Simulation (with rviz2)
   - Video (Physical Robot)
