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
   - `goal.py`
   - `plan.py`
5. Result
   - Simulation (with rviz2)
   - Video (Physical Robot)
