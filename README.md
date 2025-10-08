# basic ros2 example (with rclpy)
- Understanding ROS2 nodes/topics/services/parameters/actions
  - https://docs.ros.org/en/humble/Tutorials.html
## 0. configuring environment
- DOMAIN
  ```
  export ROS_DOMAIN_ID=<your_domain_id>
  ```
- source setup files
  ```
  source /opt/ros/humble/setup.bash
  ```
  - don't want to have to source the setup file every time
    ```
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```
- work space 사용하는 경우
  - `source ~ws/install/setup.bash`
 
</br>

## 1. ROS2 nodes
- [1] graph
  - 동시에 데이터를 처리하는 ROS2 요소들의 네트워크 (연결)
- [2] nodes
  - 각 노드는 topics, services, actions, parameters 를 통해 다른 노드와 데이터 주고받음
    > ![Image](https://github.com/user-attachments/assets/fc53e75c-9def-488d-9c1b-5c325c763ea6)
- [3] 실행
  - `ros2 run <package_name> <executable_file_name>`
- [4] node list & node info
  - `ros2 node list`
    - run 한 node 이름
  - `ros2 node info /<node name>`
- [5] remapping
  - `ros2 run <package_name> <executable_file_name> --ros-args -r <args name>:= `

</br>

## 2. ROS2 topics
- [1] topics
  - 동시에
    > ![Image](https://github.com/user-attachments/assets/50a9f958-113a-4423-8674-3b5de8e9629a)
- [2] rqt_graph
  - `rqt_graph`
     - 사각형: topics / 원: nodes / 파랑: publisher, 초록: subscriber
       > <img width="1546" height="965" alt="image" src="https://github.com/user-attachments/assets/76278aef-5154-45df-b0fc-e74c66c5fb51" />
- [3] topic list & info
  - `ros2 topic list`: topic name
  - `ros2 topic list -t`: topic name, type(type: `pacakage/msg/msg_name`)
  - `ros2 topic info <topic 명>`
    - Type, publisher, subscriprion counts 출력
- [4] interface show
  - `ros2 interface show <msg_type>`
  - 메세지가 기대하는 데이터 구조
- [5] echo
  - `ros2 topic echo <topic name>`
  - interface show 가 기대하는 데이터 구조를 만족하는 데이터 반환
  - echo 결과 `/_ros2cli_~` 라는 노드 생성됨
- [6] publish
  - `ros2 topic pub <topic_name> <msg_type> '<args>'`
    - ex) ` ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"`
    - no command: 1초에 msg 한 번씩 보냄 (1 Hz)
    - `ros2 topic pub --once -w 2 <topic_name> <msg_type> '<args>'`
      - pub 전 2초 wait 후 한 번 msg 보냄
- [7] topic hz
  - `ros2 topic hz <topic name>`
    - publish 하는 average rate, min, max, std dev, window 출력
- [8] topic bw
  - `ros2 topic bw <topic name>`
    - bandwidth
- [9] topic find
  - `ros2 topic find <topic_type>`
    - 가능한 토픽들의 topic type 출력
  - `ros2 topic find <찾은 topic type>`
    - 해당 topic type 을 가지는 topic 명 반환

</br>

## 3. ROS2 services
- [1] 
## 4. ROS2 parameters
## 5. ROS2 actions
