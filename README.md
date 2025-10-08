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
    > <img width="854" height="480" alt="image" src="https://github.com/user-attachments/assets/5d998a70-ea2b-4f43-b852-b4f84d4bc4a0" />
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
    > https://github.com/gimseongyeon545/Basic-Ros2/issues/1#issue-3495637000

</br>

## 3. ROS2 services
## 4. ROS2 parameters
## 5. ROS2 actions
