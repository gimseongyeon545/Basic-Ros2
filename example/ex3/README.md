# service_client_ik.py
1. `from moveit_msgs.srv import GetPositionIK`
   - https://docs.ros.org/en/noetic/api/moveit_msgs/html/srv/GetPositionIK.html
   - `GetPositionIK`: service type
   - `req = GetPositionIK.Request()`
     - `Request()` & `Response()` (for `srv`)
       - service request/response message type
       - `GetPositionIK` class 에 속성으로 `Request`, `Response` class
       - 반환(`req` 형태): Request class의 object
   - https://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/PositionIKRequest.html
      - `req.ik_request.group_name = 'arm'`                # 환경에 맞게
      - `req.ik_request.ik_link_name = 'end_effector_link'` # 환경에 맞게
      - `req.ik_request.avoid_collisions = False`
      - `req.ik_request.timeout.sec = 2` & `req.ik_request.timeout.nanosec = 0`
        - https://docs.ros2.org/foxy/api/builtin_interfaces/msg/Duration.html
      - `req.ik_request.pose_stamped = target`

</br>

2. `self.cli = self.create_client(GetPositionIK, '/compute_ik')`
   - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L1740
     > `create_client(srv_type, srv_name, *, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)`
   - 반환: Client[SrvRequestT, SrvResponseT] (Client class 객체)
   - GetPositionIK service 를 제공하는 server 에 request 하는 client 생성
     - server: moveit 의 move_group -> move_group node 를 띄우면 /compute_ik 라는 GetPositionIK service 를 제공
   - `while not self.cli.wait_for_service(timeout_sec=1.0):`
     - 반환된 Client class 객체의 `wait_for_service` 함수
       - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/client.py#L180
   - `future = self.cli.call_async(req)`
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/client.py#L120
       > `def call_async(self, request: SrvRequestT) -> Future[SrvResponseT]:`
       - 반환: `Future[SrvResponseT]` & `self._pending_requests[sequence_number] = future` & `return future`
         - `self._pending_requests: Dict[int, Future[SrvResponseT]] = {}` (https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/client.py#L69)
         - Future class: https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/task.py#L44
     - 생성한 client 가 만든 요청 메세지 (`req`) 를 비동기로 server 에 Request 하고 비어있는 Future class 객체 반환
   - `rclpy.spin_until_future_complete(self, future)`
     - https://docs.ros2.org/foxy/api/rclpy/api/init_shutdown.html#rclpy.spin_until_future_complete
       > `rclpy.spin_until_future_complete(node, future, executor=None, timeout_sec=None)`
       > Execute work until the future is complete.
       - 응답 메세지 도착할 때까지 대기 후 응답이 오면 Future class 객체에 응답 메세지 (`GetPositionIK.Response`) 생성
      - move_group 이 Response
        - `spin_until_future_complete` 함수 실행 후 Response message 가 `self._result` 에 할당되는 과정
        ```
        [1] spin_until_future_complete
        - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L358
      
        [2] _spin_once_until_future_complete
        - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L944
      
        [3] _spin_once_impl
         - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L919
         - 이때, `wait_for_ready_callbacks` 만 실행하고, handler() 대기 (_make_handler 의 async def handler)
           - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L934
      
        [4] wait_for_ready_callbacks
        - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L875
      
        [5] _wait_for_ready_callbacks
        - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L641C9-L641C34
        - cliient for문
           - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L850
           - _make_handler 에서 호출하는 함수는 `_take_clilent`
              - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L853

        [6] _make_handler
        - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L575
        - 내부: async def handler 정의: https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L592
           - 이때 아직 실행 안되고 Task 객체만 생성해서 반환
        - return: `task: Task[None] = Task(
                  handler, (entity, self._guard, self._is_shutdown, self._work_tracker),
                  executor=self)` & `return task`
        [7] `_spin_once_impl` 로 다시 와서 handler() func 실행
               - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L934
        - 받는 인자가 entity, node, take_from_wait_list
           > `def _make_handler(
                  self,
                  entity: 'EntityT',
                  node: 'Node',
                  take_from_wait_list: Callable[['EntityT'],
                                                Optional[Callable[[], Coroutine[None, None, None]]]],
                  ) -> Task[None]:`
        -  위 `handler = self._make_handler(client, node, self._take_client)`에서 호출한대로 인자 받고 `call_coroutine = take_from_wait_list(entity)`
            - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L853
            - `take_from_wait_list(entity)` 는 `self._take_client(client)` 을 실행
          - `_take_client`
            - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L504
            - `set_result` 함수 호출 부분
               - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L522
        ```
   - `res = future.result()`
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/task.py#L103
     - 반환: `return self._result`
       - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/task.py#L19
     
   - `self.get_logger().info(
            f'IK error_code={res.error_code.val} | '
            f'joints={len(res.solution.joint_state.name)} | '
            f'names={list(res.solution.joint_state.name)}'
        )`
     - 꺼낸 응답 메세지 res 가 `moveit_msgs.srv.GetPositionIK.Response`
       - https://docs.ros.org/en/noetic/api/moveit_msgs/html/srv/GetPositionIK.html 

</br>

3. `target.pose.orientation.w = 1.0`
   - x, y, z 는 default 0, 0, 0
