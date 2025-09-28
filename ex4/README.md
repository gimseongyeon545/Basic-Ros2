# action_gripper_client.py
1. `from rclpy.action import ActionClient`
   - https://docs.ros2.org/foxy/api/rclpy/api/actions.html
   - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/action/client.py#L165
     > `rclpy.action.client.ActionClient(node, action_type, action_name, *,)`
   - `self.ac = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')`
     - action name 은 `ros2 action list -t` 확인
   - `if not self.ac.wait_for_server(timeout_sec=0.0):`
     - https://docs.ros2.org/foxy/api/rclpy/api/actions.html#rclpy.action.client.ActionClient.wait_for_server
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/action/client.py#L659
   - [1] `goal_future = self.ac.send_goal_async(goal)`
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/action/client.py#L485https://docs.ros2.org/foxy/api/rclpy/api/actions.html#rclpy.action.client.ActionClient.send_goal_async
       > `send_goal_async(goal, feedback_callback=None, goal_uuid=None)`
     - Send a goal and asynchronously get the result.
     - 반환: `return future` & `future: Future[ClientGoalHandle[GoalT, ResultT, FeedbackT]] = Future()`
     - `rclpy.spin_until_future_complete(self, goal_future, timeout_sec=5.0)`
     - `goal_handle = goal_future.result()`
        - (ex3 ref.) [8] 에서 goal
          ```
          [8] async def handler 실행
          - 대기 중인 coroutine 확보
             - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L603
          - await call_coroutine()
             - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L611
             - ActionClient.execute() 에서 pending dict (call_async 에서 반환됨) 를 참고하고 나온 request 를 set_result 함수로 self._result 에 값을 넣어줌
                - `self._pending_goal_requests[sequence_number].set_result(goal_handle)`
                - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/action/client.py#L367
                - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/task.py#L132
          ```
        - 반환: **rclpy.action.client.ClientGoalHandle**
     - `if not goal_handle or not goal_handle.accepted:` (accepted property)
        - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/action/client.py#L119
   - [2] `result_future = goal_handle.get_result_async()`
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/action/client.py#L155
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/action/client.py#L620
       > `def _get_result_async(self, goal_handle: ClientGoalHandle[GoalT, ResultT, FeedbackT]
                          ) -> Future[GetResultServiceResponse[ResultT]]:`
     - `rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)`
     - `result = result_future.result()`
       - (ex3 ref.) [8] 에서 goal
          ```
          [8] async def handler 실행
          - 대기 중인 coroutine 확보
             - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L603
          - await call_coroutine()
             - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L611
             - ActionClient.execute() 에서 pending dict (call_async 에서 반환됨) 를 참고하고 나온 request 를 set_result 함수로 self._result 에 값을 넣어줌
                - `self._pending_result_requests[sequence_number].set_result(result_response)`
                - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/action/client.py#L384
                - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/task.py#L132
          ```
        - 반환: **status(action_msgs.msg.GoalStatus), result(control_msgs.action.GripperCommand.Result)**

</br>

2. `from control_msgs.action import GripperCommand`
   - https://docs.ros.org/en/noetic/api/control_msgs/html/action/GripperCommand.html
   - `goal = GripperCommand.Goal()`
     - action 의 Goal message (Goal, Result, Feedback)
   - https://docs.ros.org/en/noetic/api/control_msgs/html/msg/GripperCommand.html
     - `goal.command.position = position`
     - `goal.command.max_effort = 40.0`
     - Result, Feedback msg 는 /control_msgs/action/GripperCommand
       - https://docs.ros.org/en/noetic/api/control_msgs/html/action/GripperCommand.html

</br>

3. `self.destroy_timer(self.timer)`
   - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L1976
   - Timer class 의 destroy func
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L1985
     - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/timer.py#L122
