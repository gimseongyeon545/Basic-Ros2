# action_gripper_client.py
1. `from rclpy.action import ActionClient`
   - https://docs.ros2.org/foxy/api/rclpy/api/actions.html
     > `rclpy.action.client.ActionClient(node, action_type, action_name, *,)`
   - `self.ac = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')`
     - action name 은 `ros2 action list -t` 확인
   - `if not self.ac.wait_for_server(timeout_sec=0.0):`
     - https://docs.ros2.org/foxy/api/rclpy/api/actions.html#rclpy.action.client.ActionClient.wait_for_server
   - `goal_future = self.ac.send_goal_async(goal)`
     - https://docs.ros2.org/foxy/api/rclpy/api/actions.html#rclpy.action.client.ActionClient.send_goal_async
       > `send_goal_async(goal, feedback_callback=None, goal_uuid=None)`
     - Send a goal and asynchronously get the result.
2. `from control_msgs.action import GripperCommand`
   - https://docs.ros.org/en/noetic/api/control_msgs/html/action/GripperCommand.html
   - `goal = GripperCommand.Goal()`
     - action 의 Goal message (Goal, Result, Feedback)
   - `goal.command.position = position`
   - `goal.command.max_effort = 40.0`
3. `self.destroy_timer(self.timer)`
4. `result_future = goal_handle.get_result_async()`
5. 
