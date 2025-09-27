# 04_action_gripper_client.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

class Gripper(Node):
    def __init__(self):
        super().__init__('gripper_client')
        self.ac = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        # 액션 서버 준비되면 실행
        self.timer = self.create_timer(0.1, self._kick)

    def _kick(self):
        if not self.ac.wait_for_server(timeout_sec=0.0):
            self.get_logger().info('waiting for gripper action...'); return
        self.destroy_timer(self.timer)
        self.open_then_close()

    def open_then_close(self):
        self._send_and_wait(0.08)  # 열기
        self._send_and_wait(0.00)  # 닫기

    def _send_and_wait(self, position: float):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 40.0
        goal_future = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=5.0)
        goal_handle = goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn(f'goal rejected: pos={position}'); return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        result = result_future.result()
        self.get_logger().info(f'goal done: pos={position}, status={getattr(result, "status", None)}')

def main():
    rclpy.init()
    node = Gripper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
