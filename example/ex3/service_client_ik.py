# 03_service_client_ik_ros2_base.py
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
# (선택) 시드 쓰려면 아래 두 줄 주석 해제
# from moveit_msgs.msg import RobotState
# from sensor_msgs.msg import JointState

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting /compute_ik ...')
        self.call_ik()

    def call_ik(self):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'                 # 환경에 맞게
        req.ik_request.ik_link_name = 'end_effector_link' # 환경에 맞게
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        req.ik_request.timeout.nanosec = 0

        target = PoseStamped()
        target.header.frame_id = 'base_link'              # 환경에 맞게
        target.pose.position.x = 0.40
        target.pose.position.y = 0.20
        target.pose.position.z = 0.12
        target.pose.orientation.w = 1.0                   # 단위 quaternion (회전 없음)
        req.ik_request.pose_stamped = target

        # --- (선택) 현재 조인트 상태를 시드로 쓰고 싶다면 이렇게 채움 ---
        # seed = RobotState()
        # seed.joint_state = JointState()
        # seed.joint_state.name = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
        # seed.joint_state.position = [0.0, -1.0, 1.2, 0.0, 0.8, 0.0]
        # req.ik_request.robot_state = seed

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res is None:
            self.get_logger().error('IK service call failed.')
            return
        self.get_logger().info(
            f'IK error_code={res.error_code.val} | '
            f'joints={len(res.solution.joint_state.name)} | '
            f'names={list(res.solution.joint_state.name)}'
        )

def main():
    rclpy.init()
    node = IKClient()
    # 한 번 호출하고 로그 찍으면 종료
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
