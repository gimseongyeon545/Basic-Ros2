# 03_service_client_ik.py
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting /compute_ik ...')
        self.call_ik()

    def call_ik(self):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.ik_link_name = 'end_effector_link'
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(seconds=2.0).to_msg()

        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.pose.position.x, target.pose.position.y, target.pose.position.z = (0.40, 0.20, 0.12)
        target.pose.orientation.w = 1.0
        req.ik_request.pose_stamped = target

        # --- seed RobotState (권장) ---
        seed = RobotState()
        seed.joint_state = JointState()
        seed.joint_state.name = [
            'joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'  # 환경에 맞게
        ]
        seed.joint_state.position = [0.0, -1.0, 1.2, 0.0, 0.8, 0.0]       # 합리적 초기값
        req.ik_request.robot_state = seed

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        self.get_logger().info(f'IK result: {res.error_code.val}, joints: {len(res.solution.joint_state.name)}')

rclpy.init()
node = IKClient()
rclpy.spin(node)
rclpy.shutdown()
