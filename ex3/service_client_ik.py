# 03_service_client_ik.py
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
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
        req.ik_request.group_name = 'arm'             # 환경에 맞게
        req.ik_request.ik_link_name = 'end_effector_link'
        req.ik_request.attempts = 5
        req.ik_request.timeout.sec = 2
        
        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.pose.position.x, target.pose.position.y, target.pose.position.z = (0.4, 0.2, 0.12)
        target.pose.orientation.w = 1.0
        
        req.ik_request.pose_stamped = target
        
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        self.get_logger().info(f'IK result: {res.error_code.val}, joints: {len(res.solution.joint_state.name)}')

rclpy.init(); rclpy.spin(IKClient()); rclpy.shutdown()
