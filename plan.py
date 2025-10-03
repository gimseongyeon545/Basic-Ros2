# rclpy core
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# msgs
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# actions (namespaces는 네 환경에 맞게 교체)
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand

# MoveIt services (IK + Cartesian path)
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetCartesianPath

# (optional) MoveIt trajectory container
from moveit_msgs.msg import RobotTrajectory

# TF2 (frame checks / transforms)
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

from builtin_interfaces.msg import Duration


class Planner(Node):
    def __init__(self):
        super().__init__('planner')

        sub = self.create_subscriber(PoseStmaped, '/target_pose', cb_target)

        self.create_client(FollowJointTrajectory, '/arm_controller/follow_joint_trajcetory')
        self.create_client(GripperCommand, '/gripper_controller/gripper_cmd')
        self.create_client(GetPositionIK, '/compute_ik')

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, )

        self.declare_parameter('group_name', 'arm')
        self.declare_paramter('ee_link', 'end_effector_link')
        self.declare_paramter('pre_grasp_offset_z', 0.2)
        self.declare_paramter('speed_scale', )
        self.declare_paramter('accel_scale', )
        

    def cb_target(self, msg):
        self.grasp = self.buf.transform(msg, 'base_link', timeout = Duration(seconds = 0.5))

        self.pre_grasp_offset_z = self.get_parameter('pre_grasp_offset_z')
        self.pre_grasp = self.grasp.copy()
        self.pre_grasp.pose.position.z += self.pre_grasp_offset_z

        self.group_name = self.get_paramter('group_name')
        self.ee_link = self.get_paramter('end_effector_link')

        self.req = GetCartesianPath.Request()
        self.req.group_name = self.group_name
        self.req.link_name = self.ee_link

        self.req.waypoints = [self.pre_grasp.pose, self.grasp.pose] # approach_line

        self.req.
        
        

        approach_line = straight-down segment from pre_grasp to grasp
        retreat_line  = straight-up segment back to pre_grasp

        # (옵션) IK:
        #   call IK on pre_grasp to get seed joints
        #   if fail -> retry a few times / slightly adjust yaw

        # 1) plan joint trajectory for move to pre_grasp (via MoveIt OR simple IK+time-param)
        jtraj_to_pre = self.plan_joint_trajectory(target=pre_grasp)

        # 2) execute trajectory
        send FollowJointTrajectory goal with jtraj_to_pre, wait result

        # 3) approach (cartesian-like step): small linear descent
        jtraj_approach = self.plan_or_synthesize_small_linear_segment(to=grasp)
        execute it

        # 4) close gripper
        send GripperCommand(position=close, effort=...) and wait

        # 5) retreat (linear up)
        jtraj_retreat = self.plan_or_synthesize_small_linear_segment(to=pre_grasp)
        execute it

        # (선택) move to place pose with same pattern

    def plan_joint_trajectory(self, target_pose):
        # Hint only:
        # - if using MoveIt: request a plan (service/action) with constraints & speed scaling
        # - otherwise: run IK -> build JointTrajectory with reasonable timing
        return JointTrajectory

    def execute_joint_trajectory(self, jtraj):
        build FollowJointTrajectory.Goal from jtraj
        send via ActionClient, wait for result, handle errors/timeouts

    def control_gripper(self, open_or_close):
        build GripperCommand.Goal (position/effort)
        send & wait

def main():
    rclpy.init()
    rclpy.spin(Planner)
    rclpy.shutdown()
