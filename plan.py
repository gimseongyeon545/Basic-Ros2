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

# (선택) IK 서비스가 필요하면
from moveit_msgs.srv import GetPositionIK

# (선택) TF 변환이 필요하면
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class Planner(Node):
    init:
        super().__init__('planner')
        subscribe to /target_pose (PoseStamped) -> cb_target
        create action client: FollowJointTrajectory -> /arm_controller/follow_joint_trajectory
        create action client: GripperCommand      -> /gripper_controller/gripper_cmd
        (optional) create IK client: GetPositionIK -> /compute_ik
        (optional) tf2 Buffer/Listener for frame checks
        set config params: group_name, eef_link, pre_grasp_offset_z, speed_scale, accel_scale

    cb_target(msg: PoseStamped):
        pose_base = ensure frame is base_link (tf2 transform if needed)
        pre_grasp = pose_base lifted by +pre_grasp_offset_z
        approach_line = straight-down segment from pre_grasp to grasp
        retreat_line  = straight-up segment back to pre_grasp

        # (옵션) IK:
        #   call IK on pre_grasp to get seed joints
        #   if fail -> retry a few times / slightly adjust yaw

        # 1) plan joint trajectory for move to pre_grasp (via MoveIt OR simple IK+time-param)
        jtraj_to_pre = plan_joint_trajectory(target=pre_grasp)

        # 2) execute trajectory
        send FollowJointTrajectory goal with jtraj_to_pre, wait result

        # 3) approach (cartesian-like step): small linear descent
        jtraj_approach = plan_or_synthesize_small_linear_segment(to=grasp)
        execute it

        # 4) close gripper
        send GripperCommand(position=close, effort=...) and wait

        # 5) retreat (linear up)
        jtraj_retreat = plan_or_synthesize_small_linear_segment(to=pre_grasp)
        execute it

        # (선택) move to place pose with same pattern

    plan_joint_trajectory(target_pose):
        # Hint only:
        # - if using MoveIt: request a plan (service/action) with constraints & speed scaling
        # - otherwise: run IK -> build JointTrajectory with reasonable timing
        return JointTrajectory

    execute_joint_trajectory(jtraj):
        build FollowJointTrajectory.Goal from jtraj
        send via ActionClient, wait for result, handle errors/timeouts

    control_gripper(open_or_close):
        build GripperCommand.Goal (position/effort)
        send & wait
