# rclpy core
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# msgs
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# action: .wait_for_server() / .Goal().field
# send goal: .send_goal_async(goal) -> .spin_until_future_complete() -> .result().field
# get result: .get_result_async() -> .spin_until_future_complete() -> .result().field
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand

# MoveIt services (IK + Cartesian path)
# service: create_client / .Request().field
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetCartesianPath

# MoveIt trajectory container
from moveit_msgs.msg import RobotTrajectory

# TF2 (frame checks / transforms)
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

from builtin_interfaces.msg import Duration


class Planner(Node):
    def __init__(self):
        super().__init__('planner_min')

        # --- Params (set to your robot) ---
        # planning frame + group/link names + ordered group joint list
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('group_name', 'arm')
        self.declare_parameter('ee_link', 'end_effector_link')
        self.declare_parameter('group_joint_names', ['j1','j2','j3','j4','j5','j6'])
        self.declare_parameter('move_time_sec', 3.0)   # simple duration for single-shot move

        # --- Subscriptions ---
        self.sub_goal = self.create_subscription(PoseStamped, '/target_pose', self.cb_target, 10)
        self.sub_js   = self.create_subscription(JointState,  '/joint_states', self.cb_js, 50)
        self.last_js = None  # seed for IK

        # --- Service client (IK) ---
        self.cli_ik = self.create_client(GetPositionIK, '/compute_ik')
        # wait_for_service(...)  # (do in real code)

        # --- Action client (execute) ---
        self.ac_arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        # wait_for_server(...)  # (do in real code)

        # --- TF2 ---
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)

        # state flag (avoid overlapping goals)
        self.busy = False

    # --------------------

    def cb_js(self, msg: JointState):
        # store latest joint state for IK seed
        self.last_js = msg

    # --------------------

    def cb_target(self, msg_in: PoseStamped):
        if self.busy:
            return  # drop or queue; keep minimal

        self.busy = True

        # 0) normalize target to base_frame using TF (if frames differ)
        # base_frame = get_param('base_frame')
        # grasp = TF2 transform(msg_in -> base_frame) with timeout
        # on TF failure: self.busy=False; return

        # 1) build IK request for target EE pose
        # req = GetPositionIK.Request()
        # req.ik_request.group_name    = get_param('group_name')
        # req.ik_request.ik_link_name  = get_param('ee_link')
        # req.ik_request.pose_stamped  = grasp
        # if self.last_js: req.ik_request.robot_state.joint_state = self.last_js
        # call IK (call_async + spin_until_future_complete) -> res

        # 2) extract joint positions in group order
        # names_all  = res.solution.joint_state.name
        # pos_all    = res.solution.joint_state.position
        # group      = get_param('group_joint_names')  # ordered list
        # q_target   = [pos_all[names_all.index(nm)] for nm in group]
        # on failure/missing: self.busy=False; return

        # 3) build minimal JointTrajectory (single point → controller interpolates)
        # jt = JointTrajectory()
        # jt.joint_names = group
        # pt = JointTrajectoryPoint()
        # pt.positions = q_target
        # pt.time_from_start = Duration(sec=int(get_param('move_time_sec')))
        # jt.points = [pt]

        # 4) execute via FollowJointTrajectory action
        # goal = FollowJointTrajectory.Goal()
        # goal.trajectory = jt
        # send_goal_async(...); wait; get_result_async(...); wait
        # (check wrapped.status; log)

        self.busy = False

    # --------------------

    # (Optional) helper to read param quickly
    # def get_param(self, name): return self.get_parameter(name).get_parameter_value().string_value / double_value / ...

def main():
    rclpy.init()
    node = Planner()
    rclpy.spin(node)
    rclpy.shutdown()


    

def main():
    rclpy.init()
    rclpy.spin(Planner)
    rclpy.shutdown()
