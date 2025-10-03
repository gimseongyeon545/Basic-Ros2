import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration

# MoveIt IK
from moveit_msgs.srv import GetPositionIK

# FollowJointTrajectory (arm)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

# Gripper action
from control_msgs.action import GripperCommand

# TF2 (선택적: pose가 다른 프레임으로 올 때만 사용)
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

# --- helper ---
function quat_from_rpy(roll, pitch, yaw):
    compute quaternion from roll/pitch/yaw
    return (qx, qy, qz, qw)

# --- main class ---
class Planner(Node):
  def __init__(self):
    super().__init__('plan')
    self.declare_parameter('group_name', 'arm')
    self.declare_parameter('ee_link', 'end_effector_link')
    self.declare_paramter('frame_id', 'base_link')
    self.declare_paramter('ik_service', '/compute_ik')
    self.declare_parameter('traj', '/joint_trajectory_controller')
    self.declare_parameter('gripper', '/gripper_controller/gripper_cmd')
    self.declare_paramter('joint', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
    
    self.declare_paramter('pre_offset', 0.2)
    self.declare_parameter('lift_height', 0.4)
    self.declare_parameter('move_time', 3.0)
    self.declare_paramter('grip_open', 0.4)
    self.declare_paramter('grip_close', 0.05)
    
    self.joint_state = None
    self.pick_pose   = None
    self.place_pose  = None

    # comm
    self.sub = self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

    self.ik_name = self.get_paramter('ik_service').value
    self.ik_srv = self.create_client(GetPositionIK, self.ik_name)

    self.traj_action = self.get_parameter('traj').value
    self.act1 = self.ActionClient(self, FollowJointTrajectory, self.traj_action)
    self.gripper_action = self.get_paramter('gripper').value
    self.act2 = self.ActionClient(self, GripperCommand, self.gripper_action)

    self.buf = Buffer()
    self.listener = TransformListener(self.buf, self)

    self.create_subscription(PoseStamped, '/pick_pose', self._on_pick)
    self.create_subscription(PoseStamped, '/place_pose', self._on_place)

    self.get_logger().info('Ready..')

  # callback functions
  def _js_cb(self, msg: JointState):
    self.joint_state_msg

  def _on_pick(self, msg: PoseStamped):
    self.pick_pose = msg
    self._try_run()

  def _on_place(self, msg: PoseStamped):
    self.place_pose = msg
    self._try_run()

  # run (main)
  def _try_run(self):
    if self.pick_pose is Node or self.place_pose is None: return
    
    self.base = self.get_paramter('frame_id').value
    self.pick = self._to_base(self.pick_pose, self.base)
    self.pose = self._to_base(self.place_pose, self.base)

    self.pre = self.get_parameter('pre_offset').value
    self.lift_h = self.get_paramter('lift_height').value
    self.T = self.get_paramter('move_time').value
    self.open_v = self.get_paramter('grip_open').value
    self.close_v = self.get_parameter('grip_close').value
    
    self.pick_above = self._offset_z(self.pick, self.pre)
    self.pick_down = self._offset_z(self.pick, 0.0)
    self.pick_lift = self._offset_z(self.pick, self.lift_h)

    self.place_above = self._offset_z(self.place, self.pre)
    self.place_down = self._offset_z(self.place, 0.0)
    self.place_up = self._offset_z(self.place, self.lift_h)

    self._gripper(self.open_v)
    self._move_to_pose(self.pick_above, self.T)
    self._move_to_pose(self.pick_down, self.T)


  def _to_base(self, pose: PoseStamped, base_frame: str):
    # pose: PoseStamped msg
    if pose.header.frame_id == base_frame: return pose

    try:
      return self.listener.transform(pose, base_frame, Duration(seconds = 1.0))
    except:
      self.get_logger().warn('unable to tf')

  def _offset_z(pose, dz):
    pose_cp = pose.copy()
    pose_cp.position.z += dz

    return pose_cp

  # IK
  def _move_to_pose(target_pose, move_time: Duration()):
    q = self._compute_ik(target_pose)

    if q is None: return False

    return _exec_joint_positions(q, move_time)

  def _compute_ik(target_pose):
    if self.joint_state is None: return None

    # Client class 의 _wait_for_service
    if self.ik_srv.wait_for_service() is None:
      return None

    # GetPositionIK srv -> Request()
    self.ik_req = GetPositionIK.Request().ik_request
    self.ik_req.group_name = self.get_parameter('group_name').value
    self.ik_req.ee_link = self.get_parameter('ee_link').value
    self.ik_req.pose_stamped = target_pose
    self.ik_req.timeout = Duration(seconds = 0.5)
    self.ik_req.robot_state.joint_state = self.joint_state
    
    fut = self.ik_srv.call_async(self.ik_req)
    rclpy.spin_until_future_complete(self, fut)
    res = fut.result()

    if res is None or res.error_code != 1:
        return None

    if res.solution.joint_state.name is None or res.solution.joint_state.position is None:
      return self.get_logger().warn('no joints')
    else:
      return res.solution.joint_state.position
      
    map solution.joint_state → joint_names order
        return positions list

  def _exec_joint_positions(positions, move_time):
    #[0] wait_for_server
    if not self.act1.wait_for_server(): return False

    names = self.get_paramter('joint').value

    jt = JointTrajectory()
    jt.joint_names = names
    
    jtp = JointTrajectoryPoint()
    jtp.positions = positions
    jtp.time_from_start = Duration(seconds = move_time)

    jt.points.append(jtp)

    # FollowJointTrajectory: Goal, Result, Feedback sub class
    fjt = FollowJointTrajectory.Goal()
    fjt.trajectory = jt

    #[1] send_goal_async() -> Future 객체 반환
    send_goal = self.act1.send_goal_async(fjt)
    
    #[2] spin_until_future_complete()
    rclpy.spin_until_future_complete(self, send_goal)
    
    #[3] result() -> Future class 함수
    goal = send_goal.result()
    # +) accepted: ClientGoalHandle class 
    if goal is None or res.accepted is False:
      return None
    
    #[1] get_result_async()
    res = goal.get_result_async()
    #[2] spin_until_future_complete()
    rclpy.spin_until_future_complete(self, res)
    #[3] result()
    res = res.result()
    # +) status
    return (res is not None and res.status == 4) or (res is not None and res.status == 0)
        
def _gripper(position, max_effort = 40.0):
  if not self.act2.wait_for_server:
    return None

  # msg
  gr = GripperCommand.Goal()
  gr.position = position
  gr.max_effort = max_effort

  #[1] send_goal_async() -> Future 객체 반환
  send_goal = self.act2.send_goal_async(gr)
  
  #[2] spin_until_future_complete()
  rclpy.spin_until_future_complete(self, send_goal)
  
  #[3] result() -> Future class 함수
  goal = send_goal.result()
  # +) accepted: ClientGoalHandle class 
  if goal is None or res.accepted is False:
    return None
  
  #[1] get_result_async()
  res = goal.get_result_async()
  #[2] spin_until_future_complete()
  rclpy.spin_until_future_complete(self, res)
  #[3] result()
  res = res.result()

  return res is not None
        

# --- main entry ---
def main():
  rclpy.init()
  rclpy.spin(Planner)
  rclpy.shutdown()
