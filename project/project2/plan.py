#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration as RosDuration

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


def quat_from_rpy(roll, pitch, yaw):
    # 최소 구현용 간단 변환
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return (qx, qy, qz, qw)


class PickPlaceMin(Node):
    def __init__(self):
        super().__init__('pick_place_min')

        # ---------- 파라미터(필요시 런치에서 override) ----------
        self.declare_parameter('group_name', 'arm')
        self.declare_parameter('ee_link', 'end_effector_link')  # gen3 lite config에 맞춰 수정
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ik_service', '/compute_ik')
        self.declare_parameter('traj_action', '/joint_trajectory_controller/follow_joint_trajectory')
        self.declare_parameter('gripper_action', '/gripper_controller/gripper_cmd')
        self.declare_parameter('joint_names', ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'])

        # 접근/리프트/그리퍼 값(장비에 맞게 조정)
        self.declare_parameter('pre_offset', 0.05)    # 집기 전 위로 접근
        self.declare_parameter('lift_height', 0.10)   # 집은 뒤 들어올림
        self.declare_parameter('grip_open', 0.80)     # 장비에 맞게 조정 필요
        self.declare_parameter('grip_close', 0.00)    # 장비에 맞게 조정 필요
        self.declare_parameter('move_time', 3.0)      # 각 세그먼트 이동 시간(초)

        # ---------- 상태 ----------
        self.joint_state: Optional[JointState] = None

        # ---------- 통신 ----------
        self.js_sub = self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        self.ik_cli = self.create_client(GetPositionIK, self.get_parameter('ik_service').value)

        self.arm_ac = ActionClient(self, FollowJointTrajectory, self.get_parameter('traj_action').value)
        self.grip_ac = ActionClient(self, GripperCommand, self.get_parameter('gripper_action').value)

        self.tf_buf = Buffer()
        self.tf_lis = TransformListener(self.tf_buf, self, spin_thread=True)

        # ---------- 목표 Pose 수신(간단히 두 토픽) ----------
        self.pick_pose: Optional[PoseStamped] = None
        self.place_pose: Optional[PoseStamped] = None
        self.create_subscription(PoseStamped, '/pick_pose', self._on_pick, 10)
        self.create_subscription(PoseStamped, '/place_pose', self._on_place, 10)

        self.get_logger().info('Ready. Publish PoseStamped to /pick_pose and /place_pose to start.')

    # ----------------- 콜백 -----------------
    def _js_cb(self, msg: JointState):
        self.joint_state = msg

    def _on_pick(self, msg: PoseStamped):
        self.pick_pose = msg
        self._try_run()

    def _on_place(self, msg: PoseStamped):
        self.place_pose = msg
        self._try_run()

    # ----------------- 메인 시퀀스 -----------------
    def _try_run(self):
        if self.pick_pose is None or self.place_pose is None:
            return
        # 이미 한 번 실행했다면 중복 방지하려면 여기에서 return 처리 가능

        # 1) 입력 Pose들을 base_frame으로 정규화
        base = self.get_parameter('base_frame').value
        pick = self._to_base(self.pick_pose, base)
        place = self._to_base(self.place_pose, base)

        pre = float(self.get_parameter('pre_offset').value)
        lift_h = float(self.get_parameter('lift_height').value)
        T = float(self.get_parameter('move_time').value)
        open_v = float(self.get_parameter('grip_open').value)
        close_v = float(self.get_parameter('grip_close').value)

        # 파생 포즈들
        pick_above = self._offset_z(pick, pre)
        pick_down  = self._offset_z(pick, 0.0)
        pick_lift  = self._offset_z(pick, lift_h)

        place_above = self._offset_z(place, pre)
        place_down  = self._offset_z(place, 0.0)
        place_up    = self._offset_z(place, lift_h)

    # ----------------- 유틸: TF/포즈 -----------------
    def _to_base(self, pose: PoseStamped, base_frame: str) -> PoseStamped:
        if pose.header.frame_id == base_frame:
            return pose
        try:
            return self.tf_buf.transform(pose, base_frame, timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f'tf to {base_frame} failed: {e}. Using as-is.')
            return pose

    def _offset_z(self, pose: PoseStamped, dz: float) -> PoseStamped:
        out = PoseStamped()
        out.header.frame_id = pose.header.frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose = pose.pose
        out.pose.position.z = pose.pose.position.z + dz
        return out

    # ----------------- IK + 실행 -----------------
    def _move_to_pose(self, target: PoseStamped, move_time: float) -> bool:
        ik_sol = self._compute_ik(target)
        if ik_sol is None:
            self.get_logger().warn('IK failed.')
            return False
        return self._exec_joint_positions(ik_sol, move_time)

    def _compute_ik(self, target: PoseStamped) -> Optional[list]:
        if self.joint_state is None:
            self.get_logger().warn('No joint_states yet.')
            return None

        if not self.ik_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('compute_ik service not available')
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.get_parameter('group_name').value
        req.ik_request.ik_link_name = self.get_parameter('ee_link').value
        req.ik_request.pose_stamped = target
        req.ik_request.timeout = RosDuration(sec=1, nanosec=0)
        # 현재 상태를 seed로
        req.ik_request.robot_state.joint_state = self.joint_state

        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res is None or res.error_code.val != 1:  # SUCCESS=1
            return None

        # 결과 joint_state에서 그룹 조인트만 추출(여기선 파라미터 joint_names 순서 사용)
        names = self.get_parameter('joint_names').value
        name_to_pos = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        try:
            return [float(name_to_pos[n]) for n in names]
        except KeyError:
            self.get_logger().error('IK result missing some joints.')
            return None

    def _exec_joint_positions(self, positions: list, move_time: float) -> bool:
        if not self.arm_ac.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('FollowJointTrajectory action not available')
            return False

        names = self.get_parameter('joint_names').value

        traj = JointTrajectory()
        traj.joint_names = names

        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = RosDuration(sec=int(move_time), nanosec=int((move_time % 1.0)*1e9))
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        send_fut = self.arm_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn('Trajectory goal rejected')
            return False

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=move_time+5.0)
        res = res_fut.result()
        return (res is not None and res.status == 4) or (res is not None and res.status == 0)
        # status 4=SUCCEEDED (control_msgs/FollowJointTrajectoryResult) 구현체마다 0일 수도 있음

    # ----------------- 그리퍼 -----------------
    def _gripper(self, position: float, max_effort: float = 40.0) -> bool:
        if not self.grip_ac.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Gripper action not available')
            return False

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)

        send_fut = self.grip_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn('Gripper goal rejected')
            return False

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=5.0)
        res = res_fut.result()
        return res is not None  # 최소 확인만 (stalled/reached_goal 필요시 res.result.* 확인)


def main():
    rclpy.init()
    node = PickPlaceMin()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
