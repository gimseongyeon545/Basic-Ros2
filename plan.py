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

from rclpy.executors import MultiThreadedExecutor
import copy
import threading  # ★ 추가

def quat_from_rpy(roll, pitch, yaw):
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
        self.declare_parameter('gripper_action', '/gen3_lite_2f_gripper_controller/gripper_cmd')
        self.declare_parameter('joint_names', ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'])

        # 접근/리프트/그리퍼 값(장비에 맞게 조정)
        self.declare_parameter('pre_offset', 0.05)    # 집기 전 위로 접근
        self.declare_parameter('lift_height', 0.10)   # 집은 뒤 들어올림
        self.declare_parameter('grip_open', 0.20)     # 장비에 맞게 조정 필요
        self.declare_parameter('grip_close', 0.4)    # 장비에 맞게 조정 필요
        self.declare_parameter('move_time', 10.0)      # 각 세그먼트 이동 시간(초)

        # ---------- 상태 ----------
        self.joint_state: Optional[JointState] = None

        # ---------- 통신 ----------
        self.js_sub = self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        self.ik_cli = self.create_client(GetPositionIK, self.get_parameter('ik_service').value)

        self.arm_ac = ActionClient(self, FollowJointTrajectory, self.get_parameter('traj_action').value)
        self.grip_ac = ActionClient(self, GripperCommand, self.get_parameter('gripper_action').value)

        self.tf_buf = Buffer()
        self.tf_lis = TransformListener(self.tf_buf, self, spin_thread=True)

        self._ran_once = False  # 한 번만 실행
        self._running = False

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
        self.get_logger().info('got pick')
        self.pick_pose = msg
        self._try_run()

    def _on_place(self, msg: PoseStamped):
        self.place_pose = msg
        self.get_logger().info('got place')
        self._try_run()

    # ----------------- 메인 시퀀스 -----------------
    def _try_run(self):
        self.get_logger().info('try_run: entered')
        if self.pick_pose is None or self.place_pose is None:
            self.get_logger().info('try_run: missing pick/place -> return')
            return
        if self._ran_once:
            self.get_logger().info('try_run: already ran once -> return')
            return
        if self._running:
            self.get_logger().info('try_run: already running -> return')
            return
        self._ran_once = True
        self._running = True
        threading.Thread(target=self._run_sequence, daemon=True).start()

    def _run_sequence(self):
        try:
            base = self.get_parameter('base_frame').value
            pre  = float(self.get_parameter('pre_offset').value)
            lift = float(self.get_parameter('lift_height').value)
            T    = float(self.get_parameter('move_time').value)
            open_v  = float(self.get_parameter('grip_open').value)
            close_v = float(self.get_parameter('grip_close').value)

            # pick/place를 지역 복사(콜백 갱신과 독립)
            pick  = self._to_base(copy.deepcopy(self.pick_pose),  base);  self.get_logger().info('try_run: pick -> base OK')
            place = self._to_base(copy.deepcopy(self.place_pose), base);  self.get_logger().info('try_run: place -> base OK')

            pick_above  = self._offset_z(pick,  pre)
            pick_down   = self._offset_z(pick,  0.0)
            pick_lift   = self._offset_z(pick,  lift)
            place_above = self._offset_z(place, pre)
            place_down  = self._offset_z(place, 0.0)
            place_up    = self._offset_z(place, lift)

            ok = True
            ok &= self._gripper(open_v);                 self.get_logger().info('seq: gripper open done')
            ok &= self._move_to_pose(pick_above, T);     self.get_logger().info('seq: move pick_above done')
            ok &= self._move_to_pose(pick_down,  T);     self.get_logger().info('seq: move pick_down done')
            ok &= self._gripper(close_v);                self.get_logger().info('seq: gripper close done')
            ok &= self._move_to_pose(pick_lift,  T);     self.get_logger().info('seq: move pick_lift done')
            ok &= self._move_to_pose(place_above, T);    self.get_logger().info('seq: move place_above done')
            ok &= self._move_to_pose(place_down,  T);    self.get_logger().info('seq: move place_down done')
            ok &= self._gripper(open_v);                 self.get_logger().info('seq: gripper open (place) done')
            ok &= self._move_to_pose(place_up,    T);    self.get_logger().info('seq: move place_up done')

            self.get_logger().info(f'seq: done ok={ok}')
        finally:
            self._running = False

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
        out.pose = copy.deepcopy(pose.pose)
        out.pose.position.z += dz
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

        if not self.ik_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('compute_ik service not available')
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.get_parameter('group_name').value
        req.ik_request.ik_link_name = self.get_parameter('ee_link').value
        req.ik_request.pose_stamped = target
        req.ik_request.timeout = RosDuration(sec=1, nanosec=0)
        req.ik_request.robot_state.joint_state = self.joint_state

        fut = self.ik_cli.call_async(req)

        # ★ 워커 스레드에서 spin 금지 → Event로 대기
        evt = threading.Event()
        fut.add_done_callback(lambda _: evt.set())
        if not evt.wait(timeout=10.0):
            self.get_logger().error('IK call timeout')
            return None

        res = fut.result()
        if res is None or res.error_code.val != 1:  # SUCCESS=1
            self.get_logger().warn(f'IK failed: res={res}')
            return None

        names = self.get_parameter('joint_names').value
        name_to_pos = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
        try:
            q = [float(name_to_pos[n]) for n in names]
            self.get_logger().info(f'IK OK -> joints: {q}')
            return q
        except KeyError as e:
            self.get_logger().error(f'IK result missing joint {e}, have={res.solution.joint_state.name}')
            return None

    def _exec_joint_positions(self, positions: list, move_time: float) -> bool:
        if not self.arm_ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('FollowJointTrajectory action not available')
            return False

        names = self.get_parameter('joint_names').value
        self.get_logger().info(f'Sending JTC goal: names={names}, pos={positions}, T={move_time:.2f}s')

        traj = JointTrajectory()
        traj.joint_names = names

        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = RosDuration(sec=int(move_time),
                                         nanosec=int((move_time % 1.0)*1e9))
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        send_fut = self.arm_ac.send_goal_async(goal)

        # ★ Event 대기
        evt_goal = threading.Event()
        send_fut.add_done_callback(lambda _: evt_goal.set())
        if not evt_goal.wait(timeout=10.0):
            self.get_logger().warn('JTC goal send timeout')
            return False

        gh = send_fut.result()
        if gh is None:
            self.get_logger().warn('Trajectory goal handle is None (no response)')
            return False
        self.get_logger().info(f'Goal accepted? {getattr(gh, "accepted", False)}')

        if not getattr(gh, 'accepted', False):
            self.get_logger().warn('Trajectory goal rejected by controller')
            return False

        res_fut = gh.get_result_async()
        evt_res = threading.Event()
        res_fut.add_done_callback(lambda _: evt_res.set())
        if not evt_res.wait(timeout=move_time + 10.0):
            self.get_logger().warn('JTC result timeout')
            return False

        res = res_fut.result()
        self.get_logger().info(f'JTC result status={res.status}, result={res.result}')
        return res.status in (4, 0)  # 4=SUCCEEDED, 일부 구현 0 반환

    # ----------------- 그리퍼 -----------------
    def _gripper(self, position: float, max_effort: float = 10.0) -> bool:
        if not self.grip_ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Gripper action not available')
            return False

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)

        send_fut = self.grip_ac.send_goal_async(goal)

        # ★ Event 대기
        evt_goal = threading.Event()
        send_fut.add_done_callback(lambda _: evt_goal.set())
        if not evt_goal.wait(timeout=10.0):
            self.get_logger().warn('Gripper goal send timeout')
            return False

        gh = send_fut.result()
        if gh is None or not getattr(gh, 'accepted', False):
            self.get_logger().warn('Gripper goal rejected')
            return False

        res_fut = gh.get_result_async()
        evt_res = threading.Event()
        res_fut.add_done_callback(lambda _: evt_res.set())
        if not evt_res.wait(timeout=5.0):
            self.get_logger().warn('Gripper result timeout')
            return False

        res = res_fut.result()
        self.get_logger().info(f'Gripper result: {res.result}')
        return True


def main():
    rclpy.init()
    node = PickPlaceMin()
    exec = MultiThreadedExecutor()  # ★ 멀티스레드 실행자
    exec.add_node(node)
    try:
        exec.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
