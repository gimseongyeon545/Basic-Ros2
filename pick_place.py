import sys, copy
import rclpy
from rclpy.node import Node
import moveit_commander
from geometry_msgs.msg import PoseStamped


노드 시작
MoveIt 초기화
arm  = 그룹("arm")
grip = 그룹("gripper")

함수 go_to_pose(Pose ps):
    arm.set_pose_target(ps)
    arm.go(wait=True)
    arm.stop()
    arm.clear_pose_targets()

함수 set_gripper(float pos):
    q = grip.get_current_joint_values()
    q[0] = pos
    grip.go(q, wait=True)
    grip.stop()

# ---- 목표 정의 (base_link 기준 예시) ----
pre_grasp = Pose(x=0.35, y=0.00, z=0.20, w=1.0)
grasp     = 복사(pre_grasp); grasp.z = 0.12
place     = Pose(x=-0.35, y=0.00, z=0.25, w=1.0)

# ---- 시퀀스 ----
set_gripper(0.80)          # open
go_to_pose(pre_grasp)      # 물체 위
go_to_pose(grasp)          # 물체까지 내려감
set_gripper(0.00)          # close(집기)
lift = 복사(grasp); lift.z += 0.10
go_to_pose(lift)           # 들어올리기
go_to_pose(place)          # 놓을 위치로 이동
set_gripper(0.80)          # open(놓기)

종료(Shutdown)
