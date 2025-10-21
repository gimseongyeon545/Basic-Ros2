import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
trajectory_msgs/JointTrajectory
from control_msgs.action import FollowJointTrajectory


class traj_action_client(Node):
  def __init__(self):
    super().__init__(self)
    self.ac = ActionClient(self, FollowJointTrajectory, 'trajaction')

  # [0] server 호출 waiting: wait_for_server
  #========[Goal]=========
  # [1] Goal 생성 후 field 채우기

  # [2] send_goal_async(): Goal 전달 -> future 반환
  # feedback 함수: goal 보내고 server 가 feedback 보낼 때 호출됨
  
  # [3] spin_until_future_complete() -> future.result() + accepted
  
  #========[Cancel]========= 
  # 실행 중인 goal cancel
  # [1] cancel_goal_async
  
  #========[Result]========= 
  # [1] get_result_async()
  # [2] spin_until_future_complete() -> future.result() -> .status, .result
  # GetResultServiceResponse -> status, result field
  # https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/type_support.py#L100
    
  #========[Feedback]========= 
  # 중간마다 보내는 상태 업데이트 / goal accepted 후 최종 result 나오기 전까지 필요할 때마다 도착
