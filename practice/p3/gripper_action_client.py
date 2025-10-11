import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient # action client = Goal, Feedback, Result(cancel) / service client = request, response
from control_msgs.action import GripperCommand 
# https://github.com/ros-controls/control_msgs/blob/master/control_msgs/action/GripperCommand.action

class gripper_action_client(Node):
  def __init__(self):
    super().__init__('gripper_action_client')
    self.ac = ActionClient(self, GripperCommand, 'gripperac')
    
    # [0] server 호출 waiting: wait_for_server
    if self.ac.wait_for_server(1.0):
       self.get_logger().info('get server')
    else:
       self.get_logger().warn('no server')

    #========[Goal]=========
    # [1] Goal 생성 후 field 채우기
    self.goal = GripperCommand.Goal()
    self.goal.command.position = 0.2 # gap
    self.goal.command.effort = 40.0

    # [2] send_goal_async(): Goal 전달 -> future 반환
    self.goal_future = self.ac.send_goal_async(self.goal) # future class (Future[ClientGoalHandle])

    # [3] spin_until_future_complete() -> future.result() + accepted
    rclpy.spin_until_future_complete(self.goal_future)
    self.goal_handle = self.goal_future.result() # T (generic) -> ClientGoalHandle 객체 반환 

    if self.goal_handle.accepted:
      self.get_logger().info('goal accepted')
    else:
      self.get_logger().warn('goal not accepted')

    
    #========[Result]========= 
    # [1] get_result_async()
    self.result_future = self.goal_handle.get_result_async() # future 객체 반환 (Future[GetResultServiceResponse[ResultT]])
    
    # [2] spin_until_future_complete() -> future.result() -> .status, .result
    rclpy.spin_until_future_complete()
    self.res = self.result_future.result() # T (generic) -> GetResultServiceResponse

    # GetResultServiceResponse -> status, result field
    # https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/type_support.py#L100
    self.res_status = self.res.status
    self.res_result = self.res.result


    
