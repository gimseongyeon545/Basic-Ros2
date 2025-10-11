'''
[gripper_action_client] ==goal==> (GripperCommand.action server)
       ▲                 <== feedback/result ==
       └==================== cancel =====================
'''

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
    
    
    # [1] Goal 생성 후 field 채우기
    self.goal = GripperCommand.Goal()
    self.goal.command.position = 0.2 # gap
    self.goal.command.effort = 40.0

    # [1] 

    
    
