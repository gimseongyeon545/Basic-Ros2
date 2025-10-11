'''
[adder_client] --call--> (AddTwoInts.srv) --serve--> [adder_service]
      ▲                                         │
      └────────────── result <──────────────────┘
'''

# adder_client.py

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class adder_client(Node):
  def __init__(self, a, b):
    super().__init__(self):
    self.cli = self.create_client(AddTwoInts, 'adder', 10)

    # [0] service 호출 waiting
    if self.cli.wait_for_service(1.0): # 1초 안에 server 뜨면
      self.get_logger().info('service came')
    else:
      self.get_logger().warn('no service')
    
    # [1] 사용할 srv 의 request 채우기
    self.req = AddTwoInts.Request()
    self.req.a = a
    self.req.b = b
    
    # [2] call_async(): req 전송하기
    self.future = self.cli.call_async(self.req) # Future class 반환

    # [3] 방법1) 동기식: spin_until_future_complete -> future.result()
    rclpy.spin_until_future_complete(self, self.future)
    self.res = self.future.result()
    self.get_logger().info(f'res: {self.res}')
    return self.res
    
    #(# [3] 방법2) 비동기식: spin -> future.add_done_callback() or future.done())


rclpy.init()
rclpy.spin(adder_client())
rclpy.shutdown()
