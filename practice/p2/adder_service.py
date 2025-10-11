'''
[adder_client] --call--> (AddTwoInts.srv) --serve--> [adder_service]
      ▲                                         │
      └────────────── result <──────────────────┘
'''

# adder_service.py

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Request: a, b -> Response: sum

class adder_service(Node):
  def __init__(self):
    super().__init__('adder_service')
    self.srv = self.create_service(AddTwoInts, 'adder', self.cb, 10) # service: Request, Response msg 를 둘다 보냄

  def cb(self, request, response):
    out = request.a + request.b
    self.get_logger().info(f'adder service request: {out}')

rclpy.init()
rclpy.spin(adder_service())
rclpy.shutdown()
