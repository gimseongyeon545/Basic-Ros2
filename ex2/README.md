# param_goal_publisher.py
1. `from geometry_msgs.msg import PoseStamped`
2. `declare_parameters()`
   - rclpy.node.Node 의 `declare_parameters(namespace, parameters, ignore_override=False)`
     > parameters (List[Union[Tuple[str], Tuple[str, Any], Tuple[str, Any, ParameterDescriptor]]]) – List of tuples with parameters to declare.
     - https://docs.ros2.org/foxy/api/rclpy/api/node.html#rclpy.node.Node.declare_parameters
   -
