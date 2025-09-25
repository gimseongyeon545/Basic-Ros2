# 02_param_goal_publisher.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.declare_parameters('', [
            ('target_xyz', [0.40, 0.20, 0.12]),
            ('target_rpy', [0.0, math.pi, 0.0]),  # 수직 하향
            ('frame_id', 'base_link')
        ])
        self.pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.timer = self.create_timer(1.0, self.publish_once)

    def publish_once(self):
        xyz = self.get_parameter('target_xyz').value
        rpy = self.get_parameter('target_rpy').value
        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter('frame_id').value
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
        # 간단: RPY→quat 대충(roll=0, pitch=π, yaw=0)
        import tf_transformations as tft
        q = tft.quaternion_from_euler(*rpy)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        self.pub.publish(pose)
        self.get_logger().info('target published')

rclpy.init(); rclpy.spin(GoalPublisher()); rclpy.shutdown()
