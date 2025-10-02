# rclpy core
import rclpy
from rclpy.node import Node

# msgs
from geometry_msgs.msg import PoseStamped

# utils (RPY→quat)
# pip/apt로 제공되는 tf_transformations 또는 equivalent 사용
from tf_transformations import quaternion_from_euler


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_pub')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('target_xyz', [0, 0, 0])
        self.declare_parameter('target_rpy', [1.75, 0, -1.75])

        self.pub = self.create_publisher('PoseStamped', '/target_pose')

        self.timer = self.create_timer(1.0, self.publish_once)

    def publish_once(self):
        id = self.get_paramaeter('frame_id')
        x, y, z = self.get_parameter('target_xyz')
        r, p, y = self.get_parameter('target_rpy')

        pose = PoseStamped()
        pose.header.stamp = self.timer
        pose.header.frame_id = id
        
        qx, qy, qz, qw = self.quaternion_from_euler(r, p, y)

        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw

        self.pub.publish(pose)

        now = self.get_clock()
        
        self.get_logger().info(f'time:{now}')
    

def main():
    rclpy.init()
    rclpy.spin(GoalPublisher)
    rclpy.shutdown()
