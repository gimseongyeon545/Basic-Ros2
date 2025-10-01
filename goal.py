# rclpy core
import rclpy
from rclpy.node import Node

# msgs
from geometry_msgs.msg import PoseStamped

# utils (RPY→quat)
# pip/apt로 제공되는 tf_transformations 또는 equivalent 사용
from tf_transformations import quaternion_from_euler


class GoalPublisher(Node):
    init:
        super().__init__('goal_pub')
        declare parameters: frame_id, target_xyz, target_rpy
        create publisher: PoseStamped -> /target_pose
        create timer(period=1.0, callback=publish_once)

    publish_once():
        read params -> xyz, rpy
        build PoseStamped with header.stamp = now, header.frame_id = frame_id
        convert rpy -> quaternion
        set pose.position & pose.orientation
        publish(/target_pose)
        (optional) log a short line

main():
    rclpy.init
    spin(GoalPublisher)
    shutdown
