# 09_planning_scene_box.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive

class SceneBox(Node):
    def __init__(self):
        super().__init__('scene_box')
        self.pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.timer = self.create_timer(0.5, self.tick)
        self.toggle = False

    def tick(self):
        ps = PlanningScene()
        ps.is_diff = True
      
        co = CollisionObject()
        co.id = 'obstacle_box'
        co.header.frame_id = 'base_link'
      
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.10, 0.10, 0.10]  # 10cm 큐브
      
        co.primitives.append(box)
      
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'

      
        # 위치를 번갈아가며 바꿔, 경로가 달라지는지 시험
        pose.pose.position.x = 0.45
        pose.pose.position.y = 0.15 if self.toggle else 0.05
        pose.pose.position.z = 0.05
        pose.pose.orientation.w = 1.0
      
        co.primitive_poses.append(pose.pose)
        co.operation = CollisionObject.ADD
      
        ps.world.collision_objects.append(co)
        ps.robot_state.is_diff = True
      
        self.pub.publish(ps)
        self.toggle = not self.toggle
      
        self.get_logger().info('scene diff published')

rclpy.init(); rclpy.spin(SceneBox()); rclpy.shutdown()
