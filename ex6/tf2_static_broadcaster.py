# 06_tf2_static_broadcaster.py
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class StaticTF(Node):
    def __init__(self):
        super().__init__('static_tf')
        self.br = StaticTransformBroadcaster(self)
      
        t = TransformStamped()
        t.header.frame_id = 'base_link'
        t.child_frame_id  = 'camera_link'
        
        now = self.get_clock().now().to_msg()
        t.header.stamp = now
        
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (0.30, 0.0, 0.50)
        
        import tf_transformations as tft
        q = tft.quaternion_from_euler(0.0, math.radians(90), 0.0)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
      
        self.br.sendTransform(t)
        self.get_logger().info('static TF published')

rclpy.init(); rclpy.spin(StaticTF()); rclpy.shutdown()
