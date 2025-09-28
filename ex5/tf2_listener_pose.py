# 05_tf2_listener_pose.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # geometry_msgs PoseStamped 변환 헬퍼
from rclpy.duration import Duration

class PoseXform(Node):
    def __init__(self):
        super().__init__('pose_xform')
        self.buf = Buffer(); self.tl = TransformListener(self.buf, self)
        self.sub = self.create_subscription(PoseStamped, '/pose_in_camera', self.cb, 10) # 환경에 맞게
        self.pub = self.create_publisher(PoseStamped, '/pose_in_base', 10) # 환경에 맞게

    def cb(self, msg):
        try:
            out = self.buf.transform(msg, 'base_link', timeout=Duration(seconds=0.2))
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f'tf failed: {e}')

rclpy.init(); rclpy.spin(PoseXform()); rclpy.shutdown()
