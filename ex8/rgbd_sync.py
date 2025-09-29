# 08_rgbd_sync.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer

class RGBDSync(Node):
    def __init__(self):
        super().__init__('rgbd_sync')
        self.sub_c  = Subscriber(self, Image, '/color/image_raw')
        self.sub_d  = Subscriber(self, Image, '/aligned_depth_to_color/image_raw')
        self.sub_ci = Subscriber(self, CameraInfo, '/color/camera_info')
      
        self.sync = ApproximateTimeSynchronizer([self.sub_c, self.sub_d, self.sub_ci], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.cb)

    def cb(self, color, depth, caminfo):
        self.get_logger().info(f'sync OK t={color.header.stamp.sec}.{color.header.stamp.nanosec:09d}')

rclpy.init(); rclpy.spin(RGBDSync()); rclpy.shutdown()
