# 07_depth_pixel_to_point.py
import rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class Depth3D(Node):
    def __init__(self):
        super().__init__('depth3d')
        self.bridge = CvBridge()
        self.K = None
        
        self.create_subscription(CameraInfo, '/color/camera_info', self.caminfo, 1)
        self.create_subscription(Image, '/aligned_depth_to_color/image_raw', self.depth_cb, 10)

    def caminfo(self, msg):
        self.K = np.array(msg.k).reshape(3,3)

    def depth_cb(self, msg):
        if self.K is None: return
        
        depth = self.bridge.imgmsg_to_cv2(msg)  # mm or m: RealSense는 보통 mm
        
        v, u = depth.shape[0]//2, depth.shape[1]//2  # 중앙 픽셀 예시
        z = depth[v, u] / 1000.0                     # meter
        
        if z <= 0: return
        
        fx, fy, cx, cy = self.K[0,0], self.K[1,1], self.K[0,2], self.K[1,2]
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        self.get_logger().info(f'center 3D = ({x:.3f}, {y:.3f}, {z:.3f}) m')

rclpy.init(); rclpy.spin(Depth3D()); rclpy.shutdown()
