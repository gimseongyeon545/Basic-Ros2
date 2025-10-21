#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

class DepthToPick(Node):
    def __init__(self):
        super().__init__('depth_to_pick')

        # ---- params (런치/CLI로 덮어쓰기 쉬움) ----
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('pick_topic', '/pick_pose')
        self.declare_parameter('place_topic', '/place_pose')
        self.declare_parameter('publish_place', True)
        self.declare_parameter('place_frame_id', 'base_link')
        self.declare_parameter('place_x', 0.30)
        self.declare_parameter('place_y', 0.00)
        self.declare_parameter('place_z', 0.10)
        self.declare_parameter('pixel_u', -1)   # -1이면 화면 중앙 사용
        self.declare_parameter('pixel_v', -1)
        self.declare_parameter('window', 5)     # 중앙 주변 창(홀수): 노이즈 완화

        # ---- pubs/subs ----
        self.pick_pub = self.create_publisher(PoseStamped, self.get_parameter('pick_topic').value, 10)
        self.place_pub = self.create_publisher(PoseStamped, self.get_parameter('place_topic').value, 10)

        self.caminfo: CameraInfo | None = None
        self.sub_info = self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self._on_info, 1)
        self.sub_depth = self.create_subscription(Image, self.get_parameter('depth_topic').value, self._on_depth, 1)

        self.once_done = False
        self.get_logger().info('depth.py ready: waiting for depth + camera_info')

    def _on_info(self, msg: CameraInfo):
        self.caminfo = msg

    def _on_depth(self, msg: Image):
        if self.once_done or self.caminfo is None:
            return

        # ---- pick pixel ----
        w = msg.width; h = msg.height
        u = int(self.get_parameter('pixel_u').value)
        v = int(self.get_parameter('pixel_v').value)
        if u < 0 or v < 0:
            u = w // 2
            v = h // 2

        # ---- depth array 로딩 (16UC1: mm, 32FC1: m) ----
        enc = msg.encoding.lower()
        if enc in ('16uc1', 'mono16'):
            dtype = np.uint16
            depth_m_factor = 0.001  # mm -> m
        elif enc in ('32fc1'):
            dtype = np.float32
            depth_m_factor = 1.0
        else:
            self.get_logger().error(f'unsupported depth encoding: {msg.encoding}')
            return

        arr = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, msg.width))
        win = int(self.get_parameter('window').value)
        if win < 1 or win % 2 == 0:
            win = 5
        hs = win // 2
        u0 = max(0, u - hs); u1 = min(w, u + hs + 1)
        v0 = max(0, v - hs); v1 = min(h, v + hs + 1)
        patch = arr[v0:v1, u0:u1]

        # 유효값 필터 + 중값
        flat = patch.flatten()
        if enc == '16uc1' or enc == 'mono16':
            valid = flat[flat > 0]
        else:
            valid = flat[np.isfinite(flat) & (flat > 0)]
        if valid.size == 0:
            self.get_logger().warn('no valid depth around the pixel')
            return
        depth_m = float(np.median(valid) * depth_m_factor)

        # ---- pinhole back-project (CameraInfo.K) ----
        K = self.caminfo.k  # [fx,0,cx, 0,fy,cy, 0,0,1]
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        X = (u - cx) * depth_m / fx
        Y = (v - cy) * depth_m / fy
        Z = depth_m

        # ---- PoseStamped (카메라 프레임) ----
        pick = PoseStamped()
        pick.header = msg.header  # frame_id / stamp 그대로 사용 (카메라 optical frame)
        pick.pose.position.x = float(X)
        pick.pose.position.y = float(Y)
        pick.pose.position.z = float(Z)
        # 최소화: 자세는 단순 단위 quaternion(필요 시 이후 개선)
        pick.pose.orientation.x = 0.0
        pick.pose.orientation.y = 0.0
        pick.pose.orientation.z = 0.0
        pick.pose.orientation.w = 1.0

        self.pick_pub.publish(pick)
        self.get_logger().info(f'published /pick_pose at camera[{u},{v}] depth={depth_m:.3f} m')

        # ---- 선택: /place_pose 고정 발행 ----
        if bool(self.get_parameter('publish_place').value):
            place = PoseStamped()
            place.header.frame_id = str(self.get_parameter('place_frame_id').value)
            place.header.stamp = msg.header.stamp
            place.pose.position.x = float(self.get_parameter('place_x').value)
            place.pose.position.y = float(self.get_parameter('place_y').value)
            place.pose.position.z = float(self.get_parameter('place_z').value)
            place.pose.orientation.w = 1.0  # 단위 quaternion
            self.place_pub.publish(place)
            self.get_logger().info('published /place_pose (fixed)')

        self.once_done = True
        self.get_logger().info('done (one-shot). You can relaunch to pick again.')

def main():
    rclpy.init()
    node = DepthToPick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
