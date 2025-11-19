#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

class DepthCameraInfoSync(Node):
    def __init__(self):
        super().__init__('depth_camera_info_sync')
        
        # Subscribe to the depth image
        self.subscription = self.create_subscription(
            Image,
            '/depth/image_raw',
            self.image_callback,
            10)
        
        # Publish the camera info
        self.publisher_ = self.create_publisher(CameraInfo, '/depth/camera_info', 10)
        
        self.get_logger().info('Depth CameraInfo Sync Node started.')

        # Static Camera Info (matches Gazebo model: 640x480, FOV 1.047)
        self.cam_info = CameraInfo()
        self.cam_info.width = 640
        self.cam_info.height = 480
        self.cam_info.distortion_model = 'plumb_bob'
        self.cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Focal length fx = fy = width / (2 * tan(fov/2))
        # 640 / (2 * tan(30 deg)) ~= 554.256
        f = 554.256
        cx = 320.5
        cy = 240.5
        
        self.cam_info.k = [f, 0.0, cx,
                           0.0, f, cy,
                           0.0, 0.0, 1.0]
        self.cam_info.r = [1.0, 0.0, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 1.0]
        self.cam_info.p = [f, 0.0, cx, 0.0,
                           0.0, f, cy, 0.0,
                           0.0, 0.0, 1.0, 0.0]

    def image_callback(self, img_msg):
        # Copy header from image to ensure exact synchronization
        info_msg = self.cam_info
        info_msg.header = img_msg.header
        self.publisher_.publish(info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthCameraInfoSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
