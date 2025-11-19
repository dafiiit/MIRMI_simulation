#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import numpy as np

class PCLDockingController(Node):
    def __init__(self):
        super().__init__('pcl_docking_controller')
        
        # Subscriber für die generierte Point Cloud
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/depth/points',
            self.pc_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('PCL Docking Controller gestartet. Warte auf PointCloud...')

    def pc_callback(self, msg):
        # Hier kommt später deine ICP / Registration Logik hin
        # Für jetzt: Einfach mal loggen, dass wir Daten haben
        
        width = msg.width
        height = msg.height
        data_len = len(msg.data)
        
        self.get_logger().info(
            f'PointCloud empfangen! Größe: {width}x{height}, Bytes: {data_len}', 
            throttle_duration_sec=2.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = PCLDockingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
