#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class OdomToTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_to_tf_publisher')
        
        self.declare_parameter('use_ground_truth', False)
        use_gt = self.get_parameter('use_ground_truth').value
        
        # Topic Auswahl
        topic_name = '/ground_truth' if use_gt else '/odom'
        
        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # QoS Profile für Gazebo-Kompatibilität
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            topic_name, # Hier die Variable nutzen
            self.odom_callback,
            qos_profile
        )
        
        self.get_logger().info('Odometry to TF Publisher gestartet - subscribed to {topic_name}')
        
        # Timer für Debug-Ausgaben
        self.debug_timer = self.create_timer(2.0, self.debug_callback)
        self.msg_count = 0
    
    def debug_callback(self):
        """Debug-Ausgabe"""
        if self.msg_count > 0:
            self.get_logger().info(f'TF Transforms publiziert: {self.msg_count}')
        else:
            self.get_logger().warn('Noch keine Odometry-Nachrichten empfangen!')
    
    def odom_callback(self, msg: Odometry):
        """Konvertiert Odometry in TF Transform"""
        self.msg_count += 1
        
        t = TransformStamped()
        
        # Header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'robot/chassis'
        
        # Translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Rotation
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        
        # Publish transform
        self.tf_broadcaster.sendTransform(t)
        
        # Debug bei erster Nachricht
        if self.msg_count == 1:
            self.get_logger().info(f'✓ Erste Odometry empfangen: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')
            self.get_logger().info('✓ Publishing TF: world → robot/chassis')

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
