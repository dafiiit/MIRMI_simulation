#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs 

class AprilTagPosePublisher(Node):
    def __init__(self):
        super().__init__('apriltag_pose_publisher')
        
        self.publisher_ = self.create_publisher(PoseStamped, '/localization/docking_pose', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Wir suchen diesen Frame (wird von apriltag_ros gepublisht)
        self.target_tag_frame = 'tag36_11_00001' 
        self.robot_frame = 'robot/chassis'

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('AprilTag Adapter started.')

    def timer_callback(self):
        try:
            # Lookup Transform: Wo ist der Tag aus Sicht des Roboters?
            # Timeout 0 -> nimm den letzten verfügbaren
            t = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.target_tag_frame,
                rclpy.time.Time())
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.robot_frame
            
            pose_msg.pose.position.x = t.transform.translation.x
            pose_msg.pose.position.y = t.transform.translation.y
            pose_msg.pose.position.z = t.transform.translation.z
            pose_msg.pose.orientation = t.transform.rotation
            
            self.publisher_.publish(pose_msg)
            
        except TransformException:
            # Tag gerade nicht sichtbar
            pass

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
