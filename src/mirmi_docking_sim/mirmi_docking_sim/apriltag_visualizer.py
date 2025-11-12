#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class AprilTagVisualizer(Node):
    def __init__(self):
        super().__init__('apriltag_visualizer')
        
        self.bridge = CvBridge()
        self.latest_detections = None
        self.latest_image = None
        
        # Subscribe to raw image and detections
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )
        
        # Publisher for annotated image
        self.annotated_pub = self.create_publisher(
            Image,
            '/camera/tag_detections_image',
            10
        )
        
        self.get_logger().info('AprilTag Visualizer gestartet')
    
    def detection_callback(self, msg):
        """Store latest detections"""
        self.latest_detections = msg
    
    def image_callback(self, msg):
        """Process image and draw detections"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Draw detections if available
            if self.latest_detections is not None and len(self.latest_detections.detections) > 0:
                for detection in self.latest_detections.detections:
                    # Get corners
                    corners = detection.corners
                    if len(corners) == 4:
                        # Convert corners to numpy array for drawing
                        pts = np.array([
                            [int(corners[0].x), int(corners[0].y)],
                            [int(corners[1].x), int(corners[1].y)],
                            [int(corners[2].x), int(corners[2].y)],
                            [int(corners[3].x), int(corners[3].y)]
                        ], dtype=np.int32)
                        
                        # Calculate center from corners
                        center_x = int(np.mean([c.x for c in corners]))
                        center_y = int(np.mean([c.y for c in corners]))
                        
                        # Draw the tag outline in green
                        cv2.polylines(cv_image, [pts], True, (0, 255, 0), 3)
                        
                        # Draw corner circles with different colors
                        corner_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]
                        for i, corner in enumerate(corners):
                            cv2.circle(cv_image, 
                                     (int(corner.x), int(corner.y)), 
                                     6, 
                                     corner_colors[i], 
                                     -1)
                        
                        # Draw center point in cyan
                        cv2.circle(cv_image, (center_x, center_y), 8, (0, 255, 255), -1)
                        
                        # Draw ID text
                        tag_id = detection.id
                        cv2.putText(
                            cv_image,
                            f'ID: {tag_id}',
                            (center_x - 30, center_y - 15),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 255, 255),
                            2
                        )
                        
                        # Draw distance if pose is available
                        try:
                            # Try to access pose information
                            if hasattr(detection, 'pose') and detection.pose.pose.pose.position.z > 0:
                                pos = detection.pose.pose.pose.position
                                distance = np.sqrt(pos.x**2 + pos.y**2 + pos.z**2)
                                cv2.putText(
                                    cv_image,
                                    f'Dist: {distance:.2f}m',
                                    (center_x - 30, center_y + 25),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6,
                                    (0, 255, 255),
                                    2
                                )
                                
                                # Draw position coordinates
                                cv2.putText(
                                    cv_image,
                                    f'X:{pos.x:.2f} Y:{pos.y:.2f} Z:{pos.z:.2f}',
                                    (center_x - 60, center_y + 45),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.4,
                                    (255, 255, 255),
                                    1
                                )
                        except Exception as e:
                            # If pose is not available, just skip it
                            self.get_logger().debug(f'Pose nicht verfügbar: {str(e)}')
                            pass
                
                # Add detection count to image
                cv2.putText(
                    cv_image,
                    f'Tags detected: {len(self.latest_detections.detections)}',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )
            else:
                # No detections
                cv2.putText(
                    cv_image,
                    'No tags detected',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )
            
            # Convert back to ROS Image and publish
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f'Fehler beim Visualisieren: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
