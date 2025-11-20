#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import numpy as np
from sensor_msgs_py import point_cloud2
import math
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R

class PCLPosePublisher(Node):
    def __init__(self):
        super().__init__('pcl_pose_publisher')
        
        self.subscription = self.create_subscription(
            PointCloud2, '/depth/points', self.pc_callback, qos_profile_sensor_data)
            
        self.publisher_ = self.create_publisher(PoseStamped, '/localization/docking_pose', 10)
        
        # Referenz-Wolke der Hütte generieren
        self.reference_cloud = self.generate_reference_cloud()
        self.get_logger().info('PCL Localization Node started.')

    def generate_reference_cloud(self):
        # ... (Gleiche Generierungslogik wie in deinem alten Skript) ...
        # Hier dummy implementation der Punkte:
        points = []
        # Einfache Box Punkte (Back Wall)
        for z in np.linspace(0, 1.5, 15):
            for y in np.linspace(-1.0, 1.0, 20):
                points.append([-1.45, y, z])
        return np.array(points, dtype=np.float32)

    def pc_callback(self, msg):
        # 1. Pointcloud in Numpy wandeln
        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        scene_points = np.array(list(gen))
        if len(scene_points) < 100: return

        # 2. Filter (Z > 0.5 && Z < 5.0 etc...)
        # ... (Preprocessing Logik hier) ...

        # 3. ICP durchführen (Referenz vs Szene)
        # Wir suchen T, das Scene -> Reference mappt.
        # Das Ergebnis T ist Pose der Kamera im Hütten-Frame (Inverse der Hütte in Kamera).
        # Für Einfachheit: Wir nehmen an, wir bekommen T_cam_hut (Hütte in Cam Frame).
        
        # Placeholder für ICP Ergebnis (Hier müsste dein ICP Algorithmus stehen):
        # Angenommen ICP liefert eine Matrix `T_cam_hut` zurück.
        # Dummy Pose für Test (Hütte 2m vor der Kamera):
        T_cam_hut = np.eye(4)
        T_cam_hut[0, 3] = 2.0 
        
        # 4. Transformieren in Robot Frame
        # Wir brauchen die statische TF Camera -> Robot
        # Hier hardcoded oder besser per TF Listener holen. 
        # Angenommen wir haben T_robot_cam:
        # T_robot_hut = T_robot_cam * T_cam_hut
        
        # 5. Publishen
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "robot/chassis"
        pose_msg.pose.position.x = 2.0 # Dummy
        # ... orientation setzen ...
        self.publisher_.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PCLPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
