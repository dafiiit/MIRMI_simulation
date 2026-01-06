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
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial import KDTree

class PCLPosePublisher(Node):
    def __init__(self):
        super().__init__('pcl_pose_publisher')
        
        # TF Buffer für Transformationen
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Abonniere Punktwolke (Depth Camera)
        self.subscription = self.create_subscription(
            PointCloud2, '/depth/points', self.pc_callback, qos_profile_sensor_data)
            
        self.publisher_ = self.create_publisher(PoseStamped, '/localization/docking_pose', 10)
        
        # 2D Referenzmodell (nur Grundriss)
        self.reference_cloud = self.generate_2d_reference_cloud()
        self.reference_tree = KDTree(self.reference_cloud)
        
        self.last_transform = None 
        
        self.get_logger().info('PCL 2D-Localization Node started.')

    def generate_2d_reference_cloud(self):
        """
        Erstellt einen 2D-Grundriss der Hütte (U-Form) bei Z=0.
        Basierend auf SDF Maßen.
        """
        points = []
        density = 30 # Punkte pro Meter für hohe Genauigkeit
        
        def add_line(start, end):
            dist = np.linalg.norm(end - start)
            steps = int(dist * density)
            for i in range(steps + 1):
                t = i / max(1, steps)
                p = start + t * (end - start)
                points.append(p)

        # Maße aus der SDF (docking_world.sdf):
        # Back wall: x=-1.45. Breite Y geht von ca. -1.1 bis +1.1
        p_back_left = np.array([-1.45, 1.1, 0.0])
        p_back_right = np.array([-1.45, -1.1, 0.0])
        
        # Left wall: y=1.05. Länge X geht von ca. -1.5 bis +1.5
        p_left_back = np.array([-1.5, 1.05, 0.0])
        p_left_front = np.array([1.5, 1.05, 0.0])
        
        # Right wall: y=-1.05. Länge X geht von ca. -1.5 bis +1.5
        p_right_back = np.array([-1.5, -1.05, 0.0])
        p_right_front = np.array([1.5, -1.05, 0.0])

        # Linien generieren (U-Form)
        add_line(p_back_left, p_back_right)  # Rückwand
        add_line(p_left_back, p_left_front)  # Linke Wand
        add_line(p_right_back, p_right_front) # Rechte Wand
        
        return np.array(points, dtype=np.float32)

    def get_transform_matrix(self, target_frame, source_frame):
        """Holt die TF-Transformation als 4x4 Matrix."""
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time())
            
            trans = t.transform.translation
            rot = t.transform.rotation
            
            T = np.eye(4)
            T[:3, 3] = [trans.x, trans.y, trans.z]
            r = R.from_quat([rot.x, rot.y, rot.z, rot.w])
            T[:3, :3] = r.as_matrix()
            return T
        except TransformException:
            return None

    def pc_callback(self, msg):
        # 1. Hole Transformation Camera -> Robot Chassis
        # Wir wollen alles im Roboter-Koordinatensystem rechnen, um Tilt zu ignorieren.
        T_robot_cam = self.get_transform_matrix("robot/chassis", msg.header.frame_id)
        if T_robot_cam is None:
            return

        # 2. Pointcloud lesen
        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        raw_points = np.array(list(gen))
        if len(raw_points) == 0: return
        
        points_cam = np.column_stack((raw_points['x'], raw_points['y'], raw_points['z'])) # Nx3
        
        # 3. Transformiere Punkte in den Robot-Frame (homogene Koordinaten)
        points_hom = np.ones((len(points_cam), 4))
        points_hom[:, :3] = points_cam
        points_robot = np.dot(T_robot_cam, points_hom.T).T[:, :3] # Nx3 im Chassis Frame
        
        # 4. Filterung im Roboter-Frame
        # Boden entfernen: Alles unter 5cm Höhe
        # Decke entfernen: Alles über 1.0m Höhe
        # Entfernung begrenzen: Nur Punkte im Umkreis von 5m
        mask = (points_robot[:, 2] > 0.05) & \
               (points_robot[:, 2] < 1.0) & \
               (points_robot[:, 0] < 5.0) 
        
        scene_points = points_robot[mask]
        
        # Downsampling für Performance
        if len(scene_points) > 500:
            idx = np.random.choice(len(scene_points), 500, replace=False)
            scene_points = scene_points[idx]
            
        if len(scene_points) < 50:
            return

        # 5. 2D PROJEKTION: Zwinge alle Punkte auf Z=0
        scene_points[:, 2] = 0.0

        # 6. Initial Guess (Tracking oder Initialisierung)
        if self.last_transform is not None:
             T_init = self.last_transform
        else:
            # Einfaches Centroid Alignment für den Start
            centroid_src = np.mean(scene_points, axis=0)
            centroid_dst = np.mean(self.reference_cloud, axis=0)
            translation = centroid_dst - centroid_src
            T_init = np.eye(4)
            T_init[:3, 3] = translation

        # 7. ICP (Source: Scene, Target: Reference)
        # Findet Transform T, so dass: Ref = T * Scene
        # Das bedeutet T ist T_hut_robot (Pose der Hütte relativ zum Roboter ist aber das Inverse!)
        T_hut_robot, fitness, _ = self.icp(scene_points, self.reference_cloud, init_pose=T_init)
        
        if fitness > 0.1: # Fitness Threshold (in Metern)
            self.get_logger().warn(f"ICP schlechte Qualität: {fitness:.3f}")
            if fitness > 0.5: self.last_transform = None # Reset bei Tracking-Verlust
            return

        self.last_transform = T_hut_robot

        # 8. Pose berechnen
        # ICP liefert T_hut_robot (Transformiert Punkte VOM Roboter ZUR Hütte)
        # Die Pose der Hütte im Roboter-Frame ist aber genau die Umkehrung (oder Matrix direkt?)
        # Moment: ICP Formel: dst = T * src. 
        # src=RobotFrame, dst=HutFrame. -> P_hut = T * P_robot.
        # Die Pose der Hütte (Wo ist die Hütte?) wird durch die Translation von T_robot_hut beschrieben.
        # Also brauchen wir T_robot_hut = inv(T_hut_robot).
        
        T_robot_hut = np.linalg.inv(T_hut_robot)
        
        # Erzwinge 2D Pose (kein Roll/Pitch, Z=0)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "robot/chassis"
        
        pose_msg.pose.position.x = T_robot_hut[0, 3]
        pose_msg.pose.position.y = T_robot_hut[1, 3]
        pose_msg.pose.position.z = 0.0 # Force Floor
        
        # Rotation extrahieren, aber nur Yaw (Drehung um Z)
        rot_mat = T_robot_hut[:3, :3]
        r = R.from_matrix(rot_mat)
        euler = r.as_euler('xyz')
        # Nur Yaw behalten, Roll/Pitch nullen
        r_clean = R.from_euler('xyz', [0, 0, euler[2]])
        quat = r_clean.as_quat()
        
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        self.publisher_.publish(pose_msg)

    # --- Standard ICP Hilfsfunktionen (unverändert zur Logik, nur sauberer) ---
    def best_fit_transform(self, A, B):
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        AA = A - centroid_A
        BB = B - centroid_B
        H = np.dot(AA.T, BB)
        U, S, Vt = np.linalg.svd(H)
        R_mat = np.dot(Vt.T, U.T)
        if np.linalg.det(R_mat) < 0:
            Vt[2,:] *= -1
            R_mat = np.dot(Vt.T, U.T)
        t = centroid_B.T - np.dot(R_mat, centroid_A.T)
        T = np.identity(4)
        T[:3, :3] = R_mat
        T[:3, 3] = t
        
        return T, R_mat, t

    def icp(self, A, B, init_pose=None, max_iterations=20, tolerance=0.001):
        src = np.copy(A)
        dst = np.copy(B) # Reference
        
        if init_pose is not None:
            src_h = np.ones((src.shape[0], 4))
            src_h[:,0:3] = src
            src = np.dot(init_pose, src_h.T).T[:, :3]

        prev_error = 0
        final_T = init_pose if init_pose is not None else np.eye(4)

        for i in range(max_iterations):
            # Nearest Neighbors
            distances, indices = self.reference_tree.query(src)
            
            # Outlier Rejection (nur Punkte die nah an der Modell-Kontur sind)
            valid = distances < 0.5 
            if np.sum(valid) < 10: break
            
            src_valid = src[valid]
            dst_valid = dst[indices[valid]]
            
            # Berechne Delta-Transform für diesen Schritt
            T, _, _ = self.best_fit_transform(src_valid, dst_valid)
            
            # Update Source
            src_h = np.ones((src.shape[0], 4))
            src_h[:,0:3] = src
            src = np.dot(T, src_h.T).T[:, :3]
            
            # Update Gesamt-Transform
            final_T = np.dot(T, final_T)
            
            mean_error = np.mean(distances[valid])
            if np.abs(prev_error - mean_error) < tolerance:
                break
            prev_error = mean_error

        # Fitness Check am Ende
        distances, _ = self.reference_tree.query(src)
        fitness = np.mean(distances[distances < 0.5]) if np.any(distances < 0.5) else 100.0
        
        return final_T, fitness, i

def main(args=None):
    rclpy.init(args=args)
    node = PCLPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
