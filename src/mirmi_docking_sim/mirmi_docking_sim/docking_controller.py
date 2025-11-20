#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from std_msgs.msg import String

class DockingState(Enum):
    SEARCHING = 1
    LOCALIZING = 2
    GOTO_RADIUS_POINT = 3
    FOLLOW_ARC = 4
    ALIGN_TO_HUT = 5
    FINAL_ALIGNMENT = 6
    DOCKING = 7
    FINAL_STOP = 8
    SEARCH_TIMEOUT = 9  

class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')
        
        # --- Parameter ---
        self.declare_parameter('use_ground_truth', False)
        self.use_ground_truth = self.get_parameter('use_ground_truth').value
        
        # --- Constants ---
        self.HUT_CENTER_GLOBAL = np.array([5.0, 2.0]) # Globale Position der Hütte (für Navigation)
        self.TARGET_RADIUS = 5.5
        self.RADIUS_TOLERANCE = 0.20 
        self.WAYPOINT_OPENING = np.array([6.5, 2.0])
        self.WAYPOINT_INSIDE = np.array([5.5, 2.0])
        
        self.MAX_LINEAR_SPEED = 0.5
        self.MAX_ANGULAR_SPEED = 1.0
        
        # Toleranzen für Final Alignment (Pose-basiert statt Pixel)
        self.ALIGN_Y_TOLERANCE = 0.05  # 5cm seitlicher Versatz erlaubt
        self.ALIGN_YAW_TOLERANCE = 0.05 # ca 3 Grad
        self.DOCKING_DIST_TARGET = 1.9 # Meter vor der Hütte stoppen
        
        # --- State Variables ---
        self.state = DockingState.SEARCHING
        self.current_odom_pose = None     # (x, y, theta) global
        self.latest_hut_pose = None       # PoseStamped (Relativ zum Roboter)
        self.last_pose_time = self.get_clock().now()
        
        self.has_localized = False
        self.odom_offset_x = 0.0
        self.odom_offset_y = 0.0
        self.odom_offset_theta = 0.0
        self.raw_odom_pose = None

        # --- Search logic ---
        self.search_accumulated_yaw = 0.0
        self.last_yaw_search = None

        # --- Subscribers / Publishers ---
        
        # 1. Odometrie (Global)
        odom_topic = '/ground_truth' if self.use_ground_truth else '/odom'
        self.create_subscription(Odometry, odom_topic, self.odom_callback, qos_profile_sensor_data)
        
        # 2. Die neue SCHNITTSTELLE: Relative Pose der Hütte (vom Adapter)
        self.create_subscription(PoseStamped, '/localization/docking_pose', self.pose_callback, 10)
        
        # 3. Command Velocity
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 4. State & Initial Pose
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/docking_controller/state', state_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.set_initial_pose_callback, 10)

        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("GENERIC DOCKING CONTROLLER GESTARTET")

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def pose_callback(self, msg: PoseStamped):
        """Empfängt die relative Pose der Hütte (robot/chassis -> docking_frame)"""
        self.latest_hut_pose = msg
        self.last_pose_time = self.get_clock().now()
        
        # Wenn wir suchen, haben wir es jetzt gefunden!
        if self.state == DockingState.SEARCHING:
            self.publish_twist(0.0, 0.0)
            self.get_logger().info("Ziel erkannt! Wechsle zu LOCALIZING.")
            self.change_state(DockingState.LOCALIZING)

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        _, _, raw_yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        self.raw_odom_pose = (pos.x, pos.y, raw_yaw)
        
        # Offset anwenden
        eff_x = pos.x + self.odom_offset_x
        eff_y = pos.y + self.odom_offset_y
        eff_yaw = self.normalize_angle(raw_yaw + self.odom_offset_theta)
        self.current_odom_pose = (eff_x, eff_y, eff_yaw)

    def set_initial_pose_callback(self, msg):
        if self.raw_odom_pose is None: return
        # Setze Offset basierend auf Rviz "2D Pose Estimate"
        target_x = msg.pose.pose.position.x
        target_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, target_theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        raw_x, raw_y, raw_theta = self.raw_odom_pose
        self.odom_offset_x = target_x - raw_x
        self.odom_offset_y = target_y - raw_y
        self.odom_offset_theta = self.normalize_angle(target_theta - raw_theta)
        
        self.change_state(DockingState.SEARCHING)
        self.has_localized = False
        self.search_accumulated_yaw = 0.0
        self.last_yaw_search = None
        self.get_logger().info("RESET: State -> SEARCHING")

    def change_state(self, new_state):
        self.state = new_state
        self.state_pub.publish(String(data=self.state.name))
        self.get_logger().info(f"STATE: {self.state.name}")

    def is_pose_fresh(self, timeout=1.0):
        if self.latest_hut_pose is None: return False
        delay = (self.get_clock().now() - rclpy.time.Time.from_msg(self.latest_hut_pose.header.stamp)).nanoseconds / 1e9
        return delay < timeout

    def control_loop(self):
        if self.current_odom_pose is None: return
        cx, cy, ctheta = self.current_odom_pose
        
        # --- 1. SEARCHING (Rotation) ---
        if self.state == DockingState.SEARCHING:
            if self.last_yaw_search is not None:
                delta = abs(self.normalize_angle(ctheta - self.last_yaw_search))
                self.search_accumulated_yaw += delta
            self.last_yaw_search = ctheta
            
            if self.search_accumulated_yaw > 6.4:
                self.publish_twist(0.0, 0.0)
                self.change_state(DockingState.SEARCH_TIMEOUT)
                return
            self.publish_twist(0.0, 0.3)
            return

        if self.state == DockingState.SEARCH_TIMEOUT: return

        # --- 2. LOCALIZING (Map Correction) ---
        if self.state == DockingState.LOCALIZING:
            # Hier könnten wir die Odometrie korrigieren, indem wir die bekannte Hütte (Global) 
            # mit der gemessenen relativen Pose vergleichen.
            # Fürs erste skippen wir komplexe Map-Korrektur und fahren einfach los.
            self.has_localized = True
            self.change_state(DockingState.GOTO_RADIUS_POINT)
            return

        # --- 3. GLOBAL NAVIGATION (Arc Approach) ---
        if self.state in [DockingState.GOTO_RADIUS_POINT, DockingState.FOLLOW_ARC]:
            # ... (Identisch zum alten Code, nutzt globale Odometrie) ...
            # Verkürzt für Übersichtlichkeit:
            target = self.calculate_nav_target(cx, cy) # Hilfsfunktion
            if target is not None:
                self.simple_go_to(target)
            return

        # --- 4. ALIGN TO HUT (Global alignment before tracking) ---
        if self.state == DockingState.ALIGN_TO_HUT:
            target_theta = math.atan2(self.WAYPOINT_INSIDE[1] - cy, self.WAYPOINT_INSIDE[0] - cx)
            angle_err = self.normalize_angle(target_theta - ctheta)
            if abs(angle_err) < 0.05:
                self.change_state(DockingState.FINAL_ALIGNMENT)
                self.publish_twist(0.0, 0.0)
            else:
                self.publish_twist(0.0, np.clip(1.5 * angle_err, -0.5, 0.5))
            return

        # --- 5. FINAL ALIGNMENT (Visual Servoing via Pose) ---
        if self.state == DockingState.FINAL_ALIGNMENT:
            if not self.is_pose_fresh(timeout=1.0):
                self.publish_twist(0.0, 0.0) # Wait for data
                return
            
            # Pose ist im Frame 'robot/chassis'. 
            # y > 0 -> Hütte ist links -> Roboter muss nach links drehen
            y_error = self.latest_hut_pose.pose.position.y
            
            # Yaw Error aus Quaternions der relativen Pose
            q = self.latest_hut_pose.pose.orientation
            _, _, yaw_error = euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            # Wir wollen y=0 und yaw=0 (gerade vor der Hütte)
            if abs(y_error) < self.ALIGN_Y_TOLERANCE and abs(yaw_error) < self.ALIGN_YAW_TOLERANCE:
                self.change_state(DockingState.DOCKING)
                self.publish_twist(0.0, 0.0)
            else:
                # Einfacher P-Regler auf den Winkel, mit Y-Einfluss
                # Wenn Y Error da ist, drehe etwas, um ihn zu korrigieren
                ang_vel = 1.5 * yaw_error + 2.0 * y_error
                self.publish_twist(0.0, np.clip(ang_vel, -0.4, 0.4))
            return

        # --- 6. DOCKING (Drive Forward) ---
        if self.state == DockingState.DOCKING:
            if not self.is_pose_fresh(timeout=2.0):
                # Blind weiterfahren oder stoppen? Stoppen.
                self.publish_twist(0.0, 0.0)
                return

            # Distanz ist x
            dist_error = self.latest_hut_pose.pose.position.x - self.DOCKING_DIST_TARGET
            y_error = self.latest_hut_pose.pose.position.y

            if abs(dist_error) < 0.1:
                self.change_state(DockingState.FINAL_STOP)
                self.publish_twist(0.0, 0.0)
            else:
                lin = np.clip(0.3 * dist_error, -0.15, 0.15)
                ang = np.clip(2.0 * y_error, -0.3, 0.3) # Feinkorrektur
                self.publish_twist(lin, ang)
            return

        if self.state == DockingState.FINAL_STOP:
            self.publish_twist(0.0, 0.0)

    def calculate_nav_target(self, cx, cy):
        # Logik für GOTO_RADIUS und FOLLOW_ARC (vereinfacht aus deinem Code kopieren)
        # Hier nur Placeholder Logik:
        if self.state == DockingState.GOTO_RADIUS_POINT:
             # ... Logik ...
             self.change_state(DockingState.ALIGN_TO_HUT) # Skip ARC for simplicity in this snippet
        return None

    def simple_go_to(self, target_pos, tolerance):
        if self.current_odom_pose is None: return False
        cx, cy, ctheta = self.current_odom_pose
        dist = np.linalg.norm(target_pos - np.array([cx, cy]))
        if dist < tolerance:
            self.publish_twist(0.0, 0.0)
            return True
        target_angle = math.atan2(target_pos[1] - cy, target_pos[0] - cx)
        angle_diff = self.normalize_angle(target_angle - ctheta)
        ang_vel = np.clip(2.0 * angle_diff, -self.MAX_ANGULAR_SPEED, self.MAX_ANGULAR_SPEED)
        if abs(angle_diff) > 0.8:
            lin_vel = 0.0
        else:
            speed_factor = (1.0 - (abs(angle_diff) / 0.8)) 
            lin_vel = np.clip(1.0 * dist, 0.0, self.MAX_LINEAR_SPEED) * speed_factor
        self.publish_twist(lin_vel, ang_vel)
        return False

    def publish_twist(self, lin, ang):
        cmd = Twist()
        cmd.linear.x = float(lin)
        cmd.angular.z = float(ang)
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
