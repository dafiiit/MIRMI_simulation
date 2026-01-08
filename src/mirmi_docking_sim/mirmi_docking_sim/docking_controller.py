#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros
from tf2_ros import TransformException
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
        
        # Offset-Variablen
        self.odom_offset_x = 0.0
        self.odom_offset_y = 0.0
        self.odom_offset_theta = 0.0
        self.raw_odom_pose = None 
        
        # Parameter
        self.declare_parameter('use_ground_truth', False)
        gt_param = self.get_parameter('use_ground_truth').value
        self.use_ground_truth = str(gt_param).lower() == 'true'
        
        # Subscriber für Odometrie
        if self.use_ground_truth:
            self.get_logger().warn("!!! ACHTUNG: Nutze GROUND TRUTH (/ground_truth) !!!")
            self.gt_sub = self.create_subscription(Odometry, '/ground_truth', self.ground_truth_callback, qos_profile_sensor_data)  
        else:
            self.get_logger().info("Modus: Nutze Standard-Odometrie (/odom)")
            self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        
        # Constants Map / Navigation
        self.HUT_CENTER = np.array([5.0, 2.0])
        self.TARGET_RADIUS = 5.5
        self.RADIUS_TOLERANCE = 0.20 
        self.WAYPOINT_OPENING = np.array([6.5, 2.0])
        self.WAYPOINT_INSIDE = np.array([5.5, 2.0])
        
        # Constants Control
        self.MAX_LINEAR_SPEED = 0.5
        self.MAX_ANGULAR_SPEED = 1.0
        
        # Ziel-Parameter für das Docking (Generisch)
        self.DOCKING_TARGET_DISTANCE = 0.45 # Meter vom Hüttenzentrum entfernt stoppen (entspricht 1.9m von Rückwand)
        self.ALIGN_Y_TOLERANCE = 0.05      # 5cm Toleranz seitlich
        self.ALIGN_YAW_TOLERANCE = 0.05    # ca. 3 Grad Toleranz
        
        # Status Variablen
        self.current_odom_pose = None
        self.has_localized = False
        self.odom_received_once = False
        
        # Generische Pose-Variablen (statt AprilTag Detections)
        self.latest_docking_pose = None
        self.last_pose_time = self.get_clock().now()
        
        self.arc_direction = 1.0
        self.current_arc_goal = None
        
        self.state = DockingState.SEARCHING
        self.state_entry_time = self.get_clock().now()
        
        # Variablen für die 360 Grad Suche
        self.search_accumulated_yaw = 0.0
        self.last_yaw_search = None

        # TF & Tools
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publisher
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/docking_controller/state', state_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.pose_sub = self.create_subscription(
            PoseStamped, 
            '/localization/docking_pose', 
            self.pose_callback, 
            10)

        # PointCloud Subscriber for Costmap
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/depth/points',
            self.pc_callback,
            qos_profile_sensor_data
        )
        self.local_costmap = None
        self.costmap_resolution = 0.1 # 10cm grid
        self.costmap_width = 40 # 4m width
        self.costmap_height = 40 # 4m height
        self.costmap_origin_x = 20 # Robot at center X
        self.costmap_origin_y = 20 # Robot at center Y
       
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self._docking_drive_active = False
        
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.set_initial_pose_callback, 10)
        
        self.get_logger().info("GENERIC DOCKING CONTROLLER GESTARTET (PCL & AprilTag kompatibel)")

    def log_debug(self, msg):
        self.get_logger().info(msg, throttle_duration_sec=1.0)

    def ground_truth_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        # Update Raw Pose für Offset-Berechnung
        self.raw_odom_pose = (pos.x, pos.y, yaw)
        
        self.current_odom_pose = (pos.x, pos.y, yaw)
        if not self.odom_received_once: self.odom_received_once = True
    
    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def set_initial_pose_callback(self, msg):
        if self.raw_odom_pose is None: return
        
        target_x = msg.pose.pose.position.x
        target_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, target_theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        raw_x, raw_y, raw_theta = self.raw_odom_pose
        
        self.odom_offset_x = target_x - raw_x
        self.odom_offset_y = target_y - raw_y
        self.odom_offset_theta = self.normalize_angle(target_theta - raw_theta)
        
        # Reset State
        self.change_state(DockingState.SEARCHING)
        self.has_localized = False
        self.latest_docking_pose = None
        self.current_arc_goal = None
        
        # Reset Search Vars
        self.search_accumulated_yaw = 0.0
        self.last_yaw_search = None
        
        self.publish_twist(0.0, 0.0)
        self.get_logger().info(f"⚠️ RESET: State -> SEARCHING. Start Rotation Check.")

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        _, _, raw_yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        self.raw_odom_pose = (pos.x, pos.y, raw_yaw)
        
        eff_x = pos.x + self.odom_offset_x
        eff_y = pos.y + self.odom_offset_y
        eff_yaw = self.normalize_angle(raw_yaw + self.odom_offset_theta)
        
        self.current_odom_pose = (eff_x, eff_y, eff_yaw)
        if not self.odom_received_once: self.odom_received_once = True

    def change_state(self, new_state: DockingState):
        self.state = new_state
        self.state_entry_time = self.get_clock().now()
        
        # Reset search specific vars if we leave searching
        if new_state != DockingState.SEARCHING:
            self.search_accumulated_yaw = 0.0
            self.last_yaw_search = None
            
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)
        self.get_logger().info(f"STATE CHANGE: -> {new_state.name}")

    # --- NEUER CALLBACK FÜR GENERISCHE POSE ---
    def pose_callback(self, msg: PoseStamped):
        """
        Empfängt die relative Pose der Hütte (vom Roboter aus gesehen).
        Funktioniert sowohl für AprilTag als auch für PCL Adapter.
        """
        self.latest_docking_pose = msg
        self.last_pose_time = self.get_clock().now()
        
        # Wenn wir im Suchmodus sind und eine Pose empfangen, haben wir das Ziel gefunden
        if self.state == DockingState.SEARCHING:
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(f"Ziel erkannt (Pose empfangen)! Age: {(self.get_clock().now() - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds / 1e9:.2f}s. Wechsle zu LOCALIZING.")
            self.change_state(DockingState.LOCALIZING)
            
        # Wenn wir im Localizing Modus sind
        if self.state == DockingState.LOCALIZING:
            if self.current_odom_pose is not None and not self.has_localized:
                self.has_localized = True
                self.change_state(DockingState.GOTO_RADIUS_POINT)

    def pc_callback(self, msg):
        """
        Generates a simple local costmap from PointCloud2.
        Robot is at center.
        """
        # Convert to numpy
        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(gen))
        if len(points) == 0: return

        # Filter: Only points in front and relevant height
        # Camera frame: Z forward, X right, Y down
        # We want X (right) and Z (forward) for 2D map
        # Height: Y. Ground is > 0.20. Obstacles are < 0.15 roughly?
        # Let's assume anything with Y < 0.15 (above ground) is an obstacle
        
        mask = (points['y'] < 0.15) & (points['z'] > 0.1) & (points['z'] < 4.0)
        obstacle_points = points[mask]
        
        # Create Grid
        grid = np.zeros((self.costmap_height, self.costmap_width), dtype=np.int8)
        
        # Map points to grid indices
        # Grid X (rows) -> Robot Z (Forward)
        # Grid Y (cols) -> Robot -X (Left)? 
        # Let's align with Robot Base Frame: X Forward, Y Left.
        # Camera: Z Forward, -X Left.
        
        # Robot X (Forward) = Camera Z
        # Robot Y (Left) = Camera -X
        
        r_x = obstacle_points['z']
        r_y = -obstacle_points['x']
        
        # Indices
        # Row (X): 0 is bottom (behind), Height is top (forward)
        # Robot is at index [origin_x, origin_y]
        
        idx_x = (r_x / self.costmap_resolution).astype(int)
        idx_y = (r_y / self.costmap_resolution).astype(int) + self.costmap_origin_y
        
        # Clip
        valid_mask = (idx_x >= 0) & (idx_x < self.costmap_height) & \
                     (idx_y >= 0) & (idx_y < self.costmap_width)
                     
        grid[idx_x[valid_mask], idx_y[valid_mask]] = 100 # Occupied
        
        self.local_costmap = grid

    def check_collision(self, lin_vel, ang_vel, time_horizon=1.0):
        """
        Simple collision check for a trajectory.
        """
        if self.local_costmap is None: return False
        
        # Simulate trajectory
        steps = 10
        dt = time_horizon / steps
        x, y, theta = 0.0, 0.0, 0.0
        
        for _ in range(steps):
            x += lin_vel * math.cos(theta) * dt
            y += lin_vel * math.sin(theta) * dt
            theta += ang_vel * dt
            
            # Check grid
            idx_x = int(x / self.costmap_resolution)
            idx_y = int(y / self.costmap_resolution) + self.costmap_origin_y
            
            if 0 <= idx_x < self.costmap_height and 0 <= idx_y < self.costmap_width:
                if self.local_costmap[idx_x, idx_y] > 50:
                    return True
        return False

    def calculate_radius_entry_point(self, current_pos):
        vec_to_center = self.HUT_CENTER - current_pos
        dist_to_center = np.linalg.norm(vec_to_center)
        if dist_to_center < 0.01: return current_pos
        unit_vec = vec_to_center / dist_to_center
        target_point = self.HUT_CENTER - (unit_vec * self.TARGET_RADIUS)
        return target_point

    def is_pose_fresh(self, timeout=1.0):
        """Prüft, ob die letzte Pose aktuell genug ist."""
        if self.latest_docking_pose is None:
            return False
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(self.latest_docking_pose.header.stamp)).nanoseconds / 1e9
        return age < timeout

    def control_loop(self):
        if self.current_odom_pose is None: 
            return
        
        cx, cy, ctheta = self.current_odom_pose
        current_pos_vec = np.array([cx, cy])
        
        self.log_debug(f"[{self.state.name}] Pose: x={cx:.1f}, y={cy:.1f}, th={ctheta:.2f}")
        
        # --- 1. SEARCHING (Rotation) ---
        if self.state == DockingState.SEARCHING:
            # Yaw Unterschied berechnen
            if self.last_yaw_search is not None:
                delta = abs(self.normalize_angle(ctheta - self.last_yaw_search))
                self.search_accumulated_yaw += delta
            
            self.last_yaw_search = ctheta
            
            # Check ob 360 Grad (2*PI = 6.28) überschritten
            if self.search_accumulated_yaw > 6.4:
                self.get_logger().warn("TIMEOUT: 360 Grad gedreht, kein Ziel gefunden.")
                self.publish_twist(0.0, 0.0)
                self.change_state(DockingState.SEARCH_TIMEOUT)
                return

            self.publish_twist(0.0, 0.3) # Langsam drehen
            return
        
        if self.state == DockingState.SEARCH_TIMEOUT:
            self.publish_twist(0.0, 0.0)
            return
        
        if self.state == DockingState.LOCALIZING:
            self.publish_twist(0.0, 0.0)
            # Der Zustandswechsel passiert jetzt im pose_callback
            return
        
        # --- 2. GLOBAL APPROACH (Odometry based) ---
        if self.state == DockingState.GOTO_RADIUS_POINT:
            target_point = self.calculate_radius_entry_point(current_pos_vec)
            # Wenn wir da sind, wechseln wir zum Bogen
            if self.simple_go_to(target_point, tolerance=self.RADIUS_TOLERANCE):
                opening_angle = math.atan2(self.WAYPOINT_OPENING[1] - self.HUT_CENTER[1], 
                                           self.WAYPOINT_OPENING[0] - self.HUT_CENTER[0])
                current_angle = math.atan2(cy - self.HUT_CENTER[1], cx - self.HUT_CENTER[0])
                angle_diff = self.normalize_angle(opening_angle - current_angle)
                self.arc_direction = 1.0 if angle_diff > 0 else -1.0
                self.change_state(DockingState.FOLLOW_ARC)
            return
            
        if self.state == DockingState.FOLLOW_ARC:
            # Check distance to target point (10.5, 2.0)
            # We use the "center between wheels" which is approximately the robot's position (cx, cy)
            if cx > 5:
                dist_to_target = abs(cy)
            else:
                dist_to_target = 10     # einfach eine Zahl > unsere toleranz
            print("aktuelle position x: ", cx)
            print("aktuelle position y: ", cy)
            
            # Restore current_angle calculation needed for next_angle
            current_angle = math.atan2(cy - self.HUT_CENTER[1], cx - self.HUT_CENTER[0])
            
            if dist_to_target < 0.1: # 10cm tolerance
                self.publish_twist(0.0, 0.0)
                self.current_arc_goal = None
                self.change_state(DockingState.ALIGN_TO_HUT)
                return

            step_angle = 0.8 / self.TARGET_RADIUS
            next_angle = current_angle + (self.arc_direction * step_angle)
            gx = self.HUT_CENTER[0] + self.TARGET_RADIUS * math.cos(next_angle)
            gy = self.HUT_CENTER[1] + self.TARGET_RADIUS * math.sin(next_angle)
            self.current_arc_goal = np.array([gx, gy])
            self.simple_go_to(self.current_arc_goal, tolerance=0.2)
            return
            
        if self.state == DockingState.ALIGN_TO_HUT:
            # Grobe Ausrichtung mittels Odometrie
            target_theta = math.atan2(self.WAYPOINT_INSIDE[1] - cy, self.WAYPOINT_INSIDE[0] - cx)
            angle_err = self.normalize_angle(target_theta - ctheta)
            
            if abs(angle_err) < 0.05:
                self.change_state(DockingState.FINAL_ALIGNMENT)
                self.publish_twist(0.0, 0.0)
            else:
                # Reduced gain to 0.8 to avoid oscillations
                ang_vel = np.clip(0.8 * angle_err, -0.4, 0.4)
                self.publish_twist(0.0, ang_vel)
            return
            
        
            
        # --- 3. FINAL ALIGNMENT (HIER WAR DER FEHLER) ---
        if self.state == DockingState.FINAL_ALIGNMENT:
            if not self.is_pose_fresh(timeout=1.0):
                self.publish_twist(0.0, 0.0)
                age = "None"
                if self.latest_docking_pose:
                     age = f"{(self.get_clock().now() - rclpy.time.Time.from_msg(self.latest_docking_pose.header.stamp)).nanoseconds / 1e9:.2f}s"
                self.get_logger().warn(f"Warte auf Pose... (Last Pose Age: {age})", throttle_duration_sec=2.0)
                return
            
            y_error = self.latest_docking_pose.pose.position.y
            q = self.latest_docking_pose.pose.orientation
            _, _, yaw_error = euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            # Debugging
            self.log_debug(f"ALIGN: Y_err={y_error:.3f}, Yaw_err={yaw_error:.3f}")

            # FIX 1: Toleranz für Y deutlich erhöht (auf 3.0m), da wir das während der Fahrt korrigieren
            if abs(y_error) < 3.0 and abs(yaw_error) < 0.1:
                self.change_state(DockingState.DOCKING)
                self.publish_twist(0.0, 0.0)
                self.get_logger().info("Ausrichtung OK. Beginne DOCKING.")
            else:
                # FIX 2: Nur Winkel korrigieren! 
                # Wir ignorieren y_error hier absichtlich, um "Zappeln" zu verhindern.
                # y_error korrigieren wir gleich im DOCKING-State durch Kurvenfahrt.
                ang_vel = 1.0 * yaw_error 
                ang_vel = np.clip(ang_vel, -0.4, 0.4)
                
                # Check Collision before turning? Turning in place is usually safe(r)
                self.publish_twist(0.0, ang_vel)
            return
            
        # --- 4. DOCKING (HIER KORRIGIEREN WIR Y) ---
        if self.state == DockingState.DOCKING:
            if not self.is_pose_fresh(timeout=1.0):
                self.publish_twist(0.0, 0.0)
                return

            current_dist = self.latest_docking_pose.pose.position.x
            dist_error = current_dist - self.DOCKING_TARGET_DISTANCE
            y_error = self.latest_docking_pose.pose.position.y # Seitlicher Versatz

            if abs(dist_error) < 0.1:
                self.change_state(DockingState.FINAL_STOP)
                self.publish_twist(0.0, 0.0)
                self.get_logger().info("ZIEL ERREICHT: Final Stop.")
            else:
                # Langsam vorfahren - Increased speed
                lin = np.clip(0.5 * dist_error, -0.3, 0.3)
                
                # Korrektur des seitlichen Versatzes WÄHREND der Fahrt funktioniert absolut überhaupt nicht. 
                # wenn er zu weit links ist muss er so weit nach RECHTS lenken (negativ), dass der Apriltag eher links im Bildschirm ist, bis er den y Versatz ausgeglichen hat. 
                # Moment: y_error ist Position der Hütte im Roboter Frame.
                # Wenn Hütte bei Y = +0.5 (Links), dann müssen wir nach Links fahren.
                # Ackermann/DiffDrive: Nach Links fahren = Nach Links drehen und vorwärts.
                # Also: y > 0 -> ang > 0.
                # Aktuelles Verhalten: Leider wackelt der Roboter und ist nicht in der Lage den y Versatz auszgleichen
                
                ang = np.clip(2.0 * y_error, -0.5, 0.5) 
                self.log_debug(f"DOCKING: Dist={dist_error:.2f}, Y_err={y_error:.2f} -> AngCmd={ang:.2f}") 
                
                # Collision Check
                if self.check_collision(lin, ang):
                    self.get_logger().warn("COLLISION AHEAD! Stopping.", throttle_duration_sec=1.0)
                    self.publish_twist(0.0, 0.0)
                    # Maybe try to rotate only?
                    if not self.check_collision(0.0, ang):
                         self.publish_twist(0.0, ang)
                    return

                self.publish_twist(lin, ang)
            return

        if self.state == DockingState.FINAL_STOP:
            self.publish_twist(0.0, 0.0)

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

    def publish_twist(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
