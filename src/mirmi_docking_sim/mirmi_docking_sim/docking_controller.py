#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
from apriltag_msgs.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String


class DockingState(Enum):
    SEARCHING = 1          # 1. Dreht sich, bis Tag gesehen wird
    LOCALIZING = 2         # 2. Bestimmt Welt-Position
    GOTO_RADIUS_POINT = 3  # 3. Fährt ZU einem Punkt auf dem 10m-Radius
    FOLLOW_ARC = 4         # 5. Fährt auf dem 10m-Radius bis zur Öffnung
    ALIGN_TO_HUT = 5       # 6. Dreht sich zur Öffnung
    FINAL_ALIGNMENT = 6    # 7. Steht still, dreht sich (PIXEL-BASIERT) auf Tag 1
    DOCKING = 7            # 7. Fährt 10cm, prüft Distanz, geht zu 6
    FINAL_STOP = 8         # 8. Ziel erreicht


class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')
        
        # PARAMETER: Ground Truth für Tests nutzen?
        self.declare_parameter('use_ground_truth', False)
        self.use_ground_truth = self.get_parameter('use_ground_truth').value
        
        if self.use_ground_truth:
            self.get_logger().warn("!!! ACHTUNG: Nutze GROUND TRUTH (Simulation) statt Odometrie !!!")
        
        # Zustand der State Machine
        self.state = DockingState.SEARCHING
        self.state_entry_time = self.get_clock().now()
        
        # Konstanten für die Navigation
        self.HUT_CENTER = np.array([5.0, 2.0])
        self.TARGET_RADIUS = 5.5
        self.RADIUS_TOLERANCE = 0.15
        
        # Wegpunkte (relativ zur Welt)
        self.WAYPOINT_OPENING = np.array([6.5, 2.0])  # Vor der Öffnung (für Ausrichtung)
        self.WAYPOINT_INSIDE = np.array([5.5, 2.0])   # In der Hütte (Endziel)
        
        # Ziel-Tag für die Endausrichtung
        self.DOCKING_TARGET_FRAME = 'tag36_11_00001' # TF-Frame (für Distanz)
        self.DOCKING_TARGET_ID = 1                   # Tag ID (für Pixel-Mitte)
        self.DOCKING_TARGET_DISTANCE = 1.9           # Ziel-Abstand (physisch)
        self.DOCKING_SPEED_ANGULAR = 1.0             # Maximale Dreh-Korrektur
        
        # KORREKTUR: Parameter für Pixel-basierte Ausrichtung
        self.IMAGE_WIDTH = 640.0                 # Gemäß camera_info.yaml
        self.IMAGE_CENTER_X = self.IMAGE_WIDTH / 2.0 # 320.0
        self.PIXEL_TOLERANCE = 10.0              # 10 Pixel Toleranz (horizontal)
        self.K_P_PIXEL = 0.005                   # P-Regler für Pixel-Fehler (Tuning nötig!)
        
        # Parameter für 30cm-Fahrmanöver
        self.NUDGE_SPEED = 0.15  # 0.15 m/s
        self.NUDGE_DURATION = 2.0 # 2.0s * 0.15m/s = 0.1m = 30cm
        
        # Aktuelle Roboter-Pose (x, y, theta) aus Odometrie
        self.current_odom_pose = None
        
        # Flag ob wir bereits lokalisiert haben
        self.has_localized = False
        
        # Speichert die letzte Detektion von Tag 1
        self.last_target_detection = None
        self.last_detection_time = self.get_clock().now()
        
        # Variablen für die Kreis-Logik
        self.arc_direction = 1.0
        self.current_arc_goal = None
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS für Gazebo
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS-Profil für "latching" State-Topic
        state_qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Publisher & Subscriber
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.state_pub = self.create_publisher(
            String,
            '/docking_controller/state',
            state_qos_profile
        )
        
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )
        
        if self.use_ground_truth:
            self.get_logger().warn("TEST-MODUS: Abonniere /model/robot/pose (Ground Truth)")
            self.gt_sub = self.create_subscription(
                PoseArray,
                '/model/robot/pose',
                self.ground_truth_callback,
                qos_profile 
            )
        else:
            self.odom_sub = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                qos_profile
            )
       
        
        # Haupt-Kontrollschleife
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Debug-Timer für regelmäßige Status-Ausgaben
        self.debug_timer = self.create_timer(2.0, self.debug_status)
        
        # Ziel für simple_go_to
        self.current_goal = None
        
        # Flags für die 10cm-Fahrmanöver
        self._align_nudge_active = False
        self._align_nudge_start_time = None
        self._docking_drive_active = False
        self._docking_drive_start_time = None
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("DOCKING CONTROLLER (PIXEL-ALIGN) GESTARTET")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Initial State: {self.state.name}")
        self.get_logger().info(f"Bildmitte X: {self.IMAGE_CENTER_X}px, Toleranz: {self.PIXEL_TOLERANCE}px")
        
        initial_state_msg = String()
        initial_state_msg.data = self.state.name
        self.state_pub.publish(initial_state_msg)

    def change_state(self, new_state: DockingState):
        """Hilfsfunktion für Zustandsübergänge mit Logging"""
        old_state = self.state
        self.state = new_state
        self.state_entry_time = self.get_clock().now()
        
        # Setze alle Manöver-Flags bei jedem State-Wechsel zurück
        self._align_nudge_active = False
        self._docking_drive_active = False
        
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"STATE CHANGE: {old_state.name} -> {new_state.name}")
        self.get_logger().info("=" * 60)

    def debug_status(self):
        """Regelmäßige Status-Ausgaben"""
        time_in_state = (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9
        
        status_lines = [
            "=" * 60,
            f"STATUS UPDATE - Zeit in {self.state.name}: {time_in_state:.1f}s",
            "=" * 60,
        ]
        
        if self.current_odom_pose is not None:
            x, y, theta = self.current_odom_pose
            status_lines.append(f"Odometrie: x={x:.2f}, y={y:.2f}, theta={math.degrees(theta):.1f}°")
        else:
            status_lines.append("Odometrie: KEINE DATEN ❌")
        
        status_lines.append(f"Lokalisierung: {'ERFOLGT ✓' if self.has_localized else 'AUSSTEHEND'}")
        
        # Zustandsspezifische Infos
        if self.state == DockingState.FINAL_ALIGNMENT:
            status_lines.append(f"Aktion: Richte auf Pixel-Mitte ({self.IMAGE_CENTER_X}px)...")
            if self._align_nudge_active:
                status_lines.append("WARNUNG: Tag verloren, fahre 10cm Nudge!")
            
            detection_age_sec = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
            if self.last_target_detection and detection_age_sec < 1.0:
                pixel_error = self.IMAGE_CENTER_X - self.last_target_detection.centre.x
                status_lines.append(f"Tag {self.DOCKING_TARGET_ID} sichtbar: Pixel X={self.last_target_detection.centre.x:.1f}, Fehler={pixel_error:.1f}px")
            else:
                status_lines.append(f"Tag {self.DOCKING_TARGET_ID} NICHT SICHTBAR.")
                
        elif self.state == DockingState.DOCKING:
            status_lines.append("Aktion: Prüfe Distanz / Fahre 10cm Schritt...")
            if self._docking_drive_active:
                status_lines.append("INFO: Führe 10cm Fahrt aus.")
        elif self.state == DockingState.FINAL_STOP:
            status_lines.append("Aktion: Ziel erreicht. Stopp.")
        else:
             status_lines.append(f"Aktion: {self.state.name}...")

        
        status_lines.append("=" * 60)
        
        for line in status_lines:
            self.get_logger().info(line)

    def ground_truth_callback(self, msg: PoseArray):
        """Liest die perfekte Pose aus der Simulation (für Tests)"""
        if len(msg.poses) == 0:
            return
            
        # Index 0 ist in der Regel das Modell selbst (Base Link)
        # Die Gazebo-Bridge sendet hier alle Posen des Modells
        robot_pose = msg.poses[0]
        
        pos = robot_pose.position
        orient = robot_pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        # Wir "faken" die Odometrie-Variable mit der perfekten Pose
        was_none = self.current_odom_pose is None
        self.current_odom_pose = (pos.x, pos.y, yaw)
        
        if was_none:
            self.get_logger().info(f"✓ Ground Truth empfangen: ({pos.x:.2f}, {pos.y:.2f})")
    
    def odom_callback(self, msg: Odometry):
        """Speichert die aktuelle Odometrie-Pose"""
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        was_none = self.current_odom_pose is None
        self.current_odom_pose = (pos.x, pos.y, yaw)
        
        if was_none:
            self.get_logger().info(f"✓ Erste Odometrie empfangen: ({pos.x:.2f}, {pos.y:.2f})")

    def detection_callback(self, msg: AprilTagDetectionArray):
        """Speichert die letzte Detektion des ZIEL-Tags"""
        
        # (Logik für SEARCHING/LOCALIZING)
        num_detections = len(msg.detections)
        if self.state == DockingState.SEARCHING and num_detections > 0:
            tag_ids = [d.id for d in msg.detections]
            self.get_logger().info(f"✓ Tag(s) {tag_ids} gefunden während SEARCHING")
            self.publish_twist(0.0, 0.0)
            self.change_state(DockingState.LOCALIZING)
            
        if self.state in [DockingState.SEARCHING, DockingState.LOCALIZING]:
            if num_detections > 0:
                self.run_localization(msg)
        
        # KORREKTUR: Suche und speichere immer die Detektion von Tag 1
        found_target = False
        if num_detections > 0:
            for detection in msg.detections:
                if detection.id == self.DOCKING_TARGET_ID:
                    self.last_target_detection = detection
                    self.last_detection_time = self.get_clock().now()
                    found_target = True
                    break # Wir haben unser Ziel-Tag
        
        if not found_target:
            # Setze es nicht auf None, damit der control_loop das Alter
            # der letzten Detektion prüfen kann
            pass

    def run_localization(self, msg: AprilTagDetectionArray):
        """Simuliert die erste Lokalisierung"""
        if self.current_odom_pose is None:
            return
            
        if not self.has_localized:
            self.has_localized = True
            self.get_logger().info("✓✓✓ ERSTE ERFOLGREICHE LOKALISIERUNG! ✓✓✓")
            self.change_state(DockingState.GOTO_RADIUS_POINT)
            

    def control_loop(self):
        """Haupt-Steuerungslogik"""
        
        if self.current_odom_pose is None:
            if self.state.value > DockingState.LOCALIZING.value:
                self.get_logger().warn("Warte auf Odometrie...", throttle_duration_sec=30.0)
            return
        
        current_pos_vec = np.array(self.current_odom_pose[:2])
        current_theta = self.current_odom_pose[2]
        
        # --- 1. Suchen ---
        if self.state == DockingState.SEARCHING:
            self.publish_twist(0.0, 0.3)
            return
        
        # --- 2. Lokalisieren ---
        if self.state == DockingState.LOCALIZING:
            self.publish_twist(0.0, 0.2)
            return
        
        # --- 3. Auf 10m Abstand fahren ---
        if self.state == DockingState.GOTO_RADIUS_POINT:
            # (Diese Logik bleibt unverändert)
            vec_to_center = self.HUT_CENTER - current_pos_vec
            dist_to_center = np.linalg.norm(vec_to_center)
            
            if dist_to_center > 0.01:
                target_point = self.HUT_CENTER - (vec_to_center / dist_to_center) * self.TARGET_RADIUS
                if self.simple_go_to(target_point, tolerance=self.RADIUS_TOLERANCE):
                    self.get_logger().info(f"✓ {self.TARGET_RADIUS}m Radius erreicht. Berechne Kreisbahn...")
                    opening_angle = math.atan2(self.WAYPOINT_OPENING[1] - self.HUT_CENTER[1], self.WAYPOINT_OPENING[0] - self.HUT_CENTER[0])
                    current_angle_on_circle = math.atan2(current_pos_vec[1] - self.HUT_CENTER[1], current_pos_vec[0] - self.HUT_CENTER[0])
                    angle_diff_to_opening = (opening_angle - current_angle_on_circle + math.pi) % (2 * math.pi) - math.pi
                    self.arc_direction = 1.0 if angle_diff_to_opening > 0 else -1.0
                    self.get_logger().info(f"✓ Kürzeste Richtung: {'CCW' if self.arc_direction > 0 else 'CW'}. Starte Kreisbahn.")
                    self.change_state(DockingState.FOLLOW_ARC)
            else:
                self.get_logger().warn("Roboter ist ZU NAH am Zentrum, fahre zurück.")
                target_point = current_pos_vec - vec_to_center * (self.TARGET_RADIUS + 1.0)
                self.simple_go_to(target_point, tolerance=self.RADIUS_TOLERANCE)
            return
            
        # --- 5. Kreis fahren (als Polygon-Annäherung) ---
        if self.state == DockingState.FOLLOW_ARC:
            # (Diese Logik bleibt unverändert)
            POLYGON_SEGMENT_LENGTH = 0.5
            GOAL_REACHED_TOLERANCE = 0.15
            
            current_angle_on_circle = math.atan2(current_pos_vec[1] - self.HUT_CENTER[1], current_pos_vec[0] - self.HUT_CENTER[0])
            opening_angle = math.atan2(self.WAYPOINT_OPENING[1] - self.HUT_CENTER[1], self.WAYPOINT_OPENING[0] - self.HUT_CENTER[0])
            angle_err_to_goal = (opening_angle - current_angle_on_circle + math.pi) % (2 * math.pi) - math.pi
            
            if abs(angle_err_to_goal) < 0.01:
                self.get_logger().info("✓ Schritt 6: Kreisbahn beendet (nahe Öffnung).")
                self.change_state(DockingState.ALIGN_TO_HUT)
                self.publish_twist(0.0, 0.0)
                self.current_arc_goal = None
                return

            if self.current_arc_goal is None:
                segment_angle_rad = POLYGON_SEGMENT_LENGTH / self.TARGET_RADIUS
                next_goal_angle = current_angle_on_circle + (self.arc_direction * segment_angle_rad)
                goal_x = self.HUT_CENTER[0] + self.TARGET_RADIUS * math.cos(next_goal_angle)
                goal_y = self.HUT_CENTER[1] + self.TARGET_RADIUS * math.sin(next_goal_angle)
                self.current_arc_goal = np.array([goal_x, goal_y])
                self.get_logger().info(f"Neues Polygon-Ziel: ({goal_x:.2f}, {goal_y:.2f})")

            if self.simple_go_to(self.current_arc_goal, tolerance=GOAL_REACHED_TOLERANCE):
                self.get_logger().info("Polygon-Vertex erreicht.")
                self.current_arc_goal = None
            return
            
        # --- 6. Zur Hütte ausrichten ---
        if self.state == DockingState.ALIGN_TO_HUT:
            # (Diese Logik bleibt unverändert)
            target_theta = math.atan2(self.WAYPOINT_INSIDE[1] - current_pos_vec[1], self.WAYPOINT_INSIDE[0] - current_pos_vec[0])
            angle_err = (target_theta - current_theta + math.pi) % (2 * math.pi) - math.pi
            
            if abs(angle_err) < 0.05:  # ~3 Grad
                self.get_logger().info("✓ Schritt 5: Grob zur Hütte ausgerichtet. Starte Endausrichtung.")
                self.change_state(DockingState.FINAL_ALIGNMENT)
                self.publish_twist(0.0, 0.0)
            else:
                angular_vel = np.clip(1.5 * angle_err, -0.5, 0.5)
                self.publish_twist(0.0, angular_vel)
            return
            
        # ==========================================================
        # --- 7. Endgültige Ausrichtung (PIXEL-BASIERT) ---
        # ==========================================================
        if self.state == DockingState.FINAL_ALIGNMENT:
            
            # --- A: Führe 10cm-Nudge aus, falls aktiv ---
            if self._align_nudge_active:
                now = self.get_clock().now()
                time_since_nudge_start = (now - self._align_nudge_start_time).nanoseconds / 1e9
                
                if time_since_nudge_start < self.NUDGE_DURATION: 
                    self.publish_twist(self.NUDGE_SPEED, 0.0) # Langsam vor
                    return # Warten, bis Manöver fertig ist
                else:
                    self.publish_twist(0.0, 0.0) # Stopp
                    self.get_logger().info("10cm Vorfahrt (Nudge) beendet. Versuche Ausrichtung erneut.")
                    self._align_nudge_active = False # Manöver beendet
            
            # --- B: Prüfe, ob Tag sichtbar ist ---
            detection_age_sec = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
            tag_visible = self.last_target_detection is not None and detection_age_sec < 1.0 # 1 Sekunde Toleranz
            
            if tag_visible:
                # --- C: Tag sichtbar -> Ausrichten ---
                
                # Berechne Pixel-Fehler (nur horizontal)
                pixel_error = self.IMAGE_CENTER_X - self.last_target_detection.centre.x
                
                if abs(pixel_error) <= self.PIXEL_TOLERANCE:
                    # Ziel-Ausrichtung erreicht
                    self.get_logger().info(f"✓ Pixel-Ausrichtung OK (Fehler: {pixel_error:.1f}px). Wechsle zu DOCKING.")
                    self.publish_twist(0.0, 0.0)
                    self.change_state(DockingState.DOCKING)
                else:
                    # P-Regler für Drehung basierend auf Pixel-Fehler
                    angular_vel = np.clip(self.K_P_PIXEL * pixel_error, 
                                          -self.DOCKING_SPEED_ANGULAR, 
                                          self.DOCKING_SPEED_ANGULAR)
                    
                    self.get_logger().info(f"Richte Kamera auf Pixel... Fehler: {pixel_error:.1f}px, Vel: {angular_vel:.2f} rad/s", throttle_duration_sec=2.0)
                    self.publish_twist(0.0, angular_vel)

            else:
                # --- D: Tag nicht sichtbar -> 10cm Nudge (wie gewünscht) ---
                if not self._align_nudge_active: # Starte Nudge nur, wenn nicht schon aktiv
                    self.get_logger().warn(f"Tag {self.DOCKING_TARGET_ID} in FINAL_ALIGNMENT nicht sichtbar. Starte 10cm Nudge.")
                    self._align_nudge_active = True
                    self._align_nudge_start_time = self.get_clock().now()
                    self.publish_twist(0.0, 0.0) # Stoppe evtl. Drehung
            
            return
            
        # --- 8. In die Hütte fahren (10cm-Schritte + Distanz-Check) ---
        if self.state == DockingState.DOCKING:
            
            # --- A: Führe 10cm-Fahrt aus, falls aktiv ---
            if self._docking_drive_active:
                now = self.get_clock().now()
                time_since_drive_start = (now - self._docking_drive_start_time).nanoseconds / 1e9
                
                if time_since_drive_start < self.NUDGE_DURATION: 
                    self.publish_twist(self.NUDGE_SPEED, 0.0) # Langsam vor
                    return # Warten, bis Manöver fertig ist
                else:
                    self.publish_twist(0.0, 0.0) # Stopp
                    self.get_logger().info("10cm Fahrt beendet. Gehe zurück zu FINAL_ALIGNMENT.")
                    self._docking_drive_active = False # Manöver beendet
                    self.change_state(DockingState.FINAL_ALIGNMENT) # Zurück zur Ausrichtung
                    return
            
            # --- B: Nicht am Fahren? Prüfe Distanz (mit TF) & starte ggf. Fahrt ---
            try:
                # Die Distanzprüfung MUSS TF verwenden, da Pixelgröße kein
                # zuverlässiges Distanzmaß ist.
                t = self.tf_buffer.lookup_transform(
                    'robot/chassis', # Physischer Abstand vom Chassis
                    self.DOCKING_TARGET_FRAME,
                    rclpy.time.Time()
                )
                trans = t.transform.translation
                # Distanzfehler (vor/zurück)
                dist_err = trans.x - self.DOCKING_TARGET_DISTANCE
                
                # Stopp-Bedingung: Zielabstand erreicht (10cm Toleranz)
                if abs(dist_err) < 0.1:
                    self.get_logger().info("✓✓✓ DOCKING ERFOLGREICH (Distanz-Check)! ✓✓✓")
                    self.change_state(DockingState.FINAL_STOP)
                    self.publish_twist(0.0, 0.0)
                    return

                # Distanz noch zu groß: Starte 10cm Fahrt
                # (Wir wissen aus State 7, dass die Pixel-Ausrichtung < 10px war)
                self.get_logger().info(f"Distanz {trans.x:.2f}m (noch > {self.DOCKING_TARGET_DISTANCE:.1f}m). Starte 10cm Vorfahrt.")
                self._docking_drive_active = True
                self._docking_drive_start_time = self.get_clock().now()

            except TransformException as e:
                self.get_logger().error(f"Tag '{self.DOCKING_TARGET_FRAME}' für Distanz-Check verloren! Gehe zurück zu FINAL_ALIGNMENT. Fehler: {e}")
                self.publish_twist(0.0, 0.0)
                self.change_state(DockingState.FINAL_ALIGNMENT)
            
            return
            
        # --- 9. Ziel erreicht ---
        if self.state == DockingState.FINAL_STOP:
            self.publish_twist(0.0, 0.0)
            
            if self.current_odom_pose is not None:
                current_pos = np.array(self.current_odom_pose[:2])
                dist_to_hut = np.linalg.norm(current_pos - self.HUT_CENTER)
                
                # Wenn wir plötzlich weit weg sind (> 3m), starte neu!
                if dist_to_hut > 3.0:
                    self.get_logger().info("Reset erkannt! Starte Docking-Sequenz neu.")
                    self.change_state(DockingState.SEARCHING)
            return

    def simple_go_to(self, target_pos: np.ndarray, tolerance: float) -> bool:
        """Einfacher P-Regler zum Anfahren eines Zielpunkts (bleibt unverändert)"""
        if self.current_odom_pose is None:
            return False

        K_linear = 1.0
        K_angular = 2.0
        max_linear = 1.5
        max_angular = 1.0

        current_pos = np.array(self.current_odom_pose[:2])
        current_theta = self.current_odom_pose[2]
        
        dist_err = np.linalg.norm(target_pos - current_pos)
        
        if dist_err < tolerance:
            self.publish_twist(0.0, 0.0)
            return True

        angle_to_target = math.atan2(
            target_pos[1] - current_pos[1],
            target_pos[0] - current_pos[0]
        )
        
        angle_err = (angle_to_target - current_theta + math.pi) % (2 * math.pi) - math.pi
        
        if abs(angle_err) > 0.3:  # ~17 Grad
            linear_vel = 0.0
            angular_vel = np.clip(K_angular * angle_err, -max_angular, max_angular)
        else:
            linear_vel = np.clip(K_linear * dist_err, 0.0, max_linear)
            angular_vel = np.clip(K_angular * angle_err, -max_angular, max_angular)
            linear_vel *= (1.0 - abs(angle_err) / math.pi)

        self.publish_twist(linear_vel, angular_vel)
        return False

    def publish_twist(self, linear: float, angular: float):
        """Hilfsfunktion zum Publizieren von Twist-Nachrichten"""
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_twist(0.0, 0.0) # Sicherstellen, dass der Roboter stoppt
        node.get_logger().info("Docking Controller wird beendet.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
