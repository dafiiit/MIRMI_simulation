#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class DockingState(Enum):
    SEARCHING = 1          # Dreht sich, bis Tag gesehen wird
    LOCALIZING = 2         # Bestimmt Welt-Position
    GOTO_WAYPOINT_1 = 3    # Fährt auf 5m Abstand
    GOTO_WAYPOINT_2 = 4    # Fährt zur Öffnung
    ALIGN_TO_HUT = 5       # Dreht sich zur Hütte
    DOCKING = 6            # Fährt in die Hütte
    FINAL_STOP = 7         # Ziel erreicht


class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')
        
        # Zustand der State Machine
        self.state = DockingState.SEARCHING
        self.state_entry_time = self.get_clock().now()
        
        # Konstanten für die Navigation
        self.HUT_CENTER = np.array([5.0, 2.0])
        self.TARGET_RADIUS = 5.0
        self.WAYPOINT_OPENING = np.array([6.5, 2.0])  # Vor der Öffnung
        self.WAYPOINT_INSIDE = np.array([5.5, 2.0])   # In der Hütte
        
        # Aktuelle Roboter-Pose (x, y, theta) aus Odometrie
        self.current_odom_pose = None
        
        # Flag ob wir bereits lokalisiert haben
        self.has_localized = False
        self.localization_count = 0
        self.last_detection_time = None
        self.detection_count = 0
        
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
        
        # Publisher & Subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )
        
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
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("DOCKING CONTROLLER V2 GESTARTET")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Initial State: {self.state.name}")
        self.get_logger().info("Warte auf Odometrie und AprilTag-Detektionen...")

    def change_state(self, new_state: DockingState):
        """Hilfsfunktion für Zustandsübergänge mit Logging"""
        old_state = self.state
        self.state = new_state
        self.state_entry_time = self.get_clock().now()
        
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
        
        # Odometrie-Status
        if self.current_odom_pose is not None:
            x, y, theta = self.current_odom_pose
            status_lines.append(f"Odometrie: x={x:.2f}, y={y:.2f}, theta={math.degrees(theta):.1f}°")
        else:
            status_lines.append("Odometrie: KEINE DATEN ❌")
        
        # Detection-Status
        if self.last_detection_time is not None:
            time_since = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
            status_lines.append(f"Detections: {self.detection_count} empfangen, letzte vor {time_since:.1f}s")
        else:
            status_lines.append("Detections: KEINE EMPFANGEN ❌")
        
        # Lokalisierungs-Status
        if self.has_localized:
            status_lines.append(f"Lokalisierung: ERFOLGT ✓ (Count: {self.localization_count})")
        else:
            status_lines.append(f"Lokalisierung: AUSSTEHEND (Attempts: {self.localization_count})")
        
        # Zustandsspezifische Infos
        if self.state == DockingState.SEARCHING:
            status_lines.append("Aktion: Rotiere, suche AprilTags...")
        elif self.state == DockingState.LOCALIZING:
            status_lines.append("Aktion: Warte auf erfolgreiche Lokalisierung...")
        elif self.state.value >= DockingState.GOTO_WAYPOINT_1.value and self.current_odom_pose:
            current_pos = np.array(self.current_odom_pose[:2])
            if self.state == DockingState.GOTO_WAYPOINT_1:
                dist = np.linalg.norm(self.HUT_CENTER - current_pos)
                status_lines.append(f"Aktion: Fahre auf 5m Radius (aktuell: {dist:.2f}m)")
            elif self.state == DockingState.GOTO_WAYPOINT_2:
                dist = np.linalg.norm(self.WAYPOINT_OPENING - current_pos)
                status_lines.append(f"Aktion: Fahre zur Öffnung (Distanz: {dist:.2f}m)")
            elif self.state == DockingState.ALIGN_TO_HUT:
                status_lines.append("Aktion: Richte zur Hütte aus...")
            elif self.state == DockingState.DOCKING:
                dist = np.linalg.norm(self.WAYPOINT_INSIDE - current_pos)
                status_lines.append(f"Aktion: Docke ein (Distanz: {dist:.2f}m)")
        
        status_lines.append("=" * 60)
        
        for line in status_lines:
            self.get_logger().info(line)

    def odom_callback(self, msg: Odometry):
        """Speichert die aktuelle Odometrie-Pose"""
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        
        # Quaternion zu Euler
        _, _, yaw = euler_from_quaternion([
            orient.x, orient.y, orient.z, orient.w
        ])
        
        was_none = self.current_odom_pose is None
        self.current_odom_pose = (pos.x, pos.y, yaw)
        
        if was_none:
            self.get_logger().info(f"✓ Erste Odometrie empfangen: ({pos.x:.2f}, {pos.y:.2f})")

    def detection_callback(self, msg: AprilTagDetectionArray):
        """Wird aufgerufen, wenn AprilTags erkannt werden"""
        self.last_detection_time = self.get_clock().now()
        self.detection_count += 1
        
        num_detections = len(msg.detections)
        
        if num_detections > 0:
            tag_ids = [d.id for d in msg.detections]
            
            # Log nur bei ersten paar Detections oder bei Zustandswechsel
            if self.detection_count <= 5 or self.detection_count % 20 == 0:
                self.get_logger().info(
                    f"📷 Detection #{self.detection_count}: "
                    f"{num_detections} Tag(s) gefunden - IDs: {tag_ids}"
                )
        
        # State-spezifische Reaktionen
        if self.state == DockingState.SEARCHING and num_detections > 0:
            self.get_logger().info(f"✓ Tag(s) gefunden während SEARCHING: {[d.id for d in msg.detections]}")
            self.publish_twist(0.0, 0.0)
            self.change_state(DockingState.LOCALIZING)
            
        # Versuche Lokalisierung während SEARCHING und LOCALIZING
        if self.state in [DockingState.SEARCHING, DockingState.LOCALIZING]:
            if num_detections > 0:
                self.run_localization(msg)

    def run_localization(self, msg: AprilTagDetectionArray):
        """Versucht, die Roboter-Welt-Pose zu berechnen"""
        
        if self.current_odom_pose is None:
            if self.localization_count % 10 == 0:
                self.get_logger().warn("⚠ Keine Odometrie verfügbar für Lokalisierung")
            return
        
        try:
            # Debug: Zeige Struktur der Detection beim ersten Mal
            if len(msg.detections) > 0 and not hasattr(self, '_structure_logged'):
                det = msg.detections[0]
                self.get_logger().info("🔍 Detection Message Struktur:")
                self.get_logger().info(f"  - Type: {type(det)}")
                self.get_logger().info(f"  - Has 'pose': {hasattr(det, 'pose')}")
                if hasattr(det, 'pose'):
                    self.get_logger().info(f"  - Pose Type: {type(det.pose)}")
                    self.get_logger().info(f"  - Pose fields: {[f for f in dir(det.pose) if not f.startswith('_')]}")
                self._structure_logged = True
            
            # Nimm die erste Detektion mit gültiger Pose
            detection = None
            for det in msg.detections:
                # Prüfe verschiedene mögliche Pose-Strukturen
                pose_valid = False
                
                if hasattr(det, 'pose'):
                    # Verschiedene apriltag_ros Versionen haben unterschiedliche Strukturen
                    if hasattr(det.pose, 'pose'):
                        if hasattr(det.pose.pose, 'pose'):
                            # pose.pose.pose Struktur
                            pose_valid = det.pose.pose.pose.position.z > 0
                        else:
                            # pose.pose Struktur
                            pose_valid = det.pose.pose.position.z > 0
                    else:
                        # direkte pose Struktur
                        if hasattr(det.pose, 'position'):
                            pose_valid = det.pose.position.z > 0
                
                if pose_valid:
                    detection = det
                    self.get_logger().info(f"✓ Pose gefunden in Detection von Tag {det.id}")
                    break
            
            if detection is None:
                # Fallback: Nutze TF für Lokalisierung
                if self.localization_count % 20 == 0:
                    self.get_logger().info("ℹ Keine Pose in Detection, versuche TF-Lookup...")
                self.run_localization_tf(msg)
                return
            
            tag_id = detection.id
            
            # Bekannte Tag-Positionen in der Welt (aus static transforms)
            tag_positions = {
                0: (3.0, 0.0, 0.5),
                1: (3.601, 2.0, 0.75),
                2: (3.499, 2.0, 0.75),
                3: (5.0, 3.101, 0.75),
                4: (5.0, 0.899, 0.75),
            }
            
            if tag_id not in tag_positions:
                self.get_logger().warn(f"⚠ Unbekannter Tag ID: {tag_id}")
                return
            
            # Tag-Position in der Welt
            tag_world_pos = np.array(tag_positions[tag_id][:2])
            
            # Tag-Position relativ zur Kamera (aus der Detection)
            # Extrahiere Pose abhängig von der Struktur
            if hasattr(detection.pose, 'pose'):
                if hasattr(detection.pose.pose, 'pose'):
                    pose = detection.pose.pose.pose
                else:
                    pose = detection.pose.pose
            else:
                pose = detection.pose
            
            tag_cam_x = pose.position.x
            tag_cam_y = pose.position.y
            tag_cam_z = pose.position.z
            
            # Kamera-Offset vom Roboter-Zentrum (aus model.sdf: 1.0m vorne)
            # Die Kamera zeigt nach vorne (in x-Richtung des Roboters)
            camera_offset = 1.0
            
            # Aktuelle Roboter-Orientierung aus Odometrie
            robot_x, robot_y, robot_theta = self.current_odom_pose
            
            # Transformation: Tag in Kamera-Koordinaten -> Tag in Roboter-Koordinaten
            # Die Kamera zeigt in +x Richtung, also:
            # - Kamera +x -> Roboter +x
            # - Kamera +y -> Roboter -z (links)
            # - Kamera +z -> Roboter +y (nach vorne zum Tag)
            
            # Tag-Position im Roboter-Frame (vereinfachte Annahme: flache Welt)
            tag_robot_distance = math.sqrt(tag_cam_x**2 + tag_cam_z**2)
            tag_robot_angle = math.atan2(-tag_cam_y, tag_cam_z)
            
            # Winkel des Tags in Welt-Koordinaten
            tag_world_angle = robot_theta + tag_robot_angle
            
            # Roboter-Position berechnen
            # Der Roboter ist "tag_robot_distance" vom Tag entfernt, in Richtung "tag_world_angle + pi"
            estimated_robot_x = tag_world_pos[0] - tag_robot_distance * math.cos(tag_world_angle)
            estimated_robot_y = tag_world_pos[1] - tag_robot_distance * math.sin(tag_world_angle)
            
            self.localization_count += 1
            
            if self.localization_count <= 3 or self.localization_count % 10 == 1:
                self.get_logger().info(
                    f"📍 Lokalisierung #{self.localization_count}: "
                    f"Robot @ ({estimated_robot_x:.2f}, {estimated_robot_y:.2f}), "
                    f"theta={math.degrees(robot_theta):.1f}°, "
                    f"Tag {tag_id} @ {tag_robot_distance:.2f}m"
                )
            
            if not self.has_localized:
                self.has_localized = True
                self.get_logger().info("✓✓✓ ERSTE ERFOLGREICHE LOKALISIERUNG! ✓✓✓")
                self.change_state(DockingState.GOTO_WAYPOINT_1)
                
        except Exception as e:
            self.get_logger().error(f"❌ Lokalisierung fehlgeschlagen: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def run_localization_tf(self, msg: AprilTagDetectionArray):
        """Alternative Lokalisierung mittels TF-Transforms"""
        try:
            if len(msg.detections) == 0:
                return
            
            detection = msg.detections[0]
            tag_id = detection.id
            tag_frame = f"tag36_11_{tag_id:05d}"
            
            # Bekannte Tag-Positionen in der Welt
            tag_positions = {
                0: (3.0, 0.0, 0.5),
                1: (3.601, 2.0, 0.75),
                2: (3.499, 2.0, 0.75),
                3: (5.0, 3.101, 0.75),
                4: (5.0, 0.899, 0.75),
            }
            
            if tag_id not in tag_positions:
                return
            
            # Versuche TF-Lookup mit timeout
            timeout = rclpy.time.Duration(seconds=0.1)
            now = rclpy.time.Time()
            
            try:
                # Hole Transform von camera zu tag
                tf_cam_tag = self.tf_buffer.lookup_transform(
                    'robot/chassis/camera_sensor',
                    tag_frame,
                    now,
                    timeout=timeout
                )
                
                # Berechne Distanz und Winkel aus Transform
                tx = tf_cam_tag.transform.translation.x
                ty = tf_cam_tag.transform.translation.y
                tz = tf_cam_tag.transform.translation.z
                
                tag_distance = math.sqrt(tx**2 + ty**2 + tz**2)
                tag_angle = math.atan2(-ty, tz)
                
                # Roboter Pose aus Odometrie
                robot_x, robot_y, robot_theta = self.current_odom_pose
                
                # Tag-Position in Welt
                tag_world_pos = np.array(tag_positions[tag_id][:2])
                
                # Winkel des Tags in Welt-Koordinaten
                tag_world_angle = robot_theta + tag_angle
                
                # Roboter-Position berechnen
                estimated_robot_x = tag_world_pos[0] - tag_distance * math.cos(tag_world_angle)
                estimated_robot_y = tag_world_pos[1] - tag_distance * math.sin(tag_world_angle)
                
                self.localization_count += 1
                
                if self.localization_count <= 3 or self.localization_count % 10 == 1:
                    self.get_logger().info(
                        f"📍 TF-Lokalisierung #{self.localization_count}: "
                        f"Robot @ ({estimated_robot_x:.2f}, {estimated_robot_y:.2f}), "
                        f"theta={math.degrees(robot_theta):.1f}°, "
                        f"Tag {tag_id} @ {tag_distance:.2f}m"
                    )
                
                if not self.has_localized:
                    self.has_localized = True
                    self.get_logger().info("✓✓✓ ERSTE ERFOLGREICHE LOKALISIERUNG (via TF)! ✓✓✓")
                    self.change_state(DockingState.GOTO_WAYPOINT_1)
                    
            except TransformException as tf_ex:
                # TF noch nicht verfügbar
                if self.localization_count % 30 == 0:
                    self.get_logger().debug(f"TF nicht verfügbar: {tf_ex}")
                pass
                
        except Exception as e:
            if self.localization_count % 30 == 0:
                self.get_logger().debug(f"TF-Lokalisierung fehlgeschlagen: {e}")

    def control_loop(self):
        """Haupt-Steuerungslogik"""
        
        if self.current_odom_pose is None:
            return
        
        # --- Schritt 1: Suchen ---
        if self.state == DockingState.SEARCHING:
            self.publish_twist(0.0, 0.4)  # Langsam drehen
            return
        
        # --- Schritt 2: Lokalisieren ---
        if self.state == DockingState.LOCALIZING:
            # Warten auf erfolgreiche Lokalisierung
            if self.has_localized:
                self.get_logger().info("✓ Lokalisierung abgeschlossen, fahre zu Waypoint 1")
                self.change_state(DockingState.GOTO_WAYPOINT_1)
            else:
                # Weiter langsam drehen für bessere Sicht
                self.publish_twist(0.0, 0.2)
            return
        
        # --- Schritt 3: Auf 5m Abstand fahren ---
        if self.state == DockingState.GOTO_WAYPOINT_1:
            current_pos = np.array(self.current_odom_pose[:2])
            vec_to_center = self.HUT_CENTER - current_pos
            dist_to_center = np.linalg.norm(vec_to_center)
            
            if dist_to_center > 0.1:
                # Berechne Zielpunkt auf 5m Radius
                target_point = self.HUT_CENTER - (vec_to_center / dist_to_center) * self.TARGET_RADIUS
                
                if self.simple_go_to(target_point, tolerance=0.3):
                    self.get_logger().info("✓ Waypoint 1 erreicht (5m Radius)")
                    self.change_state(DockingState.GOTO_WAYPOINT_2)
            return
        
        # --- Schritt 4: Zur Öffnung fahren ---
        if self.state == DockingState.GOTO_WAYPOINT_2:
            if self.simple_go_to(self.WAYPOINT_OPENING, tolerance=0.2):
                self.get_logger().info("✓ Waypoint 2 erreicht (vor Öffnung)")
                self.change_state(DockingState.ALIGN_TO_HUT)
            return
        
        # --- Schritt 5: Zur Hütte ausrichten ---
        if self.state == DockingState.ALIGN_TO_HUT:
            current_theta = self.current_odom_pose[2]
            target_theta = math.atan2(
                self.WAYPOINT_INSIDE[1] - self.current_odom_pose[1],
                self.WAYPOINT_INSIDE[0] - self.current_odom_pose[0]
            )
            
            angle_err = target_theta - current_theta
            angle_err = (angle_err + math.pi) % (2 * math.pi) - math.pi
            
            if abs(angle_err) < 0.1:  # ~6 Grad
                self.get_logger().info("✓ Zur Hütte ausgerichtet")
                self.change_state(DockingState.DOCKING)
                self.publish_twist(0.0, 0.0)
            else:
                angular_vel = np.clip(1.5 * angle_err, -0.5, 0.5)
                self.publish_twist(0.0, angular_vel)
            return
        
        # --- Schritt 6: In die Hütte fahren ---
        if self.state == DockingState.DOCKING:
            if self.simple_go_to(self.WAYPOINT_INSIDE, tolerance=0.15):
                self.get_logger().info("✓✓✓ DOCKING ERFOLGREICH! ✓✓✓")
                self.change_state(DockingState.FINAL_STOP)
            return
        
        # --- Ziel erreicht ---
        if self.state == DockingState.FINAL_STOP:
            self.publish_twist(0.0, 0.0)
            self.control_timer.cancel()
            self.debug_timer.cancel()
            return

    def simple_go_to(self, target_pos: np.ndarray, tolerance: float) -> bool:
        """
        Einfacher P-Regler zum Anfahren eines Zielpunkts
        Returns: True wenn Ziel erreicht
        """
        if self.current_odom_pose is None:
            return False

        # Parameter
        K_linear = 0.5
        K_angular = 2.0
        max_linear = 0.5
        max_angular = 1.0

        current_pos = np.array(self.current_odom_pose[:2])
        current_theta = self.current_odom_pose[2]
        
        # Abstand zum Ziel
        dist_err = np.linalg.norm(target_pos - current_pos)
        
        if dist_err < tolerance:
            self.publish_twist(0.0, 0.0)
            return True

        # Winkel zum Ziel
        angle_to_target = math.atan2(
            target_pos[1] - current_pos[1],
            target_pos[0] - current_pos[0]
        )
        
        angle_err = angle_to_target - current_theta
        # Normalisieren auf [-pi, pi]
        angle_err = (angle_err + math.pi) % (2 * math.pi) - math.pi
        
        # Wenn stark falsch ausgerichtet: nur drehen
        if abs(angle_err) > 0.3:  # ~17 Grad
            linear_vel = 0.0
            angular_vel = np.clip(K_angular * angle_err, -max_angular, max_angular)
        else:
            # Fahren und korrigieren
            linear_vel = np.clip(K_linear * dist_err, 0.0, max_linear)
            angular_vel = np.clip(K_angular * angle_err, -max_angular, max_angular)
            
            # Geschwindigkeit reduzieren bei großem Winkelfehler
            linear_vel *= (1.0 - abs(angle_err) / math.pi)

        self.publish_twist(linear_vel, angular_vel)
        
        # Debug-Ausgabe alle 5 Sekunden
        if not hasattr(self, '_last_nav_debug_time'):
            self._last_nav_debug_time = 0.0
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self._last_nav_debug_time > 5.0:
            self._last_nav_debug_time = current_time
            self.get_logger().info(
                f"🎯 Navigation: Dist={dist_err:.2f}m, "
                f"Angle={math.degrees(angle_err):.1f}°, "
                f"Vel=(lin={linear_vel:.2f}, ang={angular_vel:.2f})"
            )
        
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
        node.publish_twist(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
