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
from std_msgs.msg import String


class DockingState(Enum):
    SEARCHING = 1          # 1. Dreht sich, bis Tag gesehen wird
    LOCALIZING = 2         # 2. Bestimmt Welt-Position
    GOTO_RADIUS_POINT = 3  # 3. Fährt ZU einem Punkt auf dem 10m-Radius
    FOLLOW_ARC = 4         # 5. Fährt auf dem 10m-Radius bis zur Öffnung
    ALIGN_TO_HUT = 5       # 6. Dreht sich zur Öffnung
    FINAL_ALIGNMENT = 6    # 7. Steht still, dreht sich exakt auf Tag 1
    DOCKING = 7            # 7. Fährt in die Hütte
    FINAL_STOP = 8         # 8. Ziel erreicht


class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')
        
        # Zustand der State Machine
        self.state = DockingState.SEARCHING
        self.state_entry_time = self.get_clock().now()
        
        # Konstanten für die Navigation
        self.HUT_CENTER = np.array([5.0, 2.0])
        self.TARGET_RADIUS = 5.5      # Dein Wunsch: 10 Meter
        self.RADIUS_TOLERANCE = 0.15    # Dein Wunsch: +-10cm
        self.ARC_SPEED = 0.4           # Geschwindigkeit beim Fahren des Bogens (m/s) (wird nicht mehr genutzt)
        self.K_P_RADIUS = 0.7          # Verstärkung für Radius-Korrektur (wird nicht mehr genutzt)
        
        # Wegpunkte (relativ zur Welt)
        self.WAYPOINT_OPENING = np.array([6.5, 2.0])  # Vor der Öffnung (für Ausrichtung)
        self.WAYPOINT_INSIDE = np.array([5.5, 2.0])   # In der Hütte (Endziel)
        
        # Ziel-Tag für die Endausrichtung
        self.DOCKING_TARGET_FRAME = 'tag36_11_00001' # Das Tag INNEN an der Rückwand
        self.DOCKING_TARGET_DISTANCE = 1.9 # Ziel-Abstand zum Tag (Robot-Origin bei 5.5, Tag bei 3.6 -> 1.9m)
        self.DOCKING_SPEED_LINEAR = 0.3    # Langsame, präzise Fahrt
        self.DOCKING_SPEED_ANGULAR = 1.5   # Maximale Dreh-Korrektur
        
        # Aktuelle Roboter-Pose (x, y, theta) aus Odometrie
        self.current_odom_pose = None
        
        # Flag ob wir bereits lokalisiert haben
        self.has_localized = False
        self.localization_count = 0
        self.last_detection_time = None
        self.detection_count = 0
        
        # Variablen für die Kreis-Logik
        self.arc_direction = 1.0  # 1.0 für CCW (gegen UZS), -1.0 für CW (im UZS)
        self.tangent_goal_angle = 0.0
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
        
        #Publisher für den Robot state
        self.state_pub = self.create_publisher(
            String,
            '/docking_controller/state',
            state_qos_profile  # Nutzt das latching Profil
        )
        
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
        self.get_logger().info("DOCKING CONTROLLER (8-PUNKTE-PLAN) GESTARTET")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Initial State: {self.state.name}")
        self.get_logger().info(f"Ziel-Radius: {self.TARGET_RADIUS}m")
        self.get_logger().info("Warte auf Odometrie und AprilTag-Detektionen...")
        
        initial_state_msg = String()
        initial_state_msg.data = self.state.name
        self.state_pub.publish(initial_state_msg)

    def change_state(self, new_state: DockingState):
        """Hilfsfunktion für Zustandsübergänge mit Logging"""
        old_state = self.state
        self.state = new_state
        self.state_entry_time = self.get_clock().now()
        
        state_msg = String()
        state_msg.data = self.state.name  # z.B. "SEARCHING"
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
        
        # Odometrie-Status
        if self.current_odom_pose is not None:
            x, y, theta = self.current_odom_pose
            status_lines.append(f"Odometrie: x={x:.2f}, y={y:.2f}, theta={math.degrees(theta):.1f}°")
        else:
            status_lines.append("Odometrie: KEINE DATEN ❌")
        
        # Lokalisierungs-Status
        status_lines.append(f"Lokalisierung: {'ERFOLGT ✓' if self.has_localized else 'AUSSTEHEND'}")
        
        # Zustandsspezifische Infos
        if self.state == DockingState.SEARCHING:
            status_lines.append("Aktion: Rotiere, suche AprilTags...")
        elif self.state == DockingState.GOTO_RADIUS_POINT and self.current_odom_pose:
            current_pos = np.array(self.current_odom_pose[:2])
            dist = np.linalg.norm(self.HUT_CENTER - current_pos)
            status_lines.append(f"Aktion: Fahre auf {self.TARGET_RADIUS}m Radius (aktuell: {dist:.2f}m)")
        elif self.state == DockingState.FOLLOW_ARC and self.current_odom_pose:
            current_pos = np.array(self.current_odom_pose[:2])
            dist = np.linalg.norm(self.HUT_CENTER - current_pos)
            status_lines.append(f"Aktion: Folge Kreis (Dist: {dist:.2f}m, Soll: {self.TARGET_RADIUS}m, Dir: {'CCW' if self.arc_direction > 0 else 'CW'})")
        elif self.state == DockingState.ALIGN_TO_HUT:
            status_lines.append("Aktion: Richte zur Hütte aus...")
        elif self.state == DockingState.DOCKING:
            status_lines.append("Aktion: Docke ein...")

        
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
        
        # State-spezifische Reaktionen
        if self.state == DockingState.SEARCHING and num_detections > 0:
            tag_ids = [d.id for d in msg.detections]
            self.get_logger().info(f"✓ Tag(s) {tag_ids} gefunden während SEARCHING")
            self.publish_twist(0.0, 0.0)
            self.change_state(DockingState.LOCALIZING)
            
        # Versuche Lokalisierung während SEARCHING und LOCALIZING
        if self.state in [DockingState.SEARCHING, DockingState.LOCALIZING]:
            if num_detections > 0:
                self.run_localization(msg)

    def run_localization(self, msg: AprilTagDetectionArray):
        """Versucht, die Roboter-Welt-Pose zu berechnen (Stark vereinfacht)"""
        
        # Diese Funktion sollte eine robuste Pose liefern.
        # Hier simulieren wir, dass es funktioniert, sobald Odometrie da ist.
        if self.current_odom_pose is None:
            return
            
        try:
            # (Hier käme die komplexe TF-Logik hin)
            # ...
            # Wir nehmen für dieses Beispiel an, dass die Odometrie
            # nach der ersten Sichtung "gut genug" ist.
            self.localization_count += 1
            
            if not self.has_localized:
                self.has_localized = True
                self.get_logger().info("✓✓✓ ERSTE ERFOLGREICHE LOKALISIERUNG! ✓✓✓")
                # Gehe zum nächsten Schritt
                self.change_state(DockingState.GOTO_RADIUS_POINT)
                
        except Exception as e:
            self.get_logger().error(f"❌ Lokalisierung fehlgeschlagen: {e}")
            
    # HINWEIS: run_localization_tf wurde der Einfachheit halber entfernt.
    # Die Logik in run_localization sollte bei Bedarf TF nutzen.

    def control_loop(self):
        """Haupt-Steuerungslogik"""
        
        if self.current_odom_pose is None:
            # Warten auf Odometrie
            if self.state.value > DockingState.LOCALIZING.value:
                self.get_logger().warn("Warte auf Odometrie...", throttle_skip_count=30)
            return
        
        current_pos_vec = np.array(self.current_odom_pose[:2])
        current_theta = self.current_odom_pose[2]
        
        # --- 1. Suchen ---
        if self.state == DockingState.SEARCHING:
            self.publish_twist(0.0, 0.3)  # Langsam drehen
            return
        
        # --- 2. Lokalisieren ---
        if self.state == DockingState.LOCALIZING:
            # Warten, bis run_localization() den Status auf has_localized setzt
            # und den Status auf GOTO_RADIUS_POINT ändert.
            # Drehe weiter, um bessere Sicht zu bekommen.
            self.publish_twist(0.0, 0.2)
            return
        
        # --- 3. Auf 10m Abstand fahren ---
        if self.state == DockingState.GOTO_RADIUS_POINT:
            vec_to_center = self.HUT_CENTER - current_pos_vec
            dist_to_center = np.linalg.norm(vec_to_center)
            
            if dist_to_center > 0.01:
                # Berechne Zielpunkt auf 10m Radius
                target_point = self.HUT_CENTER - (vec_to_center / dist_to_center) * self.TARGET_RADIUS
                
                if self.simple_go_to(target_point, tolerance=self.RADIUS_TOLERANCE):
                    self.get_logger().info(f"✓ {self.TARGET_RADIUS}m Radius erreicht. Berechne kürzeste Kreisbahn...")
                    
                    # 1. Winkel des Ziels (Öffnung)
                    opening_angle = math.atan2(
                        self.WAYPOINT_OPENING[1] - self.HUT_CENTER[1],
                        self.WAYPOINT_OPENING[0] - self.HUT_CENTER[0]
                    )
                    # 2. Aktueller Winkel des Roboters
                    current_angle_on_circle = math.atan2(
                        current_pos_vec[1] - self.HUT_CENTER[1],
                        current_pos_vec[0] - self.HUT_CENTER[0]
                    )
                    # 3. Differenzwinkel (kürzester Weg)
                    angle_diff_to_opening = opening_angle - current_angle_on_circle
                    angle_diff_to_opening = (angle_diff_to_opening + math.pi) % (2 * math.pi) - math.pi
                    
                    # 4. Drehrichtung setzen
                    if angle_diff_to_opening > 0:
                        self.arc_direction = 1.0  # CCW
                    else:
                        self.arc_direction = -1.0 # CW
                    
                    self.get_logger().info(f"✓ Kürzeste Richtung: {'CCW' if self.arc_direction > 0 else 'CW'}. Starte Kreisbahn (Schritt 4).")
                    
                    # 5. DIREKT ZU STATE 4 (FOLLOW_ARC) SPRINGEN
                    self.change_state(DockingState.FOLLOW_ARC)
            else:
                self.get_logger().warn("Roboter ist ZU NAH am Zentrum, fahre zurück.")
                target_point = current_pos_vec - vec_to_center * (self.TARGET_RADIUS + 1.0)
                self.simple_go_to(target_point, tolerance=self.RADIUS_TOLERANCE)
            return
            
        # --- 5. Kreis fahren (als Polygon-Annäherung) ---
        if self.state == DockingState.FOLLOW_ARC:
            
            POLYGON_SEGMENT_LENGTH = 0.5  # 50cm pro Segment
            GOAL_REACHED_TOLERANCE = 0.15 # 15cm Toleranz pro Segment
            
            # --- 1. Stopp-Bedingung: Sind wir an der Öffnung? ---
            # (Diese Logik bleibt gleich)
            current_angle_on_circle = math.atan2(
                current_pos_vec[1] - self.HUT_CENTER[1],
                current_pos_vec[0] - self.HUT_CENTER[0]
            )
            opening_angle = math.atan2(
                self.WAYPOINT_OPENING[1] - self.HUT_CENTER[1],
                self.WAYPOINT_OPENING[0] - self.HUT_CENTER[0]
            )
            
            angle_err_to_goal = opening_angle - current_angle_on_circle
            angle_err_to_goal = (angle_err_to_goal + math.pi) % (2 * math.pi) - math.pi
            
            # Stoppen, wenn wir sehr nah am Zielwinkel sind
            if abs(angle_err_to_goal) < 0.01: # muss relativ akkurat sein
                self.get_logger().info("✓ Schritt 6: Kreisbahn beendet (nahe Öffnung).")
                self.change_state(DockingState.ALIGN_TO_HUT)
                self.publish_twist(0.0, 0.0)
                self.current_arc_goal = None # Ziel zurücksetzen
                return

            # --- 2. Ziel-Management: Haben wir ein Ziel oder brauchen wir ein neues? ---
            
            if self.current_arc_goal is None:
                # Wir haben kein Ziel (oder haben das letzte erreicht)
                # Berechne das NÄCHSTE Polygon-Vertex
                
                # Der Winkel eines Segments (s = r * theta -> theta = s / r)
                segment_angle_rad = POLYGON_SEGMENT_LENGTH / self.TARGET_RADIUS
                
                # Wende den Winkel in die korrekte Richtung an
                next_goal_angle = current_angle_on_circle + (self.arc_direction * segment_angle_rad)
                
                # Konvertiere den Winkel zurück in eine (x, y) Koordinate
                goal_x = self.HUT_CENTER[0] + self.TARGET_RADIUS * math.cos(next_goal_angle)
                goal_y = self.HUT_CENTER[1] + self.TARGET_RADIUS * math.sin(next_goal_angle)
                
                self.current_arc_goal = np.array([goal_x, goal_y])
                self.get_logger().info(f"Neues Polygon-Ziel: ({goal_x:.2f}, {goal_y:.2f})")

            # --- 3. Fahre zum aktuellen Ziel-Vertex ---
            
            # Nutze die simple_go_to Funktion, um zum Vertex zu fahren
            if self.simple_go_to(self.current_arc_goal, tolerance=GOAL_REACHED_TOLERANCE):
                # Wir haben das Vertex erreicht!
                self.get_logger().info("Polygon-Vertex erreicht.")
                # Setze das Ziel auf None, damit in der nächsten Iteration
                # das NÄCHSTE Ziel berechnet wird.
                self.current_arc_goal = None
                # simple_go_to hat bereits einen Stopp-Befehl (0,0) gesendet
            
            # Die simple_go_to() Funktion kümmert sich um das Senden der Twist-Befehle
            return
            
        # --- 6. Zur Hütte ausrichten ---
        if self.state == DockingState.ALIGN_TO_HUT:
            # Ziel ist der Eingang (oder leicht dahinter)
            target_theta = math.atan2(
                self.WAYPOINT_INSIDE[1] - current_pos_vec[1],
                self.WAYPOINT_INSIDE[0] - current_pos_vec[0]
            )
            
            angle_err = target_theta - current_theta
            angle_err = (angle_err + math.pi) % (2 * math.pi) - math.pi
            
            if abs(angle_err) < 0.05:  # ~3 Grad
                self.get_logger().info("✓ Schritt 7: Zur Hütte ausgerichtet.")
                self.change_state(DockingState.DOCKING)
                self.publish_twist(0.0, 0.0)
            else:
                angular_vel = np.clip(1.5 * angle_err, -0.5, 0.5)
                self.publish_twist(0.0, angular_vel)
            return
            
        # --- 7. Endgültige Ausrichtung (Nur Drehen) ---
        if self.state == DockingState.FINAL_ALIGNMENT:
            try:
                t = self.tf_buffer.lookup_transform(
                    'robot/chassis',
                    self.DOCKING_TARGET_FRAME,
                    rclpy.time.Time()
                )
                trans = t.transform.translation
                
                # Nur Winkelfehler berechnen
                angle_err = math.atan2(trans.y, trans.x)

                # Toleranz für "gerade": z.B. 1.5 Grad
                # (Du kannst 0.025 rad verkleinern, wenn es noch präziser sein muss)
                if abs(angle_err) < 0.001: # ~1.4 Grad
                    self.get_logger().info("✓ End-Ausrichtung abgeschlossen. Starte Einfahrt.")
                    self.publish_twist(0.0, 0.0)
                    self.change_state(DockingState.DOCKING) # Jetzt zu State 8 (DOCKING)
                else:
                    # P-Regler nur für Drehung
                    angular_vel = np.clip(2.5 * angle_err, -self.DOCKING_SPEED_ANGULAR, self.DOCKING_SPEED_ANGULAR)
                    self.get_logger().info(f"Richte aus... Winkelfehler: {math.degrees(angle_err):.2f}°", throttle_skip_count=5)
                    self.publish_twist(0.0, angular_vel) # WICHTIG: linear_vel = 0.0
            
            except TransformException as e:
                self.get_logger().error(f"Tag '{self.DOCKING_TARGET_FRAME}' während Ausrichtung verloren! Gehe zurück zu Suche.")
                self.change_state(DockingState.SEARCHING_FOR_DOCK_TAG) # Zurück zu State 6
            
            return
            
        # --- 8. In die Hütte fahren (Visual Servoing) ---
        if self.state == DockingState.DOCKING: # ALT: 7
            try:
                # ... (lookup_transform bleibt gleich)
                t = self.tf_buffer.lookup_transform(
                    'robot/chassis',
                    self.DOCKING_TARGET_FRAME,
                    rclpy.time.Time()
                )
                
                trans = t.transform.translation
                
                # 1. Distanzfehler (vor/zurück)
                dist_err = trans.x - self.DOCKING_TARGET_DISTANCE
                
                # 2. Winkelfehler (links/rechts)
                angle_err = math.atan2(trans.y, trans.x)
                
                # --- Regler ---
                
                # Stopp-Bedingung: Zielabstand erreicht
                if abs(dist_err) < 0.1:
                    self.get_logger().info("✓✓✓ SCHRITT 9: DOCKING ERFOLGREICH! ✓✓✓")
                    self.change_state(DockingState.FINAL_STOP) # Zu State 9
                    self.publish_twist(0.0, 0.0)
                    return

                # P-Regler für lineare Geschwindigkeit
                linear_vel = np.clip(0.8 * dist_err, -self.DOCKING_SPEED_LINEAR, self.DOCKING_SPEED_LINEAR)
                
                # P-Regler für Winkelgeschwindigkeit (für Feinkorrektur)
                angular_vel = np.clip(1.5 * angle_err, -self.DOCKING_SPEED_ANGULAR, self.DOCKING_SPEED_ANGULAR)

                # Diese Sperre (falls noch vorhanden) ist jetzt weniger kritisch,
                # da der Winkelfehler durch State 7 schon klein sein sollte.
                if abs(angle_err) > 0.3: # ~17 Grad
                    linear_vel = 0.0 # Stoppe Vorwärtsbewegung, wenn Ausrichtung stark abweicht
                    
                self.publish_twist(linear_vel, angular_vel)

            except TransformException as e:
                self.get_logger().error(f"Tag '{self.DOCKING_TARGET_FRAME}' während des Dockings verloren! Stoppe.")
                self.publish_twist(0.0, 0.0)
                # Optional: Hier könnte man einen Fehler-Zustand einleiten
            
            return
            
        # --- 9. Ziel erreicht ---
        if self.state == DockingState.FINAL_STOP: # ALT: 8
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
        K_linear = 1.0    # ALT: 0.5 oder 0.7 (Schnellere Beschleunigung)
        K_angular = 2.0
        max_linear = 1.5  # ALT: 0.5 oder 0.8 (Höhere Endgeschwindigkeit)
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
                f"🎯 Navigation (GoTo): Dist={dist_err:.2f}m, "
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
        node.publish_twist(0.0, 0.0) # Sicherstellen, dass der Roboter stoppt
        node.get_logger().info("Docking Controller wird beendet.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
