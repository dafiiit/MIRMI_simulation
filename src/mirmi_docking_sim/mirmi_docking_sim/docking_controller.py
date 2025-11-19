#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
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
        
        # Subscriber
        if self.use_ground_truth:
            self.get_logger().warn("!!! ACHTUNG: Nutze GROUND TRUTH (/ground_truth) !!!")
            self.gt_sub = self.create_subscription(Odometry, '/ground_truth', self.ground_truth_callback, qos_profile_sensor_data)  
        else:
            self.get_logger().info("Modus: Nutze Standard-Odometrie (/odom)")
            self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        
        # Constants
        self.HUT_CENTER = np.array([5.0, 2.0])
        self.TARGET_RADIUS = 5.5
        self.RADIUS_TOLERANCE = 0.20 
        self.WAYPOINT_OPENING = np.array([6.5, 2.0])
        self.WAYPOINT_INSIDE = np.array([5.5, 2.0])
        
        self.MAX_LINEAR_SPEED = 0.5
        self.MAX_ANGULAR_SPEED = 1.0
        self.NUDGE_SPEED = 0.15
        self.NUDGE_DURATION = 2.0
        
        self.DOCKING_TARGET_FRAME = 'tag36_11_00001'
        self.DOCKING_TARGET_ID = 1
        self.DOCKING_TARGET_DISTANCE = 1.9
        
        self.IMAGE_WIDTH = 640.0
        self.IMAGE_CENTER_X = self.IMAGE_WIDTH / 2.0
        self.PIXEL_TOLERANCE = 10.0
        self.K_P_PIXEL = 0.004 
        
        # Status Variablen
        self.current_odom_pose = None
        self.has_localized = False
        self.odom_received_once = False
        
        self.last_target_detection = None
        self.last_detection_time = self.get_clock().now()
        
        self.arc_direction = 1.0
        self.current_arc_goal = None
        
        self.state = DockingState.SEARCHING
        self.state_entry_time = self.get_clock().now()
        
        # NEU: Variablen für die 360 Grad Suche
        self.search_accumulated_yaw = 0.0
        self.last_yaw_search = None

        # TF & Tools
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/docking_controller/state', state_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray, '/detections', self.detection_callback, 10)
       
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self._align_nudge_active = False
        self._align_nudge_start_time = None
        self._docking_drive_active = False
        
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.set_initial_pose_callback, 10)
        
        self.get_logger().info("DOCKING CONTROLLER GESTARTET (Optimiert)")

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
        self.last_target_detection = None
        self.current_arc_goal = None
        self._align_nudge_active = False
        
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
        self._align_nudge_active = False
        
        # Reset search specific vars if we leave searching
        if new_state != DockingState.SEARCHING:
            self.search_accumulated_yaw = 0.0
            self.last_yaw_search = None
            
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)
        self.get_logger().info(f"STATE CHANGE: -> {new_state.name}")

    def detection_callback(self, msg: AprilTagDetectionArray):
        num_detections = len(msg.detections)
        
        if self.state == DockingState.SEARCHING and num_detections > 0:
            self.publish_twist(0.0, 0.0)
            self.get_logger().info("Tag gefunden! Wechsle zu LOCALIZING.")
            self.change_state(DockingState.LOCALIZING)
            
        if self.state in [DockingState.SEARCHING, DockingState.LOCALIZING] and num_detections > 0:
            self.run_localization(msg)
        
        if num_detections > 0:
            for detection in msg.detections:
                if detection.id == self.DOCKING_TARGET_ID:
                    self.last_target_detection = detection
                    self.last_detection_time = self.get_clock().now()

    def run_localization(self, msg: AprilTagDetectionArray):
        if self.current_odom_pose is None: return
        if not self.has_localized:
            self.has_localized = True
            self.change_state(DockingState.GOTO_RADIUS_POINT)

    def calculate_radius_entry_point(self, current_pos):
        vec_to_center = self.HUT_CENTER - current_pos
        dist_to_center = np.linalg.norm(vec_to_center)
        if dist_to_center < 0.01: return current_pos
        unit_vec = vec_to_center / dist_to_center
        target_point = self.HUT_CENTER - (unit_vec * self.TARGET_RADIUS)
        return target_point

    def control_loop(self):
        if self.current_odom_pose is None: 
            return
        
        cx, cy, ctheta = self.current_odom_pose
        current_pos_vec = np.array([cx, cy])
        
        self.log_debug(f"[{self.state.name}] Pose: x={cx:.1f}, y={cy:.1f}, th={ctheta:.2f}")
        
        # --- NEUE SUCHE LOGIK ---
        if self.state == DockingState.SEARCHING:
            # 1. Yaw Unterschied berechnen
            if self.last_yaw_search is not None:
                delta = abs(self.normalize_angle(ctheta - self.last_yaw_search))
                self.search_accumulated_yaw += delta
            
            self.last_yaw_search = ctheta
            
            # 2. Check ob 360 Grad (2*PI = 6.28) überschritten
            # Wir nehmen 6.4 als Puffer
            if self.search_accumulated_yaw > 6.4:
                self.get_logger().warn("TIMEOUT: 360 Grad gedreht, kein Tag gefunden.")
                self.publish_twist(0.0, 0.0)
                self.change_state(DockingState.SEARCH_TIMEOUT)
                return

            self.publish_twist(0.0, 0.3) # Drehen
            return
        
        if self.state == DockingState.SEARCH_TIMEOUT:
            self.publish_twist(0.0, 0.0)
            return
        
        if self.state == DockingState.LOCALIZING:
            self.publish_twist(0.0, 0.0)
            return
        
        if self.state == DockingState.GOTO_RADIUS_POINT:
            target_point = self.calculate_radius_entry_point(current_pos_vec)
            if self.simple_go_to(target_point, tolerance=self.RADIUS_TOLERANCE):
                opening_angle = math.atan2(self.WAYPOINT_OPENING[1] - self.HUT_CENTER[1], 
                                           self.WAYPOINT_OPENING[0] - self.HUT_CENTER[0])
                current_angle = math.atan2(cy - self.HUT_CENTER[1], cx - self.HUT_CENTER[0])
                angle_diff = self.normalize_angle(opening_angle - current_angle)
                self.arc_direction = 1.0 if angle_diff > 0 else -1.0
                self.change_state(DockingState.FOLLOW_ARC)
            return
            
        if self.state == DockingState.FOLLOW_ARC:
            current_angle = math.atan2(cy - self.HUT_CENTER[1], cx - self.HUT_CENTER[0])
            opening_angle = math.atan2(self.WAYPOINT_OPENING[1] - self.HUT_CENTER[1], 
                                       self.WAYPOINT_OPENING[0] - self.HUT_CENTER[0])
            angle_err = self.normalize_angle(opening_angle - current_angle)
            
            if abs(angle_err) < 0.1:
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
            target_theta = math.atan2(self.WAYPOINT_INSIDE[1] - cy, self.WAYPOINT_INSIDE[0] - cx)
            angle_err = self.normalize_angle(target_theta - ctheta)
            
            if abs(angle_err) < 0.05:
                self.change_state(DockingState.FINAL_ALIGNMENT)
                self.publish_twist(0.0, 0.0)
            else:
                ang_vel = np.clip(1.5 * angle_err, -0.5, 0.5)
                self.publish_twist(0.0, ang_vel)
            return
            
        if self.state == DockingState.FINAL_ALIGNMENT:
            tag_visible = self.last_target_detection and \
                          (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9 < 1.0
            
            if tag_visible:
                pixel_err = self.IMAGE_CENTER_X - self.last_target_detection.centre.x
                if abs(pixel_err) <= self.PIXEL_TOLERANCE:
                    self.change_state(DockingState.DOCKING)
                    self.publish_twist(0.0, 0.0)
                else:
                    ang_vel = np.clip(self.K_P_PIXEL * pixel_err, -0.5, 0.5)
                    self.publish_twist(0.0, ang_vel)
            else:
                if not self._align_nudge_active:
                    self._align_nudge_active = True
                    self._align_nudge_start_time = self.get_clock().now()
                
                if (self.get_clock().now() - self._align_nudge_start_time).nanoseconds / 1e9 < self.NUDGE_DURATION:
                    self.publish_twist(self.NUDGE_SPEED, 0.0)
                else:
                    self.publish_twist(0.0, 0.0)
                    self._align_nudge_active = False
            return
            
        if self.state == DockingState.DOCKING:
            try:
                t = self.tf_buffer.lookup_transform('robot/chassis', self.DOCKING_TARGET_FRAME, rclpy.time.Time())
                dist_err = t.transform.translation.x - self.DOCKING_TARGET_DISTANCE
                
                if abs(dist_err) < 0.1:
                    self.change_state(DockingState.FINAL_STOP)
                    self.publish_twist(0.0, 0.0)
                else:
                    self.publish_twist(self.NUDGE_SPEED, 0.0)
            except TransformException:
                self.publish_twist(0.0, 0.0)
                self.change_state(DockingState.FINAL_ALIGNMENT)
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
