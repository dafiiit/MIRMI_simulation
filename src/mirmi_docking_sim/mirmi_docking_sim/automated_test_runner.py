#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray
import numpy as np
import time
import csv
import math
import random
import os
import sys
import subprocess
from datetime import datetime
from tf_transformations import quaternion_from_euler, quaternion_matrix, euler_from_quaternion, translation_matrix, concatenate_matrices

class DockingTestRunner(Node):
    def __init__(self):
        super().__init__('docking_test_runner')

        self.declare_parameter('test_attempts', 2)
        self.TEST_ATTEMPTS = self.get_parameter('test_attempts').value
        self.TIMEOUT_SEC = 120.0  
        self.HUT_POSITION = np.array([5.0, 2.0])
        
        self.HUT_COLLISION_BOX = {
            'x_min': 3.4, 'x_max': 6.6, 
            'y_min': 0.8, 'y_max': 3.2 
        }

        # Speicherort: Home-Verzeichnis -> docking_test_results
        home_dir = os.path.expanduser("~")
        self.results_dir = os.path.join(home_dir, "docking_test_results")
        os.makedirs(self.results_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_filename = os.path.join(self.results_dir, f"docking_results_{timestamp}.csv")
        self.setup_csv()

        # Publishers / Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth', self.odom_callback, 10)
        self.create_subscription(String, '/docking_controller/state', self.state_callback, 10)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.detection_callback, 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # State Variables
        self.current_pose = None
        self.current_controller_state = "UNKNOWN"
        self.current_attempt = 0
        self.start_time = 0
        self.reset_target_pose = None
        self.recovery_start_time = 0
        self.last_reset_send_time = 0
        
        # Error Calculation
        self.detection_errors = [] 
        self.TAG_1_TRUE_POS = np.array([3.601, 2.0, 0.75]) 
        
        # Statischer Kamera Offset (WICHTIG: Muss zum Launch File passen!)
        try:
            cam_trans = translation_matrix((1.0, 0.0, 0.25))
            cam_rot = quaternion_matrix(quaternion_from_euler(-1.5708, 0.0, -1.5708))
            self.T_robot_camera = concatenate_matrices(cam_trans, cam_rot)
        except Exception as e:
            self.get_logger().error(f"Fehler bei Matrix-Init: {e}")
            self.T_robot_camera = np.eye(4) # Fallback

        self.runner_state = "INIT" 
        self.get_logger().info(f"Test Runner gestartet. CSV: {self.csv_filename}")
        self.timer = self.create_timer(0.1, self.test_loop)

    def setup_csv(self):
        try:
            with open(self.csv_filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                header = [
                    "Versuch_ID", "Ergebnis", "Dauer", "Fehlergrund",
                    "Start_X", "Start_Y", "Start_Theta",
                    "End_X", "End_Y", "End_Theta",
                    "Avg_Detection_Error_m"
                ]
                writer.writerow(header)
                file.flush()
        except Exception as e:
            self.get_logger().error(f"Konnte CSV nicht erstellen: {e}")

    def log_result(self, result, reason):
        try:
            duration = time.time() - self.start_time
            
            # Start Daten
            sx, sy, st = 0,0,0
            if self.reset_target_pose:
                sx, sy, st = self.reset_target_pose
                
            # End Daten (Current GT)
            ex, ey, et = 0,0,0
            if self.current_pose:
                ex = self.current_pose.position.x
                ey = self.current_pose.position.y
                q = self.current_pose.orientation
                _, _, et = euler_from_quaternion([q.x, q.y, q.z, q.w])

            # Error Calc
            avg_error = "N/A"
            if len(self.detection_errors) > 0:
                avg_error = f"{np.mean(self.detection_errors):.4f}"

            with open(self.csv_filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    self.current_attempt, result, f"{duration:.2f}", reason,
                    f"{sx:.2f}", f"{sy:.2f}", f"{st:.2f}",
                    f"{ex:.2f}", f"{ey:.2f}", f"{et:.2f}",
                    avg_error
                ])
                file.flush() # Erzwingt Schreiben auf Disk
            
            self.get_logger().info(f"Ergebnis {self.current_attempt}: {result} ({reason})")
        except Exception as e:
            self.get_logger().error(f"CRITICAL: Fehler beim Loggen des Ergebnisses: {e}")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def state_callback(self, msg):
        self.current_controller_state = msg.data

    def detection_callback(self, msg):
        """Berechnet Fehler sicher in try-except Block"""
        try:
            if self.runner_state != "RUNNING" or not self.current_pose:
                return
            if not msg.detections:
                return
                
            tag_detection = None
            for d in msg.detections:
                if d.id == 1:
                    tag_detection = d
                    break
            
            if tag_detection:
                p = self.current_pose.position
                q = self.current_pose.orientation
                T_world_robot = concatenate_matrices(
                    translation_matrix((p.x, p.y, p.z)),
                    quaternion_matrix((q.x, q.y, q.z, q.w))
                )
                
                # Hier 3x .pose, da Structure: Detection -> PoseWithCov -> Pose -> Pose
                tp = tag_detection.pose.pose.pose.position
                tq = tag_detection.pose.pose.pose.orientation
                T_camera_tag = concatenate_matrices(
                    translation_matrix((tp.x, tp.y, tp.z)),
                    quaternion_matrix((tq.x, tq.y, tq.z, tq.w))
                )
                
                T_world_tag_estimated = concatenate_matrices(T_world_robot, self.T_robot_camera, T_camera_tag)
                est_pos = T_world_tag_estimated[:3, 3]
                
                error = np.linalg.norm(est_pos - self.TAG_1_TRUE_POS)
                self.detection_errors.append(error)
        except Exception as e:
            # Fange Fehler ab, damit Node nicht stirbt
            # self.get_logger().warn(f"Fehler in Detection Calc: {e}") # Optional einkommentieren
            pass

    def get_random_start_pose(self):
        dist = random.uniform(6.0, 10.0)
        angle = random.uniform(0, 2 * math.pi)
        x = self.HUT_POSITION[0] + dist * math.cos(angle)
        y = self.HUT_POSITION[1] + dist * math.sin(angle)
        theta = random.uniform(0, 2 * math.pi) 
        return x, y, theta

    def perform_teleport(self):
        if not self.reset_target_pose: return
        x, y, theta = self.reset_target_pose
        q = quaternion_from_euler(0, 0, theta)
        
        self.cmd_vel_pub.publish(Twist()) 
        
        env = os.environ.copy()
        if 'GZ_PARTITION' not in env: env['GZ_PARTITION'] = 'test_sim'
        if 'GZ_TRANSPORT_IP' not in env: env['GZ_TRANSPORT_IP'] = '127.0.0.1' 

        pose_str = f'name: "robot", position: {{x: {x:.2f}, y: {y:.2f}, z: 0.2}}, orientation: {{x: {q[0]:.4f}, y: {q[1]:.4f}, z: {q[2]:.4f}, w: {q[3]:.4f}}}'
        cmd = [
            'gz', 'service', '-s', '/world/docking_world/set_pose',
            '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000', '--req', pose_str
        ]
        try:
            subprocess.run(cmd, check=True, env=env, timeout=4.0, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

    def send_reset_signal(self):
        if not self.reset_target_pose: return
        x, y, theta = self.reset_target_pose
        q = quaternion_from_euler(0, 0, theta)
        
        reset_msg = PoseWithCovarianceStamped()
        reset_msg.header.stamp = self.get_clock().now().to_msg()
        reset_msg.header.frame_id = "world"
        reset_msg.pose.pose.position.x = x
        reset_msg.pose.pose.position.y = y
        reset_msg.pose.pose.orientation.x = q[0]
        reset_msg.pose.pose.orientation.y = q[1]
        reset_msg.pose.pose.orientation.z = q[2]
        reset_msg.pose.pose.orientation.w = q[3]
        
        self.initial_pose_pub.publish(reset_msg)
        self.get_logger().info("-> Reset-Signal gesendet.")

    def test_loop(self):
        try:
            if not self.current_pose: return 

            if self.runner_state == "INIT":
                if self.current_attempt >= self.TEST_ATTEMPTS:
                    self.get_logger().info("Alle Tests abgeschlossen. Beende Programm.")
                    sys.exit(0) 

                self.current_attempt += 1
                self.reset_target_pose = self.get_random_start_pose()
                self.detection_errors = [] 
                self.get_logger().info(f"--- Starte Versuch {self.current_attempt} ---")
                
                self.perform_teleport()
                self.wait_until_time = time.time() + 2.0 
                self.runner_state = "RESETTING"

            elif self.runner_state == "RESETTING":
                if time.time() > self.wait_until_time:
                    self.send_reset_signal()
                    self.runner_state = "WAIT_FOR_CONTROLLER_RESET"
                    self.last_reset_send_time = time.time()

            elif self.runner_state == "WAIT_FOR_CONTROLLER_RESET":
                if self.current_controller_state == "SEARCHING":
                    self.get_logger().info("Controller Reset bestätigt (SEARCHING). GO!")
                    self.start_time = time.time()
                    self.runner_state = "RUNNING"
                elif (time.time() - self.last_reset_send_time) > 1.0:
                    self.send_reset_signal()
                    self.last_reset_send_time = time.time()

            elif self.runner_state == "RUNNING":
                # Debug Log alle 2 Sekunden, falls er hängt
                if int(time.time()) % 2 == 0 and int(time.time()*10) % 10 == 0:
                     self.get_logger().info(f"RUNNING: State={self.current_controller_state}, PoseX={self.current_pose.position.x:.1f}", throttle_duration_sec=2.0)

                if (time.time() - self.start_time) > self.TIMEOUT_SEC:
                    self.log_result("FAILURE", "TIMEOUT")
                    self.runner_state = "INIT"
                    return

                if self.current_controller_state == "SEARCH_TIMEOUT":
                    self.log_result("FAILURE", "Apriltag während Suche nicht gefunden")
                    self.runner_state = "INIT"
                    return

                cx = self.current_pose.position.x
                cy = self.current_pose.position.y
                
                allowed_states = ["DOCKING", "FINAL_STOP", "FINAL_ALIGNMENT", "ALIGN_TO_HUT"]
                is_docking = self.current_controller_state in allowed_states

                is_in_box = (self.HUT_COLLISION_BOX['x_min'] < cx < self.HUT_COLLISION_BOX['x_max'] and
                             self.HUT_COLLISION_BOX['y_min'] < cy < self.HUT_COLLISION_BOX['y_max'])

                if is_in_box and not is_docking:
                    self.log_result("FAILURE", "COLLISION_WITH_HUT")
                    self.cmd_vel_pub.publish(Twist()) 
                    self.recovery_start_time = time.time()
                    self.runner_state = "COLLISION_RECOVERY"
                    return
                
                if is_docking and cx < 3.65:
                     self.log_result("FAILURE", "CRASH_INTO_BACK_WALL")
                     self.runner_state = "INIT"
                     return

                # FINAL CHECK
                if self.current_controller_state == "FINAL_STOP":
                    dist_to_hut = np.linalg.norm([cx - self.HUT_POSITION[0], cy - self.HUT_POSITION[1]])
                    if dist_to_hut < 1.5:
                        self.log_result("SUCCESS", "DOCKED")
                    else:
                        self.log_result("FAILURE", "FALSE_POSITIVE_STOP")
                    self.runner_state = "INIT"
            
            elif self.runner_state == "COLLISION_RECOVERY":
                self.cmd_vel_pub.publish(Twist()) 
                if (time.time() - self.recovery_start_time) > 1.5:
                    self.runner_state = "INIT"
                    
        except Exception as e:
            self.get_logger().error(f"UNCAUGHT EXCEPTION IN TEST LOOP: {e}")

def main(args=None):
    rclpy.init(args=args)
    runner = DockingTestRunner()
    try:
        rclpy.spin(runner)
    except SystemExit:
        pass
    finally:
        runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
