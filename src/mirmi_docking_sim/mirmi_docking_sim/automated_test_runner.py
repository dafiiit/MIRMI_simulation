#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import time
import csv
import math
import random
import os
import subprocess  # <--- WICHTIG: Für den direkten Gazebo-Befehl
from datetime import datetime
from tf_transformations import quaternion_from_euler

class DockingTestRunner(Node):
    def __init__(self):
        super().__init__('docking_test_runner')

        self.declare_parameter('test_attempts', 50)
        self.TEST_ATTEMPTS = self.get_parameter('test_attempts').value
        self.TIMEOUT_SEC = 120.0  
        self.HUT_POSITION = np.array([5.0, 2.0])
        
        # Kollisionsbox
        self.HUT_COLLISION_BOX = {
            'x_min': 3.4, 'x_max': 6.6, 
            'y_min': 0.8, 'y_max': 3.2 
        }

        # CSV Setup (Absoluter Pfad)
        home_dir = os.path.expanduser("~")
        self.results_dir = os.path.join(home_dir, "ros2_ws/src/ROS2_BA_MIRMI/test_results")
        os.makedirs(self.results_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_filename = os.path.join(self.results_dir, f"docking_results_{timestamp}.csv")
        self.setup_csv()

        # Publisher/Subscriber
        # Pose Publisher entfernt, da wir jetzt subprocess nutzen!
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/docking_controller/state', self.state_callback, 10)

        self.current_pose = None
        self.current_controller_state = "UNKNOWN"
        
        # State Machine Variablen
        self.current_attempt = 0
        self.start_time = 0
        self.reset_target_pose = None
        
        # INIT -> RESETTING -> RUNNING -> FINISHED
        self.runner_state = "INIT" 
        self.teleport_deadline = 0

        self.get_logger().info(f"Test Runner gestartet. Ergebnisse in: {self.csv_filename}")
        
        # Schneller Timer für die Logik
        self.timer = self.create_timer(0.1, self.test_loop)

    def setup_csv(self):
        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Versuch_ID", "Ergebnis", "Dauer", "Start_Distanz", "Fehlergrund"])

    def log_result(self, result, reason):
        duration = time.time() - self.start_time
        start_dist = 0.0
        if self.reset_target_pose:
             start_dist = np.linalg.norm(np.array(self.reset_target_pose[:2]) - self.HUT_POSITION)

        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([self.current_attempt, result, f"{duration:.2f}", f"{start_dist:.2f}", reason])
        
        self.get_logger().info(f"Ergebnis Versuch {self.current_attempt}: {result} ({reason})")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def state_callback(self, msg):
        self.current_controller_state = msg.data

    def get_random_start_pose(self):
        # Zufallsposition 6m bis 10m entfernt
        dist = random.uniform(6.0, 10.0)
        angle = random.uniform(0, 2 * math.pi)
        x = self.HUT_POSITION[0] + dist * math.cos(angle)
        y = self.HUT_POSITION[1] + dist * math.sin(angle)
        theta = random.uniform(0, 2 * math.pi) # Zufällige Ausrichtung
        return x, y, theta

    def perform_teleport(self):
        """Sicherer Teleport mit korrekten Flags (--req statt -p)"""
        if not self.reset_target_pose: return
        
        x, y, theta = self.reset_target_pose
        q = quaternion_from_euler(0, 0, theta)
        
        self.cmd_vel_pub.publish(Twist()) # Stop
        
        # Umgebungsvariablen (Fallback)
        env = os.environ.copy()
        if 'GZ_PARTITION' not in env: env['GZ_PARTITION'] = 'test_sim'
        if 'GZ_TRANSPORT_IP' not in env: env['GZ_TRANSPORT_IP'] = '192.168.64.7'

        # Protobuf String
        pose_str = f'name: "robot", position: {{x: {x:.2f}, y: {y:.2f}, z: 0.2}}, orientation: {{x: {q[0]:.4f}, y: {q[1]:.4f}, z: {q[2]:.4f}, w: {q[3]:.4f}}}'
        
        # WICHTIG: Neue Syntax für Gazebo Harmonic (--reqtype, --req)
        # Wir nutzen eine Liste für subprocess, das ist sicherer als shell=True bei Strings mit Quotes
        cmd = [
            'gz', 'service',
            '-s', '/world/docking_world/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', pose_str
        ]
        
        try:
            self.get_logger().info(f"Teleportiere zu x={x:.1f}, y={y:.1f}...")
            # check=True wirft Fehler bei Exit-Code != 0
            subprocess.run(cmd, check=True, env=env, timeout=4.0, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
        except subprocess.CalledProcessError as e:
            # Code 109 ist Timeout (Passiert oft, einfach ignorieren und nochmal versuchen)
            if e.returncode == 109:
                self.get_logger().warn("Gazebo Timeout (109). Simulation beschäftigt? Retry im nächsten Loop...")
            else:
                self.get_logger().error(f"Teleport Fehler (Code {e.returncode}). Befehl war falsch.")
        except Exception as e:
            self.get_logger().error(f"Unbekannter Fehler: {e}")

    def test_loop(self):
        if not self.current_pose: return # Warten auf Odometrie

        # --- STATE MACHINE ---

        # 1. INIT: Neuen Versuch vorbereiten
        if self.runner_state == "INIT":
            if self.current_attempt >= self.TEST_ATTEMPTS:
                self.get_logger().info("Alle Tests abgeschlossen.")
                self.runner_state = "FINISHED"
                return

            self.current_attempt += 1
            self.reset_target_pose = self.get_random_start_pose()
            self.get_logger().info(f"Starte Versuch {self.current_attempt}... Teleportiere Roboter.")
            
            # Wir setzen einen Zeitpunkt, wann es "losgehen" soll (z.B. in 1 Sekunde)
            # Anstatt auf Odom zu warten, vertrauen wir dem Service-Call.
            self.wait_until_time = time.time() + 1.0 
            
            self.perform_teleport()
            self.runner_state = "RESETTING"

        # 2. RESETTING: Einfaches Warten (Blind Trust)
        elif self.runner_state == "RESETTING":
            
            # Wir ignorieren die Odometrie-Prüfung, da Gazebo DiffDrive
            # beim Teleportieren die Odom-Werte oft nicht aktualisiert.
            if time.time() > self.wait_until_time:
                self.get_logger().info(f"Wartezeit vorbei. Test {self.current_attempt} läuft!")
                self.start_time = time.time()
                self.runner_state = "RUNNING"

        # 3. RUNNING: Der eigentliche Test
        elif self.runner_state == "RUNNING":
            
            # A. Timeout
            if (time.time() - self.start_time) > self.TIMEOUT_SEC:
                self.log_result("FAILURE", "TIMEOUT")
                self.runner_state = "INIT"
                return

            # B. Kollision (Hütte)
            cx = self.current_pose.position.x
            cy = self.current_pose.position.y
            if (self.HUT_COLLISION_BOX['x_min'] < cx < self.HUT_COLLISION_BOX['x_max'] and
                self.HUT_COLLISION_BOX['y_min'] < cy < self.HUT_COLLISION_BOX['y_max']):
                self.log_result("FAILURE", "COLLISION_WITH_HUT")
                self.runner_state = "INIT"
                return

            # C. Erfolg
            if self.current_controller_state == "FINAL_STOP":
                # Prüfen ob wirklich in der Hütte (basierend auf aktueller Odom)
                dist_to_hut = np.linalg.norm([cx - self.HUT_POSITION[0], cy - self.HUT_POSITION[1]])
                if dist_to_hut < 1.5:
                    self.log_result("SUCCESS", "DOCKED")
                else:
                    self.log_result("FAILURE", "FALSE_POSITIVE_STOP")
                self.runner_state = "INIT"
                
def main(args=None):
    rclpy.init(args=args)
    runner = DockingTestRunner()
    rclpy.spin(runner)
    runner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
