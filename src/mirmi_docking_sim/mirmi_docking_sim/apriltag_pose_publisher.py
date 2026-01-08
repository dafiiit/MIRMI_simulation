#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs 

class AprilTagPosePublisher(Node):
    def __init__(self):
        super().__init__('apriltag_pose_publisher')
        
        self.publisher_ = self.create_publisher(PoseStamped, '/localization/docking_pose', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Wir suchen diesen Frame (wird von apriltag_ros gepublisht)
        self.target_tag_frame = 'tag36_11_00001' 
        self.robot_frame = 'robot/chassis'

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('AprilTag Adapter started.')

    def timer_callback(self):
        try:
            # Lookup Transform: Wo ist der Tag aus Sicht des Roboters?
            # Timeout 0 -> nimm den letzten verfügbaren
            t = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.target_tag_frame,
                rclpy.time.Time())
            
            # --- NEU: Transform zum HÜTTEN-ZENTRUM ---
            # Tag 1 (tag36_11_00001) ist bei (-1.399, 0, 0.75) relativ zum Hüttenzentrum.
            # Aber wir brauchen die Hut-Pose im Roboter-Frame.
            # T_robot_hut = T_robot_tag * T_tag_hut
            
            # T_hut_tag: Pose des Tags in der Hütte
            # x=-1.399, y=0, z=0.75
            # r=0, p=-1.5708, y=0
            
            # Wir machen es einfacher: Da wir wissen, dass der Tag flach an der Rückwand klebt,
            # ist die Translation einfach ein Offset in der lokalen Z-Achse des Tags (wenn Z nach draußen zeigt).
            # Laut Gazebo/SDF: Der Tag ist um -1.5708 um Y rotiert. 
            # Das bedeutet die lokale Z-Achse des Tags zeigt in die Hütte hinein (Richtung X-Hütte).
            # Der Abstand vom Tag (-1.399) zum Zentrum (0) ist 1.399 Meter.
            
            # Wir nutzen tf2_geometry_msgs für eine saubere Transformation
            tag_to_hut = PoseStamped()
            tag_to_hut.header.frame_id = self.target_tag_frame
            # Offset: Der Tag ist bei x=-1.399. Das Zentrum ist bei x=0.
            # In der lokalen Orientierung des Tags (r=0, p=-1.5708, y=0) 
            # zeigt die lokale Z-Achse in die Hütte.
            tag_to_hut.pose.position.z = 1.399 # Gehe 1.399m "vorwärts" vom Tag ins Zentrum
            # Wir nullen auch die Höhe (Z im Roboter-Frame) später im Controller/Publisher wenn nötig,
            # aber hier halten wir es konsistent.
            
            try:
                hut_pose_robot = tf2_geometry_msgs.do_transform_pose(tag_to_hut.pose, t)
                
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = self.robot_frame
                
                pose_msg.pose.position.x = hut_pose_robot.position.x
                pose_msg.pose.position.y = hut_pose_robot.position.y
                pose_msg.pose.position.z = hut_pose_robot.position.z
                pose_msg.pose.orientation = hut_pose_robot.orientation
                
                self.publisher_.publish(pose_msg)
            except Exception as e:
                self.get_logger().error(f"Transform failure: {e}")
            
        except TransformException:
            # Tag gerade nicht sichtbar
            pass

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
