import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image  # NEUER IMPORT

class FakeCameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('fake_camera_info_publisher')
        
        # 1. Wir lauschen auf das Original-Bild-Topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Das Topic, von dem die schnellen Bilder kommen
            self.image_callback,
            10)
        
        # 2. Wir veröffentlichen auf dem Info-Topic
        self.publisher_ = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # 3. Der Timer wird ENTFERNT. Das Bild ist unser neuer Taktgeber.
        # self.timer = self.create_timer(0.5, self.publish_info) # ALT
        
        self.get_logger().info('Fake CameraInfo Publisher gestartet (im Sync-Modus).')

        # Statische Kameradaten (aus camera_info.yaml / deinem alten Skript)
        self.cam_info = CameraInfo()
        self.cam_info.width = 640
        self.cam_info.height = 480
        self.cam_info.distortion_model = 'plumb_bob'
        self.cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.cam_info.k = [554.256, 0.0, 320.5,
                           0.0, 554.256, 240.5,
                           0.0, 0.0, 1.0]
        self.cam_info.r = [1.0, 0.0, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 1.0]
        self.cam_info.p = [554.256, 0.0, 320.5, 0.0,
                           0.0, 554.256, 240.5, 0.0,
                           0.0, 0.0, 1.0, 0.0]

    def image_callback(self, img_msg):
        # Jedes Mal, wenn ein Bild (img_msg) ankommt:
        
        # 1. Nimm die vorbereitete Info-Nachricht
        info_msg = self.cam_info
        
        # 2. DAS IST DER WICHTIGSTE SCHRITT:
        # Kopiere den Header (inkl. Zeitstempel und Frame ID) vom Bild
        # direkt in die Info-Nachricht.
        info_msg.header = img_msg.header
        
        # 3. Veröffentliche die perfekt synchronisierte Info-Nachricht
        self.publisher_.publish(info_msg)

    # Die alte publish_info Methode wird nicht mehr gebraucht

def main(args=None):
    rclpy.init(args=args)
    node = FakeCameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

