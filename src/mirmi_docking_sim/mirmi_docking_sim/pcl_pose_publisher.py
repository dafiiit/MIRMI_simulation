#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import numpy as np
from sensor_msgs_py import point_cloud2
import math
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial import KDTree

class PCLPosePublisher(Node):
    def __init__(self):
        super().__init__('pcl_pose_publisher')
        
        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            PointCloud2, '/depth/points', self.pc_callback, qos_profile_sensor_data)
            
        self.publisher_ = self.create_publisher(PoseStamped, '/localization/docking_pose', 10)
        
        # Reference Cloud Generation
        self.reference_cloud = self.generate_reference_cloud()
        self.reference_tree = KDTree(self.reference_cloud) # Pre-build KDTree
        
        self.last_transform = None # For tracking
        
        self.get_logger().info('PCL Localization Node started.')

    def generate_reference_cloud(self):
        """
        Generates a synthetic point cloud of the hut based on model.sdf dimensions.
        The hut frame is at the center of the floor.
        """
        points = []
        density = 20 # points per meter
        
        # Helper to add points for a box
        def add_box(center, size):
            half_size = size / 2.0
            x_range = np.linspace(center[0] - half_size[0], center[0] + half_size[0], int(size[0] * density))
            y_range = np.linspace(center[1] - half_size[1], center[1] + half_size[1], int(size[1] * density))
            z_range = np.linspace(center[2] - half_size[2], center[2] + half_size[2], int(size[2] * density))
            
            # Back Wall
            if size[0] < 0.5: 
                 y_grid, z_grid = np.meshgrid(y_range, z_range)
                 x_val = center[0] + half_size[0] # Inner face
                 for y, z in zip(y_grid.flatten(), z_grid.flatten()):
                     points.append([x_val, y, z])
            # Side walls
            elif size[1] < 0.5: 
                x_grid, z_grid = np.meshgrid(x_range, z_range)
                y_val = center[1] - half_size[1] if center[1] > 0 else center[1] + half_size[1] # Inner face
                for x, z in zip(x_grid.flatten(), z_grid.flatten()):
                    points.append([x, y_val, z])
            
        # Dimensions from SDF
        # Back wall: -1.45, 0, 0.75; size 0.1, 2.2, 1.5
        add_box(np.array([-1.45, 0, 0.75]), np.array([0.1, 2.2, 1.5]))
        
        # Left wall: 0, 1.05, 0.75; size 3.0, 0.1, 1.5
        add_box(np.array([0, 1.05, 0.75]), np.array([3.0, 0.1, 1.5]))
        
        # Right wall: 0, -1.05, 0.75; size 3.0, 0.1, 1.5
        add_box(np.array([0, -1.05, 0.75]), np.array([3.0, 0.1, 1.5]))
        
        return np.array(points, dtype=np.float32)

    def preprocess_cloud(self, points):
        """
        Filters and downsamples the point cloud.
        """
        # Filter by Depth (Z in camera frame)
        # Increased range to 8.0m to see back wall even from distance
        mask = (points[:, 2] > 0.5) & (points[:, 2] < 8.0)
        points = points[mask]
        
        # Filter by Height (Y in camera frame, Y is down)
        # Ground is at Y > 0.20 roughly (Camera at 0.25m height)
        mask_y = points[:, 1] < 0.20 
        points = points[mask_y]
        
        # Random Downsampling
        if len(points) > 1000:
            idx = np.random.choice(len(points), 1000, replace=False)
            points = points[idx]
            
        return points

    def pc_callback(self, msg):
        # 1. Pointcloud in Numpy wandeln
        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        raw_points = np.array(list(gen))
        if len(raw_points) == 0: return
        
        scene_points = np.column_stack((raw_points['x'], raw_points['y'], raw_points['z']))
        
        # 2. Preprocessing
        scene_points = self.preprocess_cloud(scene_points)
        if len(scene_points) < 100: 
            self.get_logger().warn(f"Not enough points after preprocessing: {len(scene_points)}")
            return

        # 3. Initial Guess
        # If we have a previous good estimate, use it (Tracking).
        # Otherwise, use Centroid Alignment (Initialization).
        
        if self.last_transform is not None:
             T_init = self.last_transform
        else:
            # Centroid Alignment (Fallback)
            centroid_src = np.mean(scene_points, axis=0)
            centroid_dst = np.mean(self.reference_cloud, axis=0)
            translation_init = centroid_dst - centroid_src
            
            T_init = np.eye(4)
            T_init[:3, 3] = translation_init
        
        # 4. ICP
        # Returns T that maps Scene -> Reference (Camera Frame -> Hut Frame)
        # So T = T_hut_cam
        T_hut_cam, distances, iterations = self.icp(scene_points, self.reference_cloud, init_pose=T_init)
        
        fitness = np.mean(distances)
        if fitness > 0.05: # Threshold
            self.get_logger().warn(f"ICP fitness poor: {fitness:.4f} (Iter: {iterations})")
            # If tracking failed, reset it to force Centroid Alignment next time?
            # Or keep it? If we reset, we might jump.
            # Let's reset if it's REALLY bad.
            if fitness > 0.1:
                self.last_transform = None
            return
        
        # Update Tracking
        self.last_transform = T_hut_cam
        
        self.get_logger().info(f"ICP Success. Fitness: {fitness:.4f}. Publishing Pose.")

        # 5. Transform to Robot Frame
        # We have T_hut_cam (Scene in Hut Frame)
        # We want Pose of Hut in Robot Frame: T_robot_hut
        # T_robot_hut = T_robot_cam * T_cam_hut
        # Wait, T_hut_cam maps points from Cam to Hut: P_hut = T_hut_cam * P_cam
        # This means T_hut_cam represents the Camera Pose in Hut Frame.
        # Inverse: T_cam_hut = inv(T_hut_cam) represents Hut Pose in Camera Frame.
        
        T_cam_hut = np.linalg.inv(T_hut_cam)
        
        try:
            # Get T_robot_cam
            # We want transform from robot/chassis to camera frame (to transform Hut Pose from Cam to Robot)
            # T_robot_hut = T_robot_cam * T_cam_hut
            t = self.tf_buffer.lookup_transform(
                'robot/chassis',
                msg.header.frame_id, # Camera Frame
                rclpy.time.Time())
                
            # Convert TF to Matrix
            t_trans = t.transform.translation
            t_rot = t.transform.rotation
            
            T_robot_cam = np.eye(4)
            T_robot_cam[:3, 3] = [t_trans.x, t_trans.y, t_trans.z]
            r = R.from_quat([t_rot.x, t_rot.y, t_rot.z, t_rot.w])
            T_robot_cam[:3, :3] = r.as_matrix()
            
            # Compute Final Pose
            T_robot_hut = np.dot(T_robot_cam, T_cam_hut)
            
            # 6. Publish
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "robot/chassis"
            
            pose_msg.pose.position.x = T_robot_hut[0, 3]
            pose_msg.pose.position.y = T_robot_hut[1, 3]
            pose_msg.pose.position.z = T_robot_hut[2, 3]
            
            quat = R.from_matrix(T_robot_hut[:3, :3]).as_quat()
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            
            self.publisher_.publish(pose_msg)
            
        except TransformException as ex:
            self.get_logger().error(f'Could not transform: {ex}')


    def best_fit_transform(self, A, B):
        '''
        Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
        '''
        assert A.shape == B.shape

        # translate points to their centroids
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        AA = A - centroid_A
        BB = B - centroid_B

        # rotation matrix
        H = np.dot(AA.T, BB)
        U, S, Vt = np.linalg.svd(H)
        R_mat = np.dot(Vt.T, U.T)

        # special reflection case
        if np.linalg.det(R_mat) < 0:
            Vt[2,:] *= -1
            R_mat = np.dot(Vt.T, U.T)

        # translation
        t = centroid_B.T - np.dot(R_mat, centroid_A.T)

        # homogeneous transformation
        T = np.identity(4)
        T[:3, :3] = R_mat
        T[:3, 3] = t

        return T, R_mat, t

    def nearest_neighbor(self, src, dst):
        '''
        Find the nearest (Euclidean) neighbor in dst for each point in src
        '''
        # Use pre-built tree if available
        if hasattr(self, 'reference_tree'):
            distances, indices = self.reference_tree.query(src)
        else:
            # Fallback
            tree = KDTree(dst)
            distances, indices = tree.query(src)
            
        return distances, indices

    def icp(self, A, B, init_pose=None, max_iterations=20, tolerance=0.001):
        '''
        The Iterative Closest Point method: aligns source A to target B
        '''
        src = np.copy(A)
        dst = np.copy(B)

        if init_pose is not None:
            src = np.ones((src.shape[0], 4))
            src[:,0:3] = A
            src = np.dot(init_pose, src.T).T
            src = src[:,0:3]

        prev_error = 0

        for i in range(max_iterations):
            # find the nearest neighbors between the current source and destination points
            distances, indices = self.nearest_neighbor(src, dst)

            # compute the transformation between the current source and nearest destination points
            T, _, _ = self.best_fit_transform(src, dst[indices])

            # update the current source
            src_h = np.ones((src.shape[0], 4))
            src_h[:,0:3] = src
            src_h = np.dot(T, src_h.T).T
            src = src_h[:,0:3]

            # check error
            mean_error = np.mean(distances)
            if np.abs(prev_error - mean_error) < tolerance:
                break
            prev_error = mean_error

        # calculate final transformation
        T, _, _ = self.best_fit_transform(A, src)
        
        return T, distances, i

def main(args=None):
    rclpy.init(args=args)
    node = PCLPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
