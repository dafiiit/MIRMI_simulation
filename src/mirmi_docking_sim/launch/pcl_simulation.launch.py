import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    pkg_mirmi_docking_sim = get_package_share_directory('mirmi_docking_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    models_path = os.path.join(pkg_mirmi_docking_sim, 'models')
    world_path = os.path.join(pkg_mirmi_docking_sim, 'worlds', 'docking_world.sdf')
    bridge_config_path = os.path.join(pkg_mirmi_docking_sim, 'config', 'bridge.yaml')
    
    use_gt_arg = DeclareLaunchArgument(
        'use_ground_truth',
        default_value='False',
        description='Use Ground Truth pose instead of Odometry'
    )

    # 1. Ressourcen-Pfad setzen
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_path = f"{models_path}:{existing_gz_path}"
    set_env = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=new_gz_path)

    # 2. Gazebo starten
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-s -r -v 4 ', world_path]}.items(),
    )

    # 3. Bridge starten
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config_path}'],
        output='screen'
    )
    
    # 4. Odom to TF (wie gehabt)
    odom_tf_node = Node(
        package='mirmi_docking_sim',
        executable='odom_to_tf',
        name='odom_to_tf_publisher',
        parameters=[{'use_ground_truth': LaunchConfiguration('use_ground_truth')}]
    )
    
    # 5. TF für die Kameras
    # RGB Kamera (existierend)
    static_rgb_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_rgb_camera_publisher',
        arguments=['1.0', '0.0', '0.25', '-1.5708', '0.0', '-1.5708', 'robot/chassis', 'robot/chassis/camera_sensor']
    )

    # NEU: Depth Kamera TF
    # Annahme: Gleiche Position wie RGB, aber eigener Frame.
    # Der Frame-Name 'robot/chassis/depth_sensor' muss mit dem Header der Gazebo-Message übereinstimmen (siehe Bridge Output oder Foxglove)
    static_depth_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_depth_camera_publisher',
        arguments=['1.0', '0.0', '0.25', '-1.5708', '0.0', '-1.5708', 'robot/chassis', 'robot/chassis/depth_sensor']
    )

    # NEU: Depth Camera Info Sync Node
    # Publiziert CameraInfo synchron zu den Depth Images
    depth_info_sync_node = Node(
        package='mirmi_docking_sim',
        executable='depth_camera_info_sync',
        name='depth_camera_info_sync',
        output='screen'
    )

    # 6. Point Cloud Processing Pipeline
    # Wir nutzen einen ComposableNodeContainer für effizienteres Intra-Process-Communication
    point_cloud_container = ComposableNodeContainer(
        name='point_cloud_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz_node',
                remappings=[
                    ('image_rect', '/depth/image_raw'),   # Input Depth
                    ('camera_info', '/depth/camera_info'), # Input Info
                    ('points', '/depth/points')           # Output Cloud
                ]
            ),
        ],
        output='screen',
    )

    # 7. Dein neuer PCL Docking Controller (Placeholder)
    pcl_controller_node = Node(
        package='mirmi_docking_sim',
        executable='pcl_docking_controller',
        name='pcl_docking_controller',
        output='screen'
    )

    return LaunchDescription([
        use_gt_arg,
        set_env,
        gz_sim,
        gz_bridge,
        odom_tf_node,
        static_rgb_camera_tf,
        static_depth_camera_tf,
        depth_info_sync_node,
        point_cloud_container,
        pcl_controller_node 
    ])
