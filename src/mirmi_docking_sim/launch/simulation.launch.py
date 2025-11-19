import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_mirmi_docking_sim = get_package_share_directory('mirmi_docking_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    models_path = os.path.join(pkg_mirmi_docking_sim, 'models')
    world_path = os.path.join(pkg_mirmi_docking_sim, 'worlds', 'docking_world.sdf')
    bridge_config_path = os.path.join(pkg_mirmi_docking_sim, 'config', 'bridge.yaml')
    
    use_gt_arg = DeclareLaunchArgument(
        'use_ground_truth',
        default_value='False',
        description='Use Ground Truth pose instead of Odometry (for testing)'
    )

    # 1. Umgebungsvariable für Modelle setzen
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    # apriltag_models_path = '/home/david/gazebo_apriltag_harmonic/models'
    # new_gz_path = f"{models_path}:{apriltag_models_path}:{existing_gz_path}"
    new_gz_path = f"{models_path}:{existing_gz_path}"

    set_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_gz_path
    )

    # 2. Gazebo-Simulation starten
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-s -r -v 4 ', world_path],
        }.items(),
    )

    # 3. Die Brücke
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config_path}'],
        output='screen'
    )
    
    # 4. Camera Info Publisher
    camera_info_node = Node(
        package='mirmi_docking_sim',
        executable='camera_info_sync',
        name='camera_info_sync_node'
    )

    # 5. AprilTag Detector Node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_detector',
        output='screen',
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
            ('tag_detections_image', '/camera/tag_detections_image')
        ],
        parameters=[{
            'family': '36h11',
            'size': 0.40,
            'max_hamming': 0,
            # Versuche mehrere mögliche Parameternamen für das annotierte Bild
            'image_transport': 'raw',
            'publish_tag_detections_image': True,
            'publish_tf': True,
            'z_aligned': False,
            'z_up': False,
        }]
    )
    
    # 6. Depth Camera Node
    depth_to_scan_node = Node(
	    package='depthimage_to_laserscan',
	    executable='depthimage_to_laserscan_node',
	    name='depth_to_scan_node',
	    remappings=[
		# Nutzt die neuen Topics aus der bridge.yaml
		('image', '/depth/image_raw'),
		('camera_info', '/depth/camera_info'),
		('scan', '/scan') # Nav2 erwartet standardmäßig /scan
	    ],
	    parameters=[{
		'range_min': 0.1,
		'range_max': 100.0,
		'scan_height': 10, # Nimmt 10 Pixelzeilen aus der Bildmitte
		'output_frame': 'robot/chassis/camera_sensor' # Selber Frame wie die Kameras
	    }]
    )
        
    # Tag 1: Hütte Innen Rückwand (ID 1)
    static_tag_1_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tag_1_publisher',
        arguments=[
            '--x', '3.601', 
            '--y', '2.0', 
            '--z', '0.75',
            '--yaw', '0.0', 
            '--pitch', '-1.5708', 
            '--roll', '0.0',
            '--frame-id', 'world', 
            '--child-frame-id', 'tag36_11_00001'
        ]
    )
    
    # Tag 2: Hütte Außen Rückwand (ID 2)
    static_tag_2_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tag_2_publisher',
        arguments=[
            '--x', '3.499', 
            '--y', '2.0', 
            '--z', '0.75',
            '--yaw', '0.0', 
            '--pitch', '1.5708', 
            '--roll', '0.0',
            '--frame-id', 'world', 
            '--child-frame-id', 'tag36_11_00002'
        ]
    )
    
    # Tag 3: Hütte Außen Linke Wand (ID 3)
    static_tag_3_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tag_3_publisher',
        arguments=[
            '--x', '5.0', 
            '--y', '3.101', 
            '--z', '0.75',
            '--yaw', '0.0', 
            '--pitch', '0.0', 
            '--roll', '1.5708',
            '--frame-id', 'world', 
            '--child-frame-id', 'tag36_11_00003'
        ]
    )
    
    # Tag 4: Hütte Außen Rechte Wand (ID 4)
    static_tag_4_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tag_4_publisher',
        arguments=[
            '--x', '5.0', 
            '--y', '0.899', 
            '--z', '0.75',
            '--yaw', '0.0', 
            '--pitch', '0.0', 
            '--roll', '-1.5708',
            '--frame-id', 'world', 
            '--child-frame-id', 'tag36_11_00004'
        ]
    )
    
    # 7. AprilTag Visualizer - Zeichnet Detektionen auf das Bild
    visualizer_node = Node(
        package='mirmi_docking_sim',
        executable='apriltag_visualizer',
        name='apriltag_visualizer',
        output='screen'
    )
    
    # 8. DOCKING CONTROLLER
    docking_controller_node = Node(
        package='mirmi_docking_sim',
        executable='docking_controller',
        name='docking_controller',
        output='screen',
        parameters=[{
            'use_ground_truth': LaunchConfiguration('use_ground_truth')
        }]
    )
    
    # Odom node
    odom_tf_node = Node(
        package='mirmi_docking_sim',
        executable='odom_to_tf',
        name='odom_to_tf_publisher',
        output='screen',
        # ÄNDERUNG: Parameter durchreichen
        parameters=[{
            'use_ground_truth': LaunchConfiguration('use_ground_truth')
        }]
    )
	
    #Definiert die Position der Kamera am Roboter
    static_camera_tf = Node(
	package='tf2_ros',
	executable='static_transform_publisher',
	name='static_camera_publisher',
	arguments=[
        '--x', '1.0', 
        '--y', '0.0', 
        '--z', '0.25',
        '--yaw', '-1.5708',   # -90 Grad um Z
        '--pitch', '0.0', 
        '--roll', '-1.5708',  # -90 Grad um X
        '--frame-id', 'robot/chassis',          # Übergeordneter Frame
        '--child-frame-id', 'robot/chassis/camera_sensor' # Frame-Name aus Gazebo
        ]
        )

    return LaunchDescription([
    	use_gt_arg,
        set_env,
        gz_sim,
        gz_bridge,
        camera_info_node,
        apriltag_node,
        
        depth_to_scan_node,
 
        odom_tf_node, 
        static_camera_tf,
        
        static_tag_1_tf,
        static_tag_2_tf,
        static_tag_3_tf,
        static_tag_4_tf,
        
        visualizer_node,
        
        docking_controller_node
    ])
