import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

    pkg_mirmi_docking_sim = get_package_share_directory('mirmi_docking_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    models_path = os.path.join(pkg_mirmi_docking_sim, 'models')
    urdf_path = os.path.join(pkg_mirmi_docking_sim, 'urdf', 'robot.urdf.xacro')
    world_path = os.path.join(pkg_mirmi_docking_sim, 'worlds', 'docking_world.sdf')
    bridge_config_path = os.path.join(pkg_mirmi_docking_sim, 'config', 'bridge.yaml')
    
    # --- ARGUMENTE ---
    use_gt_arg = DeclareLaunchArgument(
        'use_ground_truth',
        default_value='False',
        description='Use Ground Truth pose instead of Odometry'
    )
    
    # NEU: Flag zur Auswahl der Lokalisierungsquelle
    loc_source_arg = DeclareLaunchArgument(
        'localization_source',
        default_value='apriltag',
        description='Source for docking pose: "apriltag" or "pcl"'
    )

    use_ground_truth = LaunchConfiguration('use_ground_truth')
    localization_source = LaunchConfiguration('localization_source')

    # Bedingungen erstellen (Hilfsvariablen für Übersichtlichkeit)
    is_apriltag = IfCondition(PythonExpression(["'", localization_source, "' == 'apriltag'"]))
    is_pcl = IfCondition(PythonExpression(["'", localization_source, "' == 'pcl'"]))

    # 1. Ressourcen-Pfad setzen
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_path = f"{models_path}:{existing_gz_path}"
    set_env = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=new_gz_path)

    # 2. Gazebo & Bridge (Basis-Infrastruktur)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-s -r -v 4 ', world_path]}.items(),
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config_path}'],
        output='screen'
    )

    # 3. Robot State Publisher (URDF)
    doc = xacro.process_file(urdf_path)
    robot_desc = doc.toxml()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc}],
    )

    # 4. Spawn Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_desc,
                   '-name', 'robot',
                   '-x', '0', '-y', '0', '-z', '0.5',
                   '-R', '0', '-P', '0', '-Y', '0'],
        output='screen'
    )
    
    odom_tf_node = Node(
        package='mirmi_docking_sim',
        executable='odom_to_tf',
        name='odom_to_tf_publisher',
        parameters=[{'use_ground_truth': use_ground_truth}]
    )

    # Statische TFs für Kameras (Basis -> Sensoren)
    static_rgb_camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['1.0', '0', '0.25', '-1.5708', '0', '-1.5708', 'robot/chassis', 'robot/chassis/camera_sensor']
    )
    
    static_depth_camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['1.0', '0', '0.25', '-1.5708', '0', '-1.5708', 'robot/chassis', 'robot/chassis/depth_sensor']
    )

    # Helper: Depth Camera Info Sync (läuft immer, schadet nicht)
    depth_info_sync_node = Node(
        package='mirmi_docking_sim', executable='depth_camera_info_sync', output='screen'
    )
    
    # Helper: RGB Camera Info Sync (läuft immer)
    camera_info_node = Node(
        package='mirmi_docking_sim', executable='camera_info_sync', output='screen'
    )

    # ---------------------------------------------------------
    # OPTION A: APRILTAG PIPELINE
    # ---------------------------------------------------------
    
    # 1. Detektor (nur wenn localization_source == 'apriltag')
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_detector',
        condition=is_apriltag,
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
            ('tag_detections_image', '/camera/tag_detections_image')
        ],
        parameters=[{'family': '36h11', 'size': 0.40, 'publish_tf': True}]
    )

    # 2. Adapter: Wandelt TF/Detections in PoseStamped für Controller um
    apriltag_adapter = Node(
        package='mirmi_docking_sim',
        executable='apriltag_pose_publisher', # Siehe setup.py entry points!
        name='localization_adapter',
        condition=is_apriltag,
        output='screen'
    )

    # 3. Visualizer (Optional)
    visualizer_node = Node(
        package='mirmi_docking_sim', executable='apriltag_visualizer',
        condition=is_apriltag
    )
    
    # Statische TFs für die Tags (Simulation "Wahrheit" für Visualisierung in Rviz)
    # Diese werden vom Controller NICHT mehr für die Logik genutzt, nur für Rviz Kontext.
    static_tag_tfs = [
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['3.601', '2.0', '0.75', '0', '-1.5708', '0', 'world', 'tag36_11_00001']), # Inner Back
    ]

    # ---------------------------------------------------------
    # OPTION B: PCL PIPELINE
    # ---------------------------------------------------------

    # 1. Processing Container (Depth Image -> PointCloud2)
    point_cloud_container = ComposableNodeContainer(
        name='point_cloud_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=is_pcl,
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz_node',
                remappings=[
                    ('image_rect', '/depth/image_raw'),
                    ('camera_info', '/depth/camera_info'),
                    ('points', '/depth/points')
                ]
            ),
        ],
        output='screen',
    )

    # 2. Adapter: Rechnet ICP und publisht PoseStamped
    pcl_adapter = Node(
        package='mirmi_docking_sim',
        executable='pcl_pose_publisher', # Siehe setup.py entry points!
        name='localization_adapter',
        condition=is_pcl,
        output='screen'
    )

    # ---------------------------------------------------------
    # GENERIC CONTROLLER
    # ---------------------------------------------------------
    docking_controller_node = Node(
        package='mirmi_docking_sim',
        executable='docking_controller', # Muss der neue, generische sein!
        name='docking_controller',
        output='screen',
        parameters=[{'use_ground_truth': use_ground_truth}]
    )

    # Foxglove Bridge
    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml'
            ])
        )
    )

    return LaunchDescription([
        use_gt_arg,
        loc_source_arg,
        set_env,
        gz_sim,
        gz_bridge,
        robot_state_publisher,
        spawn_robot,
        odom_tf_node,
        static_rgb_camera_tf,
        static_depth_camera_tf,
        depth_info_sync_node,
        camera_info_node,
        
        # Option A
        apriltag_node,
        apriltag_adapter,
        visualizer_node,
        *static_tag_tfs,

        # Option B
        point_cloud_container,
        pcl_adapter,
        
        # Controller
        docking_controller_node,

        # Foxglove Bridge
        foxglove_bridge
    ])
