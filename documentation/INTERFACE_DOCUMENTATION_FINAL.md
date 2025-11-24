# Complete System Architecture (with Hidden Dependencies)
```mermaid
graph LR
    classDef rosNode fill:#d4edda,stroke:#28a745,stroke-width:2px;
    classDef gzNode fill:#fff3cd,stroke:#ffc107,stroke-width:2px;
    classDef topic fill:#e2e3e5,stroke:#6c757d,stroke-width:1px,rx:5,ry:5;
    classDef bridge fill:#f8d7da,stroke:#dc3545,stroke-width:2px,stroke-dasharray: 5 5;
    classDef tf fill:#e1bee7,stroke:#8e24aa,stroke-width:1px,rx:5,ry:5;

    %% Unified ROS Nodes
    apriltag_visualizer(apriltag_visualizer):::rosNode
    apriltag_visualizer --> camera_tag_detections_image([/camera/tag_detections_image]):::topic
    detections --> apriltag_visualizer
    camera_image_raw --> apriltag_visualizer
    pcl_pose_publisher(pcl_pose_publisher):::rosNode
    pcl_pose_publisher --> localization_docking_pose([/localization/docking_pose]):::topic
    tf --> pcl_pose_publisher
    tf_static --> pcl_pose_publisher
    depth_points --> pcl_pose_publisher
    docking_controller(docking_controller):::rosNode
    docking_controller --> docking_controller_state([/docking_controller/state]):::topic
    docking_controller --> cmd_vel([/cmd_vel]):::topic
    ground_truth --> docking_controller
    initialpose --> docking_controller
    tf --> docking_controller
    depth_points --> docking_controller
    tf_static --> docking_controller
    odom --> docking_controller
    localization_docking_pose --> docking_controller
    depth_camera_info_sync(depth_camera_info_sync):::rosNode
    depth_camera_info_sync --> depth_camera_info([/depth/camera_info]):::topic
    depth_image_raw --> depth_camera_info_sync
    automated_test_runner(automated_test_runner):::rosNode
    automated_test_runner --> initialpose([/initialpose]):::topic
    automated_test_runner --> cmd_vel([/cmd_vel]):::topic
    ground_truth --> automated_test_runner
    tf --> automated_test_runner
    detections --> automated_test_runner
    docking_controller_state --> automated_test_runner
    tf_static --> automated_test_runner
    odom_to_tf_publisher(odom_to_tf_publisher):::rosNode
    odom_to_tf_publisher --> tf([/tf]):::tf
    camera_info_sync_node(camera_info_sync_node):::rosNode
    camera_info_sync_node --> camera_camera_info([/camera/camera_info]):::topic
    camera_image_raw --> camera_info_sync_node
    apriltag_pose_publisher(apriltag_pose_publisher):::rosNode
    apriltag_pose_publisher --> localization_docking_pose([/localization/docking_pose]):::topic
    tf --> apriltag_pose_publisher
    tf_static --> apriltag_pose_publisher
    parameter_bridge(parameter_bridge):::rosNode
    static_transform_publisher(static_transform_publisher):::rosNode
    static_transform_publisher --> tf_static([/tf_static]):::tf
    camera_info_sync(camera_info_sync):::rosNode
    apriltag_detector(apriltag_detector):::rosNode
    apriltag_detector --> camera_tag_detections_image([/camera/tag_detections_image]):::topic
    apriltag_detector --> detections([detections]):::topic
    apriltag_detector --> tf([tf]):::tf
    camera_camera_info --> apriltag_detector
    camera_image_raw --> apriltag_detector
    localization_adapter(localization_adapter):::rosNode
    point_cloud_xyz_node(point_cloud_xyz_node):::rosNode
    point_cloud_xyz_node -.-> depth_image_raw([/depth/image_raw]):::topic
    point_cloud_xyz_node -.-> depth_camera_info([/depth/camera_info]):::topic
    point_cloud_xyz_node -.-> depth_points([/depth/points]):::topic

    %% Gazebo & Bridge
    GazeboSim(Gazebo Simulation):::gzNode
    GazeboSim -- camera_sensor --> model_robot_camera[[/model/robot/camera]]:::gzNode
    GazeboSim -- depth_sensor --> model_robot_depth_camera[[/model/robot/depth_camera]]:::gzNode
    model_robot_cmd_vel ==> Bridge_cmd_vel_model_robot_cmd_vel{Bridge}:::bridge ==> cmd_vel
    cmd_vel ==> Bridge_cmd_vel_model_robot_cmd_vel{Bridge}:::bridge ==> model_robot_cmd_vel
    model_robot_odom_nav ==> Bridge_odom_model_robot_odom_nav{Bridge}:::bridge ==> odom
    odom ==> Bridge_odom_model_robot_odom_nav{Bridge}:::bridge ==> model_robot_odom_nav
    world_docking_world_model_robot_joint_state ==> Bridge_joint_states_world_docking_world_model_robot_joint_state{Bridge}:::bridge ==> joint_states
    joint_states ==> Bridge_joint_states_world_docking_world_model_robot_joint_state{Bridge}:::bridge ==> world_docking_world_model_robot_joint_state
    model_robot_camera ==> Bridge_camera_image_raw_model_robot_camera{Bridge}:::bridge ==> camera_image_raw
    camera_image_raw ==> Bridge_camera_image_raw_model_robot_camera{Bridge}:::bridge ==> model_robot_camera
    model_robot_depth_camera ==> Bridge_depth_image_raw_model_robot_depth_camera{Bridge}:::bridge ==> depth_image_raw
    depth_image_raw ==> Bridge_depth_image_raw_model_robot_depth_camera{Bridge}:::bridge ==> model_robot_depth_camera
    model_robot_depth_camera_camera_info ==> Bridge_depth_camera_info_model_robot_depth_camera_camera_info{Bridge}:::bridge ==> depth_camera_info
    depth_camera_info ==> Bridge_depth_camera_info_model_robot_depth_camera_camera_info{Bridge}:::bridge ==> model_robot_depth_camera_camera_info
    model_robot_pose ==> Bridge_model_robot_pose_model_robot_pose{Bridge}:::bridge ==> model_robot_pose
    model_robot_pose ==> Bridge_model_robot_pose_model_robot_pose{Bridge}:::bridge ==> model_robot_pose
    model_robot_odometry ==> Bridge_ground_truth_model_robot_odometry{Bridge}:::bridge ==> ground_truth
    ground_truth ==> Bridge_ground_truth_model_robot_odometry{Bridge}:::bridge ==> model_robot_odometry
    model_robot_set_pose ==> Bridge_model_robot_set_pose_model_robot_set_pose{Bridge}:::bridge ==> model_robot_set_pose
    model_robot_set_pose ==> Bridge_model_robot_set_pose_model_robot_set_pose{Bridge}:::bridge ==> model_robot_set_pose
```