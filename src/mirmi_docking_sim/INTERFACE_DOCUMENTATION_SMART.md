# Smart Interface Dokumentation
> Mit erweitertem Wissen über Standard-Nodes (AprilTag, etc.).

## Erkanntes Verhalten
Das Skript nutzt eine Wissensdatenbank für folgende Launch-Executables:
- `apriltag_node`
- `depthimage_to_laserscan_node`
- `static_transform_publisher`

## System Architektur
```mermaid
graph LR
    classDef rosNode fill:#d4edda,stroke:#28a745,stroke-width:2px;
    classDef gzNode fill:#fff3cd,stroke:#ffc107,stroke-width:2px;
    classDef topic fill:#e2e3e5,stroke:#6c757d,stroke-width:1px,rx:5,ry:5;
    classDef bridge fill:#f8d7da,stroke:#dc3545,stroke-width:2px,stroke-dasharray: 5 5;

    %% ROS Python Nodes
    DockingTestRunner(DockingTestRunner):::rosNode
    DockingTestRunner --> cmd_vel([/cmd_vel]):::topic
    DockingTestRunner --> initialpose([/initialpose]):::topic
    odom --> DockingTestRunner
    docking_controller_state --> DockingTestRunner
    FakeCameraInfoPublisher(FakeCameraInfoPublisher):::rosNode
    FakeCameraInfoPublisher --> camera_camera_info([/camera/camera_info]):::topic
    camera_image_raw --> FakeCameraInfoPublisher
    OdomToTFPublisher(OdomToTFPublisher):::rosNode
    odom --> OdomToTFPublisher
    AprilTagVisualizer(AprilTagVisualizer):::rosNode
    AprilTagVisualizer --> camera_tag_detections_image([/camera/tag_detections_image]):::topic
    camera_image_raw --> AprilTagVisualizer
    detections --> AprilTagVisualizer
    DockingController(DockingController):::rosNode
    DockingController --> docking_controller_state([/docking_controller/state]):::topic
    DockingController --> cmd_vel([/cmd_vel]):::topic
    model_robot_pose --> DockingController
    odom --> DockingController
    detections --> DockingController
    initialpose --> DockingController

    %% Launch Nodes (Smart)
    parameter_bridge(parameter_bridge):::rosNode
    camera_info_sync_node(camera_info_sync_node):::rosNode
    apriltag_detector(apriltag_detector):::rosNode
    camera_image_raw([/camera/image_raw]):::topic --> apriltag_detector
    camera_camera_info([/camera/camera_info]):::topic --> apriltag_detector
    apriltag_detector --> detections([detections]):::topic
    apriltag_detector --> camera_tag_detections_image([/camera/tag_detections_image]):::topic
    apriltag_detector --> tf([tf]):::topic
    depth_to_scan_node(depth_to_scan_node):::rosNode
    depth_image_raw([/depth/image_raw]):::topic --> depth_to_scan_node
    depth_camera_info([/depth/camera_info]):::topic --> depth_to_scan_node
    depth_to_scan_node --> scan([/scan]):::topic
    static_tag_1_publisher(static_tag_1_publisher):::rosNode
    static_tag_1_publisher --> tf([/tf]):::topic
    static_tag_1_publisher --> tf_static([/tf_static]):::topic
    static_tag_2_publisher(static_tag_2_publisher):::rosNode
    static_tag_2_publisher --> tf([/tf]):::topic
    static_tag_2_publisher --> tf_static([/tf_static]):::topic
    static_tag_3_publisher(static_tag_3_publisher):::rosNode
    static_tag_3_publisher --> tf([/tf]):::topic
    static_tag_3_publisher --> tf_static([/tf_static]):::topic
    static_tag_4_publisher(static_tag_4_publisher):::rosNode
    static_tag_4_publisher --> tf([/tf]):::topic
    static_tag_4_publisher --> tf_static([/tf_static]):::topic
    apriltag_visualizer(apriltag_visualizer):::rosNode
    docking_controller(docking_controller):::rosNode
    odom_to_tf_publisher(odom_to_tf_publisher):::rosNode
    static_camera_publisher(static_camera_publisher):::rosNode
    static_camera_publisher --> tf([/tf]):::topic
    static_camera_publisher --> tf_static([/tf_static]):::topic
    automated_test_runner(automated_test_runner):::rosNode

    %% Gazebo & Bridge
    GazeboSim(Gazebo Simulation):::gzNode
    GazeboSim -- camera_sensor --> model_robot_camera[[/model/robot/camera]]:::gzNode
    GazeboSim -- depth_sensor --> model_robot_depth_camera[[/model/robot/depth_camera]]:::gzNode
    cmd_vel ==> Bridge_cmd_vel_model_robot_cmd_vel{Bridge}:::bridge ==> model_robot_cmd_vel
    model_robot_odometry ==> Bridge_odom_model_robot_odometry{Bridge}:::bridge ==> odom
    world_docking_world_model_robot_joint_state ==> Bridge_joint_states_world_docking_world_model_robot_joint_state{Bridge}:::bridge ==> joint_states
    model_robot_camera ==> Bridge_camera_image_raw_model_robot_camera{Bridge}:::bridge ==> camera_image_raw
    model_robot_depth_camera_depth ==> Bridge_depth_image_raw_model_robot_depth_camera_depth{Bridge}:::bridge ==> depth_image_raw
    model_robot_depth_camera_depth_camera_info ==> Bridge_depth_camera_info_model_robot_depth_camera_depth_camera_info{Bridge}:::bridge ==> depth_camera_info
    model_robot_pose ==> Bridge_model_robot_pose_model_robot_pose{Bridge}:::bridge ==> model_robot_pose
    model_robot_set_pose ==> Bridge_model_robot_set_pose_model_robot_set_pose{Bridge}:::bridge ==> model_robot_set_pose
```