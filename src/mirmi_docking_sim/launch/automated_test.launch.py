import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_mirmi_docking_sim = get_package_share_directory('mirmi_docking_sim')

    # 1. Simulation mit Ground Truth Flag starten
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mirmi_docking_sim, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'use_ground_truth': 'True'}.items()
    )

    # 2. Test-Runner
    test_runner_node = Node(
        package='mirmi_docking_sim',
        executable='automated_test_runner',
        name='automated_test_runner',
        output='screen',
        parameters=[{'test_attempts': 2}]
    )

    return LaunchDescription([
        simulation_launch,
        test_runner_node
    ])
