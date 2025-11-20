import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mirmi_docking_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch-Dateien installieren
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
        # Welt-Dateien installieren
        (os.path.join('share', package_name, 'worlds'), 
            glob(os.path.join('worlds', '*.sdf'))),
        # Konfig-Dateien installieren
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.yaml'))),
        # Modell-Dateien installieren (rekursiv)
        (os.path.join('share', package_name, 'models', 'robot'), 
            glob(os.path.join('models', 'robot', '*'))),
        # Hütte-Modell installieren
        (os.path.join('share', package_name, 'models', 'april_hut'), 
            glob(os.path.join('models', 'april_hut', '*'))),
        #April Tags in Modell ordner
        (os.path.join('share', package_name, 'models', 'Apriltag36_11_00001'), 
            glob(os.path.join('models', 'Apriltag36_11_00001', 'model.*'))),
        (os.path.join('share', package_name, 'models', 'Apriltag36_11_00001', 'materials', 'textures'), 
            glob(os.path.join('models', 'Apriltag36_11_00001', 'materials', 'textures', '*.png'))),
            
        (os.path.join('share', package_name, 'models', 'Apriltag36_11_00002'), 
            glob(os.path.join('models', 'Apriltag36_11_00002', 'model.*'))),
        (os.path.join('share', package_name, 'models', 'Apriltag36_11_00002', 'materials', 'textures'), 
            glob(os.path.join('models', 'Apriltag36_11_00002', 'materials', 'textures', '*.png'))),
            
        (os.path.join('share', package_name, 'models', 'Apriltag36_11_00003'), 
            glob(os.path.join('models', 'Apriltag36_11_00003', 'model.*'))),
        (os.path.join('share', package_name, 'models', 'Apriltag36_11_00003', 'materials', 'textures'), 
            glob(os.path.join('models', 'Apriltag36_11_00003', 'materials', 'textures', '*.png'))),
            
        (os.path.join('share', package_name, 'models', 'Apriltag36_11_00004'), 
            glob(os.path.join('models', 'Apriltag36_11_00004', 'model.*'))),
        (os.path.join('share', package_name, 'models', 'Apriltag36_11_00004', 'materials', 'textures'), 
            glob(os.path.join('models', 'Apriltag36_11_00004', 'materials', 'textures', '*.png'))),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='david.metzler.2003@gmail.com',
    description='ROS 2 Simulation für meinen Roboter',
    license='MIT License',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_info_sync = mirmi_docking_sim.camera_info_sync:main',
            'apriltag_visualizer = mirmi_docking_sim.apriltag_visualizer:main',
            'docking_controller = mirmi_docking_sim.docking_controller:main',
            'odom_to_tf = mirmi_docking_sim.odom_to_tf_publisher:main',
            'automated_test_runner = mirmi_docking_sim.automated_test_runner:main',
            'apriltag_pose_publisher = mirmi_docking_sim.apriltag_pose_publisher:main',
            'pcl_pose_publisher = mirmi_docking_sim.pcl_pose_publisher:main',
            'pcl_docking_controller = mirmi_docking_sim.pcl_docking_controller:main',
            'depth_camera_info_sync = mirmi_docking_sim.depth_camera_info_sync:main',
        ],
    },
)
