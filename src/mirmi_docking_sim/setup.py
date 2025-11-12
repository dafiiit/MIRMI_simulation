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
        ],
    },
)
