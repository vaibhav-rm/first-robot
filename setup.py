from setuptools import setup
import os
from glob import glob

package_name = 'myrobot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[

        # Ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),

        # World files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),

        # ===== Gazebo ArUco model files =====

        # model.config and model.sdf
        (os.path.join('share', package_name, 'models', 'aruco_marker'),
            glob('models/aruco_marker/model.*')),

        # material scripts
        (os.path.join(
            'share', package_name, 'models', 'aruco_marker',
            'materials', 'scripts'),
            glob('models/aruco_marker/materials/scripts/*')),

        # textures (PNG)
        (os.path.join(
            'share', package_name, 'models', 'aruco_marker',
            'materials', 'textures'),
            glob('models/aruco_marker/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vaibhav',
    maintainer_email='vaibhav@example.com',
    description='Mobile robot with sensors and ArUco markers in Gazebo',
    license='Apache License 2.0',
)

