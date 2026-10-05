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

        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),

        # RViz files
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),

        # Map files (saved maps: my_map.yaml/.pgm + posegraph)
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),

        # ===== Gazebo ArUco model files (base + ids 1-3) =====
        (os.path.join('share', package_name, 'models', 'aruco_marker'),
            glob('models/aruco_marker/model.*')),
        (os.path.join('share', package_name, 'models', 'aruco_marker',
            'materials', 'scripts'),
            glob('models/aruco_marker/materials/scripts/*')),
        (os.path.join('share', package_name, 'models', 'aruco_marker',
            'materials', 'textures'),
            glob('models/aruco_marker/materials/textures/*')),

        (os.path.join('share', package_name, 'models', 'aruco_marker_1'),
            glob('models/aruco_marker_1/model.*')),
        (os.path.join('share', package_name, 'models', 'aruco_marker_1',
            'materials', 'scripts'),
            glob('models/aruco_marker_1/materials/scripts/*')),
        (os.path.join('share', package_name, 'models', 'aruco_marker_1',
            'materials', 'textures'),
            glob('models/aruco_marker_1/materials/textures/*')),

        (os.path.join('share', package_name, 'models', 'aruco_marker_2'),
            glob('models/aruco_marker_2/model.*')),
        (os.path.join('share', package_name, 'models', 'aruco_marker_2',
            'materials', 'scripts'),
            glob('models/aruco_marker_2/materials/scripts/*')),
        (os.path.join('share', package_name, 'models', 'aruco_marker_2',
            'materials', 'textures'),
            glob('models/aruco_marker_2/materials/textures/*')),

        (os.path.join('share', package_name, 'models', 'aruco_marker_3'),
            glob('models/aruco_marker_3/model.*')),
        (os.path.join('share', package_name, 'models', 'aruco_marker_3',
            'materials', 'scripts'),
            glob('models/aruco_marker_3/materials/scripts/*')),
        (os.path.join('share', package_name, 'models', 'aruco_marker_3',
            'materials', 'textures'),
            glob('models/aruco_marker_3/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_navigator = myrobot_controller.waypoint_navigator:main',
            'aruco_detector = myrobot_controller.aruco_detector:main',
            'auto_explorer = myrobot_controller.auto_explorer:main',
            'crop_map = myrobot_controller.crop_map:main',
        ],
    },
)
