"""Clean SLAM Mapping: Gazebo + EKF local + SLAM Toolbox (mapping) + RViz.

NO EKF global, NO navsat_transform - avoids TF conflicts.
Run: ros2 launch myrobot_controller mapping_only.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('myrobot_controller')

    urdf_path = os.path.join(pkg_share, 'urdf', 'myrobot.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'simple_obstacles.world')
    ekf_local_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    slam_mapping_params = os.path.join(pkg_share, 'config', 'mapper_params_mapping.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', '-r -v 4 ' + world_path)],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': ParameterValue(Command(['cat ', urdf_path]), value_type=str)}],
        output='screen')

    # Spawn robot from robot_description topic
    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', 'my_robot', '-topic', 'robot_description',
                       '-x', '0.0', '-y', '0.0', '-z', '0.3'],
            output='screen')])

    gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/wheel/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/simple_obstacles_world/model/my_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen')

    # EKF LOCAL ONLY (odom -> base_footprint)
    ekf_local_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_local_node',
        parameters=[ekf_local_config, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/local')],
        output='screen')

    # SLAM Toolbox MAPPING (async_slam_toolbox_node) - publishes map -> odom
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_mapping_params, {'use_sim_time': use_sim_time}],
    )

    # Lifecycle manager for SLAM Toolbox (auto configure + activate)
    slam_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': ['slam_toolbox']}])

    # Topic relays for camera
    relay_image = Node(
        package='topic_tools', executable='relay',
        arguments=['/camera/image', '/camera/image_raw'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    relay_info = Node(
        package='topic_tools', executable='relay',
        arguments=['/camera_info', '/camera/camera_info'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    relay_joints = Node(
        package='topic_tools', executable='relay',
        arguments=['/world/simple_obstacles_world/model/my_robot/joint_state', '/joint_states'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # RViz2
    rviz2 = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    set_model_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_share, 'models'))

    return LaunchDescription([
        set_model_path,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo, robot_state_publisher, spawn_robot, gz_bridge,
        relay_image, relay_info, relay_joints,
        ekf_local_node,
        slam_toolbox,
        slam_lifecycle_manager,
        rviz2,
    ])