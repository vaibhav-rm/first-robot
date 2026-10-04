# MyRobot Controller (ROS 2 + Gazebo Sim)

4-wheeled differential-drive robot simulation covering the full pipeline:
**URDF -> world + ArUco -> SLAM -> EKF localisation -> Nav2 -> ArUco missions**.

Stack: ROS 2 Humble, Gazebo Sim (Harmonic, `ros_gz_sim` / `ros_gz_bridge`),
SLAM Toolbox, `robot_localization` EKF, Nav2, OpenCV ArUco.

## Package structure

```
myrobot_controller
├── urdf/myrobot.urdf               # box 0.6x0.4x0.2 + 4 wheels, diff-drive + IMU/GPS/lidar/depth-cam
├── worlds/
│   ├── simple_obstacles.world       # MAIN: walls/obstacles + 3 vertical ArUco boards
│   ├── primitive_obstacles.world     # minimal box+cylinder test world
│   └── turtlebot3_world.world        # upstream reference world
├── models/
│   ├── aruco_marker/                 # legacy single-marker model
│   ├── aruco_marker_1/               # id=1 board at (2.5, 1.5)
│   ├── aruco_marker_2/               # id=2 board at (-2.5, -1.0)
│   └── aruco_marker_3/               # id=3 board at (1.5, -2.8)
├── config/
│   ├── mapper_params_mapping.yaml / mapper_params_localization.yaml
│   ├── ekf_local.yaml (odom) / ekf_global.yaml (map) / navsat_transform.yaml
│   └── nav2_params.yaml
├── launch/
│   ├── slam_ekf.launch.py            # CANONICAL: gz + EKF local+global + SLAM (mode:=mapping|localization)
│   ├── slam_mapping.launch.py        # standalone SLAM node with mode:=mapping|localization
│   ├── nav_mission.launch.py         # NAVIGATION: Nav2 + waypoint_navigator (3 waypoints, 3s halts)
│   ├── nav2_mission.launch.py        # NAVIGATION WITH ARUCO: Nav2 + aruco chain via /Nav2_coordinates
│   ├── my_robot.launchworld.py / my_robot.launch.py / slam_toolbox.launch.py  # compat wrappers
├── myrobot_controller/
│   ├── aruco_detector.py             # OpenCV detect + border + publish /Nav2_coordinates + /aruco/annotated_image
│   └── waypoint_navigator.py         # Nav2 BasicNavigator: static waypoints + dynamic /Nav2_coordinates
├── maps/                             # my_map.yaml/.pgm placeholder + README (replace with real map)
└── rviz/slam.rviz
```

## Robot topics (from `urdf/myrobot.urdf`)

| Sensor / actuator | Topic | Type |
|---|---|---|
| cmd_vel (in) | `/cmd_vel` | geometry_msgs/Twist |
| diff-drive odometry | `/wheel/odom` | nav_msgs/Odometry |
| LiDAR (360 rays, 10 m) | `/scan` | sensor_msgs/LaserScan |
| IMU (100 Hz) | `/imu/data` | sensor_msgs/Imu |
| GPS | `/gps/fix` | sensor_msgs/NavSatFix |
| RGB | `/camera/image_raw` | sensor_msgs/Image |
| Depth | `/camera/depth_image` | sensor_msgs/Image |
| Camera info / points | `/camera/camera_info`, `/camera/points` | sensor_msgs/CameraInfo / PointCloud2 |
| EKF local (odom frame) | `/odometry/local` | nav_msgs/Odometry |
| EKF global (map frame) | `/odometry/global`, `/gps/odom` | nav_msgs/Odometry |
| ArUco next-goal | `/Nav2_coordinates` | geometry_msgs/Point |
| ArUco annotated image | `/aruco/annotated_image` | sensor_msgs/Image |

4-wheel layout: `rear_left/right_wheel_joint` are driven by the DiffDrive
plugin (`wheel_separation 0.40`, `wheel_radius 0.10`); `front_*` are passive
free-spinning wheels preserving the diff-drive kinematic model.

## What are ArUco markers? (research)

ArUco = **Augmented Reality University of Cordoba** fiducial markers: square
black-border patterns with an inner binary matrix encoding an id from a fixed
dictionary (here `DICT_4X4_50`, ids 1-3). OpenCV `cv2.aruco` detects the quad,
`drawDetectedMarkers` draws the border, and corner geometry gives pose. In this
project each marker id encodes the coordinates of the *next* marker; the
detector publishes them to `/Nav2_coordinates` so Nav2 chains
marker 1 -> marker 2 -> marker 3 autonomously.

## Build

```bash
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-nav2-bringup ros-humble-teleop-twist-keyboard \
  ros-humble-cv-bridge python3-opencv
colcon build --symlink-install
source install/setup.bash
```

## Task runs

### URDF + WORLD (spawn in obstacle + ArUco world)
```bash
ros2 launch myrobot_controller slam_ekf.launch.py mode:=mapping
# new terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true
```

### SLAM (mapping -> save map)
```bash
ros2 launch myrobot_controller slam_ekf.launch.py mode:=mapping
# drive with teleop to cover the world, then:
ros2 run nav2_map_server map_saver_cli -f maps/my_map --ros-args -p use_sim_time:=true
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: '$(pwd)/maps/my_map'}"
```

### LOCALISATION (no drift check)
```bash
ros2 launch myrobot_controller slam_ekf.launch.py mode:=localization map_file:=<abs>/maps/my_map
ros2 topic echo /odometry/local --once
ros2 topic echo /odometry/global --once
```
Local EKF fuses wheel encoders + IMU (`odom` frame); global EKF fuses
`odometry/local` + GPS (`/gps/odom` via navsat_transform) + IMU (`map` frame).
SLAM Toolbox in `localization` mode localizes the LiDAR scan in the saved map.

### NAVIGATION (autonomous waypoints, 3 s halts)
```bash
# requires real maps/my_map.yaml first
ros2 launch myrobot_controller nav_mission.launch.py
# or give RViz "Nav2 Goal"s manually; waypoint_navigator.py also runs
# 3 static waypoints (2.0,1.2) -> (-2.0,-0.8) -> (1.2,-2.2) with 3 s halts.
```

### NAVIGATION WITH ARUCO
```bash
ros2 launch myrobot_controller nav2_mission.launch.py
# robot drives to marker 1 approach (2.0,1.2); detector draws borders,
# publishes marker-2 coords to /Nav2_coordinates; navigator chains to
# marker 3; watch /aruco/annotated_image.
ros2 topic echo /Nav2_coordinates
```

## Useful debug commands

```bash
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /wheel/odom
ros2 topic echo /scan --once
ros2 topic echo /gps/fix --once
ros2 topic echo /Nav2_coordinates
```
