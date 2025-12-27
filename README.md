# 🤖 MyRobot Controller (ROS 2 + Gazebo)

This repository contains a **ROS 2 (Humble) mobile robot simulation package** using **Gazebo Classic**.  
It includes a custom differential-drive robot, multiple Gazebo worlds with obstacles, and **ArUco markers** for perception experiments.

This project is intended for:
- Learning ROS 2 + Gazebo integration
- Mobile robot simulation
- Teleoperation
- Obstacle navigation
- ArUco marker–based perception

---

## 📁 Package Structure

```

myrobot_controller
├── launch
│   ├── my_robot.launch.py            # Launch robot in empty world
│   └── my_robot.launchworld.py       # Launch robot in obstacle world
├── models
│   └── aruco_marker                  # Custom ArUco marker Gazebo model
│       ├── materials
│       │   ├── scripts
│       │   │   └── aruco.material
│       │   └── textures
│       │       ├── aruco_0.png
│       │       └── genarcu.py
│       ├── model.config
│       └── model.sdf
├── urdf
│   └── myrobot.urdf                  # Robot description (URDF)
├── worlds
│   ├── barrels.world
│   ├── primitive_obstacles.world
│   ├── simple_obstacles.world        # Main obstacle + ArUco world
│   └── turtlebot3_world.world
├── myrobot_controller
│   └── **init**.py
├── setup.py
├── setup.cfg
├── package.xml
└── README.md

````

---

## 🚀 Features

- Custom **differential-drive robot**
- Gazebo simulation with ROS 2 integration
- Multiple simulation worlds:
  - Empty world
  - Obstacle-rich world
- **ArUco marker models** placed at multiple locations
- Keyboard teleoperation using `/cmd_vel`
- Ready for perception, navigation, and SLAM experiments

---

## 🧩 Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic (Gazebo 11)
- Python 3

### Install dependencies
```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs \
                 ros-humble-teleop-twist-keyboard
````

---

## 🔧 Build Instructions

From your ROS 2 workspace root:

```bash
cd first-robot
colcon build --symlink-install
source install/setup.bash
```

---

## ▶️ Running the Simulation

### 1️⃣ Launch robot in an **empty world**

```bash
ros2 launch myrobot_controller my_robot.launch.py
```

This will:

* Start Gazebo
* Load an empty world
* Spawn the robot
* Start `robot_state_publisher`

---

### 2️⃣ Launch robot in the **obstacle + ArUco world**

```bash
ros2 launch myrobot_controller my_robot.launchworld.py
```

This will:

* Load `simple_obstacles.world`
* Spawn multiple obstacles (boxes, pillars, ramp, walls)
* Place **3 ArUco markers** at different locations
* Spawn the robot into the environment

---

## 🎮 Teleoperation (Keyboard Control)

Run in a **new terminal** while Gazebo is running:

```bash
source /opt/ros/humble/setup.bash
source ~/astra/ros2_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Keyboard controls

```
w  → move forward
s  → move backward
a  → rotate left
d  → rotate right
x  → stop
```

The robot listens on:

```
/cmd_vel
```

---

## 🏷 ArUco Marker Model

* Located in:

  ```
  models/aruco_marker
  ```
* Implemented as a thin static box with an ArUco texture
* Loaded using:

  ```xml
  <uri>model://aruco_marker</uri>
  ```
* Can be detected using a simulated camera
* Suitable for pose estimation and navigation tasks

---

## 🧪 Useful Debug Commands

Check velocity commands:

```bash
ros2 topic echo /cmd_vel
```

Check odometry:

```bash
ros2 topic echo /odom
```

List all active topics:

```bash
ros2 topic list
```

---
