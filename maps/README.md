# maps/

Real maps built with SLAM Toolbox (see below), replacing the old placeholder.

## Files

- `my_map.yaml` / `my_map.pgm` — occupancy grid for Nav2 AMCL
  (`nav_mission.launch.py` / `nav2_mission.launch.py` load this by default).
  104x97 @ 0.05 m/px, origin [-2.601, -2.906, 0].
- `my_map.posegraph` / `my_map.data` — SLAM Toolbox pose graph for
  `mode:=localization` (`slam_ekf.launch.py mode:=localization
  map_file:=<abs path>/maps/my_map`).

## How this map was produced (SLAM task)

Headless Gazebo Sim run of this repo (ROS 2 Jazzy container; launches are
Humble/Jazzy compatible):

```
ros2 launch myrobot_controller slam_ekf.launch.py mode:=mapping headless:=true
```

Driven with a scripted coverage driver equivalent to manual
`teleop_twist_keyboard` (reactive wander: forward 0.18 m/s, turn away when
LiDAR sees < 1.0 m ahead, sinusoidal steering, 3.0 m radial geofence around
the obstacle course so the robot stays in mapped area). 6-minute run, then:

```
# Nav2 grid map
ros2 run nav2_map_server map_saver_cli -f maps/my_map --ros-args -p use_sim_time:=true
# SLAM Toolbox pose graph (Jazzy SaveMap takes {name: {data: ...}})
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$(pwd)/maps/my_map'}}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$(pwd)/maps/my_map'}"
```

(Humble note: `SaveMap` there takes `{name: '<path>'}` directly.)

## How to localize (LOCALISATION task)

```
ros2 launch myrobot_controller slam_ekf.launch.py mode:=localization map_file:=<abs path>/maps/my_map
```

Check in RViz: robot pose in `/map` is stable and not drifting; both EKF nodes
(`odometry/local` for odom frame, `odometry/global` for map frame) are alive:

```
ros2 topic echo /odometry/local --once
ros2 topic echo /odometry/global --once
```

## Re-mapping for fuller coverage

This map covers the obstacle course but ~60% of the grid is still unknown.
To improve: re-run mapping with a longer drive (raise `DURATION_SEC`), or
drive manually with teleop to sweep behind the wall and pillars, then re-save.
