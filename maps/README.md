# maps/

This folder holds the occupancy-grid + posegraph maps built with SLAM Toolbox.

- `my_map.yaml` / `my_map.pgm` are currently **placeholders** so
  `nav_mission.launch.py` / `nav2_mission.launch.py` can load without crashing.
  Replace them with a real map before autonomous runs.

## How to create a real map (SLAM task)

1. Launch mapping mode + teleop:
   ```
   ros2 launch myrobot_controller slam_ekf.launch.py mode:=mapping
   ```
2. Drive slowly with teleop (`teleop_twist_keyboard`) covering the whole
   `simple_obstacles.world` so LiDAR sees all walls/obstacles.
3. Save Nav2 map (for AMCL/Nav2):
   ```
   ros2 run nav2_map_server map_saver_cli -f maps/my_map --ros-args -p use_sim_time:=true
   ```
4. Save SLAM Toolbox posegraph (for `mode:=localization`):
   ```
   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
     "{name: '$(pwd)/maps/my_map'}"
   ```
   This writes `my_map.posegraph` (+ `.data`) next to the `.pgm`/`.yaml`.
5. Commit the real `my_map.*` files to GitHub.

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
