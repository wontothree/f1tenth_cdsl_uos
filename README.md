# F1tenth Seoul

This project is for 22ND F1TENTH AUTONOMOUS GRAND Prix at CDC 2024.

1:10 scaled autonomous race car

    ├── local_costmap_generator                 # local grid map through LiDAR
    ├── global_costmap_generator                # global grid map through SLAM
    ├── pose_estimator                          # localization
    ├── local_planner                           # local planner (map-free / map-based)
    ├── global_planner                          # global planner
    ├── grid_map                                # dependencies

# Tested Environment

- Ubuntu 20.04 (LTS)
- ROS2 Foxy

# Dependencies

Install dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

# Getting Started

Run 'sim-1' and 'novnc-1' in directory 'f1tenth_gym_ros' having 'docker-compose.yml'

```bash
docker-compose up
```

Contact docker image 'sim-1' in another terminal

```bash
docker exec -it f1tenth_gym_ros-sim-1 /bin/bash
```

Run launch file in path 'sim-1' of sim_ws

```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

You can see RVIZ in [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html)

You can edit source codes in 'sim-1' image by using vscode

Apply the edited codes

```bash
colcon build

colcon build --packages-select <package_name>
```

Run the node

```bash
ros2 run <package_name> <node_name>
```

# Reference

[svg-mppi ros1](https://github.com/kohonda/proj-svg_mppi?tab=readme-ov-file) \
https://github.com/bosky2001/f1tenth_mppi

ROS2 Workspace

https://github.com/CL2-UWaterloo/f1tenth_ws/tree/main
