# F1tenth CDSL_UOS Packages

    ├── local_costmap_generator                 # local grid map through LiDAR
    ├── global_costmap_generator                # global grid map through SLAM
    ├── pose_estimator                          # localization
    ├── local_planner                           # local planner (map-free / map-based)
    ├── global_planner                          # global planner

# Tested Environment

- Ubuntu 20.04 (LTS)
- ROS2 Foxy

# Dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

# Simulation

- f1tenth_gym_ros의 docker-compose.yml 파일이 있는 경로에서 다음 명령어를 통해 sim-1와 novnc-1을 실행한다.

```bash
docker-compose up
```

- 다른 terminal에서 다음 명령어를 통해 sim-1 docker image에 접속한다.

```bash
docker exec -it f1tenth_gym_ros-sim-1 /bin/bash
```

- sim-1 image의 sim_ws 경로에서 다음 명령어들을 통해 launch file을 실행한다.

```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

- [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html)에서 RVIZ 환경을 이용한다.

- Vscode를 통해 sim-1 image에 접속하여 코드를 편집한다.

- 다음 명령어들을 통해 colcon build를 하여 수정사항을 반영한다.

```bash
colcon build
// or
colcon build --packages-select <package_name>
```

- node를 실행한다.

```bash
ros2 run <package_name> <node_name>
```

# Reference

[svg-mppi ros1](https://github.com/kohonda/proj-svg_mppi?tab=readme-ov-file) \
https://github.com/bosky2001/f1tenth_mppi

[F1tenth Official Website](https://f1tenth.org/build.html) \
[https://stevengong.co/notes/F1TENTH-Field-Usage](https://stevengong.co/notes/F1TENTH-Field-Usage)
[f1tenth_system github](https://github.com/f1tenth/f1tenth_system?tab=readme-ov-file) \
[F1TENTH Autonomous Racing Community GITHUB Organization](https://github.com/f1tenth)

Slam

[Cartographer GITHUB](https://github.com/cartographer-project/cartographer?tab=readme-ov-file) \
https://github.com/changh95/fastcampus_slam_codes

ROS2 Workspace

https://github.com/CL2-UWaterloo/f1tenth_ws/tree/main
