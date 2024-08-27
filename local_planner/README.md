# local_planner

- map-free mode (localize_less)
- map-based mode (localize_based)

# Dependencies

[grid_map](https://github.com/ANYbotics/grid_map/tree/ros2)

# Functions

|Return Type|Function Name|Input|Description|Dependencies|
|---|---|---|---|---|
||LocalPlanner||생성자 함수||
|void|callback_timer||||
|void|callback_local_costmap||||
|void|callback_odometry||||

# Variables

|Data Type|Variable Name|Description|
|---|---|---|
|bool|is_localize_mode_|localize mode 여부|
|std::string|topic_odometry_|odometry에 대한 topic 이름|
|std::string|topic_local_costmap_|local costmap에 대한 topic 이름|
