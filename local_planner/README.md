# local_planner

# Functions

|Return|Function Name|Input|Description|Dependencies|
|---|---|---|---|---|
||callback_local_costmap||||
||callback_odometry||||
||callback_timer||||

# Variables

|Type|Variable Name|Description|Initialization|Dependencies|
|---|---|---|---|---|
|bool|is_localize_less_mode_|localize 모드 여부||
|std::string|topic_name_local_costmap_|||string|
|std::string|topic_name_odometry_|||string|
|rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr|sub_local_costmap_|local costmap에 대한 topic subscriber 객체 smart pointor||grid_map_msgs/msg/grid_map.hpp|
|rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr|sub_odometry_|lodometry에 대한 topic subscriber 객체 smart pointer||nav_msgs/msg/odometry.hpp|
|bool|is_local_costmap_received_||false||
|bool|is_odometry_received_||false||
||||||
|rclcpp::TimerBase::SharedPtr|timer_|타이머 참조 변수||rclcpp|
|std::string|mpc_mode_|||string|
||mpc_solver_|||