#include "rclcpp/rclcpp.hpp"
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/GridMap.hpp>
// #include <grid_map_ros/GridMapRosConverter.hpp>


class LocalPlanner : public rclcpp::Node 
{
public:
    LocalPlanner();

private:
    bool is_localize_less_mode_;

    // callback_timer
    rclcpp::TimerBase::SharedPtr timer_;

    // subscribe
    std::string topic_local_costmap_;
    std::string topic_odometry_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap_;

    // callback_odometry
    struct RobotState {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        double vel = 0.0;
        double steer = 0.0;
    };

    RobotState robot_state_;
    bool is_robot_state_valid_;

    // callback_local_costmap
    bool is_local_costmap_valid_;
    grid_map::GridMap obstacle_map_;

    void callback_timer();

    void callback_local_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap);

    void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr odometry);
};
