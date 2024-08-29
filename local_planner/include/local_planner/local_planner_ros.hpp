#ifndef LOCAL_PLANNER_HPP
#define LOCAL_PLANNER_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "nav_msgs/msg/odometry.hpp"       

class LocalPlanner : public rclcpp::Node {
public:
    LocalPlanner();

private:
    // variables
    // subscribe
    bool is_localize_less_mode_;
    std::string topic_name_local_costmap_;
    std::string topic_name_odometry_;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_local_costmap_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;

    // callback_local_costmap
    bool is_local_costmap_received_;

    // callback_odometry
    bool is_odometry_received_;

    struct RobotState {
        double x;
        double y;
        double yaw;
        double vel;
        double steer;
    };

    RobotState robot_state_;

    // callback_timer
    rclcpp::TimerBase::SharedPtr timer_;
    std::string mpc_mode_;

    // functions
    void callback_local_costmap(const grid_map_msgs::msg::GridMap::SharedPtr local_costmap);

    void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr odometry);

    void callback_timer();
};

#endif // LOCAL_PLANNER_HPP
