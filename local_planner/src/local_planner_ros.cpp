#include "local_planner/local_planner.hpp"

LocalPlanner::LocalPlanner() : Node("local_planner_node")
{
    is_localize_less_mode_ = true;

    // callback_timer
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&LocalPlanner::callback_timer, this));

    topic_local_costmap_ = "local_costmap";
    topic_odometry_ = "odom";

    if (is_localize_less_mode_) {
        // subscribe
        sub_costmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(topic_local_costmap_, 10, std::bind(&LocalPlanner::callback_local_costmap, this, std::placeholders::_1));

        sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(topic_odometry_, 10, std::bind(&LocalPlanner::callback_odometry, this, std::placeholders::_1));


    } else {
        // ...
    }

    // callback_odometry
    is_robot_state_valid_ = false;

    // callback_local_costmap
    is_local_costmap_valid_ = false;
}

void LocalPlanner::callback_timer()
{
    // ...
}

void LocalPlanner::callback_local_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap)
{
    // if (!grid_map::GridMapRosConverter::fromMessage(*local_costmap, obstacle_map_)) {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to convert grid map");
    // }

    is_local_costmap_valid_ = true;
}

void LocalPlanner::callback_odometry(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
    robot_state_.x = 0.0;
    robot_state_.y = 0.0;
    robot_state_.yaw = 0.0;
    robot_state_.vel = odometry->twist.twist.linear.x;

    is_robot_state_valid_ = true;
}