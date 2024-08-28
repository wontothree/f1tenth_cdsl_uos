#include "local_planner/local_planner_ros.hpp"

LocalPlanner::LocalPlanner() : Node("local_planner_node")
{
    // subscribe
    is_localize_less_mode_ = false;
    topic_name_local_costmap_ = "/local_costmap";
    topic_name_odometry_ = "/odom";

    if (is_localize_less_mode_) {
        sub_local_costmap_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
            topic_name_local_costmap_, 
            10, 
            [this](const grid_map_msgs::msg::GridMap::SharedPtr local_costmap) { this->callback_local_costmap(local_costmap); }
        );
        sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
            topic_name_odometry_, 
            10, 
            [this](const nav_msgs::msg::Odometry::SharedPtr odometry) { this->callback_odometry(odometry); }
        );
    } else {
        // ...
    }

    // callback_local_costmap
    is_local_costmap_received_ = false;

    // callback_odometry
    is_odometry_received_ = false;

    // callback_timer
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&LocalPlanner::callback_timer, this));
}

void LocalPlanner::callback_local_costmap(const grid_map_msgs::msg::GridMap::SharedPtr local_costmap)
{
    is_local_costmap_received_ = true;
}

void LocalPlanner::callback_odometry(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
    is_odometry_received_ = true;
}

void LocalPlanner::callback_timer()
{
    // ...
}