#ifndef LOCAL_COSTMAP_GENERATOR_HPP
#define LOCAL_COSTMAP_GENERATOR_HPP

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LocalCostmapGenerator : public rclcpp::Node {

public:
    LocalCostmapGenerator();

private:
    // Topic
    std::string laserscan_topic;

    // Create ROS subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

    // Callback function to process the laser scan data
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    // Function to print laser scan data
    void print_laser_data(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
};

#endif // LOCAL_COSTMAP_GENERATOR_HPP
