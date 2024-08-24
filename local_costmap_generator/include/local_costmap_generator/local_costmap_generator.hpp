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

    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

    bool is_laserscan_received_;

    rclcpp::TimerBase::SharedPtr timer_;

    // functions
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    void timer_callback();

    void print_laser_data(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
};

#endif // LOCAL_COSTMAP_GENERATOR_HPP
