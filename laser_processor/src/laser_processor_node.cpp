#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserProcessor : public rclcpp::Node {

public:
    LaserProcessor() : Node("laser_processor_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->lidarscan_topic, 
            10, 
            std::bind(&LaserProcessor::scan_callback, this, std::placeholders::_1)
        );
    }

private:
    // Topic
    std::string lidarscan_topic = "/scan";

    // Create ROS subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    // Callback function to process the laser scan data
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        print_laser_data(scan_msg);
    }

    // Function to print laser scan data
    void print_laser_data(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        // Print some basic information
        RCLCPP_INFO(this->get_logger(), "Received LaserScan data:");
        RCLCPP_INFO(this->get_logger(), "Angle Min: %f, Angle Max: %f", scan_msg->angle_min, scan_msg->angle_max);
        RCLCPP_INFO(this->get_logger(), "Range Min: %f, Range Max: %f", scan_msg->range_min, scan_msg->range_max);

        // Print the range values
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Range[%zu]: %f", i, scan_msg->ranges[i]);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserProcessor>());
    rclcpp::shutdown();
    return 0;
}
