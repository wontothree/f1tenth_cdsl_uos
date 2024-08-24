#ifndef LOCAL_COSTMAP_GENERATOR_HPP
#define LOCAL_COSTMAP_GENERATOR_HPP

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"

class LocalCostmapGenerator : public rclcpp::Node {

public:
    LocalCostmapGenerator();

private:
    // variables

    // Topic
    std::string laserscan_topic;

    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

    bool is_laserscan_received_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<laser_geometry::LaserProjection> laser_projection_;

    std::shared_ptr<sensor_msgs::msg::PointCloud2> pointcloud2_;


    // functions
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr laserscan_msg);

    void timer_callback();

    void laserscan_to_pointcloud2(const sensor_msgs::msg::LaserScan::ConstSharedPtr laserscan_msg);

    void print_pointcloud2(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud2_msg);
};

#endif // LOCAL_COSTMAP_GENERATOR_HPP
