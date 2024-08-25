#include "local_costmap_generator.hpp"

LocalCostmapGenerator::LocalCostmapGenerator() : Node("loca_costmap_generator_node")
{
    laserscan_topic = "/scan";

    sub_laserscan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        laserscan_topic,
        10, 
        std::bind(&LocalCostmapGenerator::scan_callback, this, std::placeholders::_1)
    );

    is_laserscan_received_ = false;

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&LocalCostmapGenerator::timer_callback, this));

    laser_projection_ = std::make_shared<laser_geometry::LaserProjection>();

    pointcloud2_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

    pub_pointcloud2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud2", 10);

    pcl_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
}

void LocalCostmapGenerator::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr laserscan_msg)
{
    laserscan_to_pointcloud2(laserscan_msg);
    pointcloud2_to_pcl(pointcloud2_);

    is_laserscan_received_ = true;
}

void LocalCostmapGenerator::timer_callback()
{
    if (!is_laserscan_received_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for laser scan data...");
        return;
    }
}

void LocalCostmapGenerator::laserscan_to_pointcloud2(const sensor_msgs::msg::LaserScan::ConstSharedPtr laserscan_msg)
{
    laser_projection_->projectLaser(*laserscan_msg, *pointcloud2_);
    print_pointcloud2(pointcloud2_);

    // test
    RCLCPP_INFO(this->get_logger(), "Publishing PointCloud2 message");

    // publish
    pub_pointcloud2_->publish(*pointcloud2_);
}

void LocalCostmapGenerator::pointcloud2_to_pcl(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2)
{
    pcl::fromROSMsg(*pointcloud2, *pcl_);
}

void LocalCostmapGenerator::print_pointcloud2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2)
{
    // 메시지 내용을 간단히 출력
    std::ostringstream oss;
    oss << "PointCloud2: " << std::endl;
    oss << "  Width: " << pointcloud2->width << std::endl;
    oss << "  Height: " << pointcloud2->height << std::endl;
    oss << "  Points: " << pointcloud2->width * pointcloud2->height << std::endl;
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
}