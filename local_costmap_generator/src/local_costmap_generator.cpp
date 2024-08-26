#include "local_costmap_generator.hpp"

LocalCostmapGenerator::LocalCostmapGenerator() : Node("loca_costmap_generator_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
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

    pcl_preprocessed_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    robot_frame_id_ = "ego_racecar/base_link";
    sensor_frame_id_ = "ego_racecar/laser";
    pcl_sensor_frame_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl_robot_frame_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
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

    preprocess_pcl(pcl_, pcl_preprocessed_);

    sensor_frame_to_robot_frame(sensor_frame_id_, robot_frame_id_, pcl_preprocessed_, pcl_robot_frame_);
}

void LocalCostmapGenerator::laserscan_to_pointcloud2(const sensor_msgs::msg::LaserScan::ConstSharedPtr laserscan_msg)
{
    laser_projection_->projectLaser(*laserscan_msg, *pointcloud2_);
    // print_pointcloud2(pointcloud2_);

    // publish
    // pub_pointcloud2_->publish(*pointcloud2_);
}

void LocalCostmapGenerator::pointcloud2_to_pcl(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2)
{
    pcl::fromROSMsg(*pointcloud2, *pcl_);

    // test
    // print_pcl(pcl_);
}

void LocalCostmapGenerator::preprocess_pcl(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl, pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_preprocessed)
{
    *pcl_preprocessed = *pcl;
}

void LocalCostmapGenerator::sensor_frame_to_robot_frame(const std::string& sensor_frame_id, const std::string& robot_frame_id, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_sensor_frame, pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_robot_frame)
{
    try {
        transform_stamped_ = tf_buffer_.lookupTransform(robot_frame_id, sensor_frame_id, rclcpp::Time(0));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "[LocalCostmapGenerator] %s", ex.what());
        return;
    }

    const Eigen::Isometry3d transform_matrix = tf2::transformToEigen(transform_stamped_.transform);

    pcl::transformPointCloud(*pcl_sensor_frame, *pcl_robot_frame, transform_matrix.matrix().cast<float>());
}

// functions for test
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

void LocalCostmapGenerator::print_pcl(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl)
{
    std::ostringstream oss;
    oss << "PCL PointCloud: " << std::endl;
    for (const auto& point : pcl->points) {
        oss << "  x: " << point.x << " y: " << point.y << " z: " << point.z << std::endl;
    }
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
}
