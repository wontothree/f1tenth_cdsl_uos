#ifndef LOCAL_COSTMAP_GENERATOR_HPP
#define LOCAL_COSTMAP_GENERATOR_HPP

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "laser_geometry/laser_geometry.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.h"
// #include "pcl/transform.h"
#include <pcl/common/transforms.h>

class LocalCostmapGenerator : public rclcpp::Node {

public:
    LocalCostmapGenerator();

private:
    // variables

    // Topic
    std::string laserscan_topic;

    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserscan_;

    bool is_laserscan_received_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<laser_geometry::LaserProjection> laser_projection_;

    std::shared_ptr<sensor_msgs::msg::PointCloud2> pointcloud2_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud2_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_preprocessed_;

    std::string robot_frame_id_;
    std::string sensor_frame_id_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_sensor_frame_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_robot_frame_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // functions
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr laserscan_msg);

    void timer_callback();

    void laserscan_to_pointcloud2(const sensor_msgs::msg::LaserScan::ConstSharedPtr laserscan_msg);

    void pointcloud2_to_pcl(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2);

    void preprocess_pcl(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl, pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_preprocessed);

    void sensor_frame_to_robot_frame(const std::string& sensor_frame_id, const std::string& robot_frame_id, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_sensor_frame, pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_robot_frame);

    void print_pointcloud2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg);

    void print_pcl(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl);
};

#endif // LOCAL_COSTMAP_GENERATOR_HPP
