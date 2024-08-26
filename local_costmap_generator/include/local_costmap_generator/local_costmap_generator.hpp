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
#include <pcl/common/transforms.h>

#include <pcl/filters/crop_box.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/TypeDefs.hpp>

#ifdef _OPENMP
#include <omp.h>
#endif

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

    // senor_frame_to_robot_frame
    std::string robot_frame_id_;
    std::string sensor_frame_id_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_robot_frame_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // remove_pcl_within_robot
    pcl::CropBox<pcl::PointXYZ> crop_box_filter_;
    double rigid_body_shape_baselink2front;
    double rigid_body_shape_baselink2rear;
    double rigid_body_shape_baselink2right;
    double rigid_body_shape_baselink2left;
    double min_high;
    double max_high;
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    Eigen::Vector4f crop_box_min_;
    Eigen::Vector4f crop_box_max_;

    // pcl_to_costmap
    int thread_num_;
    double cell_occupancy_value;

    // functions
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr laserscan);

    void timer_callback();

    // 1
    void laserscan_to_pointcloud2(const sensor_msgs::msg::LaserScan::ConstSharedPtr laserscan, sensor_msgs::msg::PointCloud2::SharedPtr pointcloud2);

    // 2
    void pointcloud2_to_pcl(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl);

    // 3
    void preprocess_pcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl);

    // 4
    void sensor_frame_to_robot_frame(const std::string& sensor_frame_id, const std::string& robot_frame_id, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_sensor_frame, pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_robot_frame);

    // 5
    void remove_pcl_within_robot(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl);

    // 6
    void pcl_to_costmap(const pcl::PointCloud<PointXYZ>::ConstPtr pcl, grid_map::GridMap* costmap) const;

    void print_pointcloud2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg);

    void print_pcl(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl);

    void print_pcl_robot_frame();
};

#endif // LOCAL_COSTMAP_GENERATOR_HPP
