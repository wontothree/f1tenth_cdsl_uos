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

    // senor_frame_to_robot_frame
    robot_frame_id_ = "ego_racecar/base_link";
    sensor_frame_id_ = "ego_racecar/laser";
    pcl_robot_frame_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    // remove_pcl_within_robot
    pcl::CropBox<pcl::PointXYZ> crop_box_filter_;
    double rigid_body_shape_baselink2front = 0.47; // m
    double rigid_body_shape_baselink2rear = 0.14;
    double rigid_body_shape_baselink2right = 0.15;
    double rigid_body_shape_baselink2left = 0.15;
    double min_high = 0.0;
    double max_high = 10.0;
    double min_x = -rigid_body_shape_baselink2rear;
    double max_x = rigid_body_shape_baselink2front;
    double min_y = -rigid_body_shape_baselink2right;
    double max_y = rigid_body_shape_baselink2left;
    crop_box_min_ = Eigen::Vector4f(min_x, min_y, min_high, 1.0);
    crop_box_max_ = Eigen::Vector4f(max_x, max_y, max_high, 1.0);

    // pcl_to_costmap
    thread_num_ = 4;
    cell_occupancy_value = 100;
}

void LocalCostmapGenerator::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr laserscan)
{
    // 1. sensor_msgs::LaserScan 2 sensor_msgs::PointCloud2
    laserscan_to_pointcloud2(laserscan, pointcloud2_);

    // 2. sensor_msgs::PointCloud2 2 pcl::PointCloud
    pointcloud2_to_pcl(pointcloud2_, pcl_);

    is_laserscan_received_ = true;
}

void LocalCostmapGenerator::timer_callback()
{
    if (!is_laserscan_received_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for laser scan data...");
        return;
    }

    // 3. preprocess
    preprocess_pcl(pcl_);

    // 4. sensor frame coordinate 2 robot frame coordinate
    sensor_frame_to_robot_frame(sensor_frame_id_, robot_frame_id_, pcl_, pcl_robot_frame_);

    // 5. remove cloud points within robot
    remove_pcl_within_robot(pcl_robot_frame_);

    // 6. pcl 2 costmap

    // 7. inflate rigid body
}

void LocalCostmapGenerator::laserscan_to_pointcloud2(const sensor_msgs::msg::LaserScan::ConstSharedPtr laserscan, sensor_msgs::msg::PointCloud2::SharedPtr pointcloud2)
{
    laser_projection_->projectLaser(*laserscan, *pointcloud2);
    // print_pointcloud2(pointcloud2_);

    // publish
    // pub_pointcloud2_->publish(*pointcloud2_);
}

void LocalCostmapGenerator::pointcloud2_to_pcl(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl)
{
    pcl::fromROSMsg(*pointcloud2, *pcl);

    // test
    // print_pcl(pcl_);
}

void LocalCostmapGenerator::preprocess_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl)
{
    *pcl = *pcl;
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

    // print_pcl_robot_frame();
}

void LocalCostmapGenerator::remove_pcl_within_robot(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl)
{
    crop_box_filter_.setInputCloud(pcl);
    crop_box_filter_.setNegative(true);
    crop_box_filter_.setMin(crop_box_min_);
    crop_box_filter_.setMax(crop_box_max_);
    crop_box_filter_.filter(*pcl);

    print_pcl_robot_frame();
}

// std::vector<grid_map::Index> LocalCostmapGenerator::pcl_to_costmap(const pcl::PointCloud<PointXYZ>::ConstPtr pcl, grid_map::GridMap* costmap) const
// {
//     grid_map::Matrix& costmap_ = costmap->get("collision_layer");

//     costmap_.setZero();

//     std::vector<grid_map::Index> occupied_indices(pcl->points.size());

//     #pragma omp parallel for num_threads(thread_num_)
//     for (unsigned int i = 0; i < pcl->points.size(); ++i) {
//         const auto& point = pcl->points[i];

//         if (costmap->isInside(grid_map::Position(point.x, point.y))) {
//             grid_map::Index index;
//             costmap->getIndex(grid_map::Position(point.x, point.y), index);
//             costmap_(index.x(), index.y()) = cell_occupancy_value;
//             occupied_indices[i] = index;
//         } else {
//             occupied_indices[i] = grid_map::Index(-1, -1);
//         }
//     }

//     occupied_indices.erase(
//         std::remove_if(
//             occupied_indices.begin(), occupied_indices.end(),
//             [](const grid_map::Index& index) {
//                 return index.x() == -1 && index.y() == -1;
//             }
//         ),
//         occupied_indices.end()
//     );


//     return occupied_indices;
// }

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

void LocalCostmapGenerator::print_pcl_robot_frame()
{
    if (pcl_robot_frame_->empty()) {
        RCLCPP_INFO(this->get_logger(), "pcl_robot_frame_ is empty.");
        return;
    }

    std::ostringstream oss;
    oss << "pcl_robot_frame_ PointCloud: " << std::endl;
    for (const auto& point : pcl_robot_frame_->points) {
        oss << "  x: " << point.x << " y: " << point.y << " z: " << point.z << std::endl;
    }
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
}
