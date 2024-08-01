// Kohei Honda, 2023

#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <deque>
#include <iostream>
#include <limits>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mppi_metrics_msgs/msg/mppi_metrics.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include "mppi_controller/StopWatch.hpp"
#include "mppi_controller/common.hpp"
#include "mppi_controller/forward_mppi.hpp"
#include "mppi_controller/reverse_mppi.hpp"
#include "mppi_controller/stein_variational_guided_mppi.hpp"
#include "mppi_controller/stein_variational_mpc.hpp"

namespace mppi {
    class MPPIControllerROS : public rclcpp::Node {
    public:
        MPPIControllerROS()
            : Node("mppi_controller_ros"),
              tf_buffer_(this->get_clock()),
              tf_listener_(tf_buffer_)
        {
            // Initialize parameters, publishers, subscribers, timers, etc.
        }
        ~MPPIControllerROS(){}

    private:
        struct RobotState {
            double x = 0.0;
            double y = 0.0;
            double yaw = 0.0;
            double vel = 0.0;
            double steer = 0.0;
        };

    private:
        std::mutex mtx_;

        std::string robot_frame_id_;
        std::string map_frame_id_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        /* pub sub */
        rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_reference_sdf_;   //!< @brief reference sdf subscriber
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;            //!< @brief robot odom subscriber
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_occupancy_grid_;  //!< @brief costmap subscriber (nav_msgs::OccupancyGrid for costmap_2d)
        rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_grid_map_;        //!< @brief grid map subscriber (grid_map_msgs::GridMap for local costmap)
        rclcpp::TimerBase::SharedPtr timer_control_;            //!< @brief timer for control command commutation
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_ackermann_cmd_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_activated_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_backward_point_;

        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_start_cmd_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_stop_cmd_;

        // For debug
        StopWatch stop_watch_;                             //!< @brief stop watch for calculation time
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_best_path_;                     //!< @brief best path topic publisher
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_nominal_path_;                  //!< @brief nominal path topic publisher
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_candidate_paths_;               //!< @brief candidate paths topic publisher
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_proposal_state_distributions_;  //!< @brief proposal state distribution topic publisher
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_control_covariances_;           //!< @brief control covariance topic publisher
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_calculation_time_;              //!< @brief calculation time topic publisher
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_speed_;                         //!< @brief robot speed topic publisher
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_collision_rate_;                //!< @brief collision rate topic publisher
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_cost_;                          //!< @brief cost topic publisher
        rclcpp::Publisher<mppi_metrics_msgs::msg::MPPIMetrics>::SharedPtr pub_mppi_metrics_;                  //!< @brief mppi metrics topic publisher

        /*control system parametes*/
        double control_sampling_time_;  //!< @brief control interval [s]
        bool is_localize_less_mode_ = false;
        bool constant_speed_mode_ = false;
        bool is_visualize_mppi_ = false;
        bool is_start_ = true;
        const std::string obstacle_layer_name_ = "collision_layer";
        const std::string distance_field_layer_name_ = "distance_field";
        const std::string angle_field_layer_name_ = "angle_field";
        const std::string speed_field_layer_name_ = "speed_field";

        /* Variables */
        RobotState robot_state_;
        double reference_speed_ = 0.0;
        double collision_rate_threshold_ = 0.95;  // [0, 1] If collision rate is over this value, stop robot
        grid_map::GridMap obstacle_map_;          // value = [0, 100], 100: collision, 0: free (obstacle layer)
        grid_map::GridMap reference_sdf_;
        ackermann_msgs::msg::AckermannDriveStamped control_msg_;

        double steer_1st_delay_ = 0.1;

        // For stuck detection
        std::deque<float> speed_deque_;
        int speed_deque_size_ = 10;
        float stuck_speed_threshold_ = 0.1;

        bool is_robot_state_ok_ = false;
        bool is_reference_sdf_ok_ = false;
        bool is_costmap_ok_ = false;
        bool is_activate_ad_ = false;
        bool is_simulation_ = false;

        std::unique_ptr<mppi::cpu::MPCTemplate> mpc_solver_ptr_;

        /**
         * @brief Main loop
         *
         */
        void timer_callback();

        void start_cmd_callback(const std_msgs::msg::Empty::SharedPtr msg);

        void stop_cmd_callback(const std_msgs::msg::Empty::SharedPtr msg);

        void callback_odom(const nav_msgs::msg::Odometry::SharedPtr odom);

        void callback_odom_with_pose(const nav_msgs::msg::Odometry::SharedPtr odom);

        void callback_reference_sdf(const grid_map_msgs::msg::GridMap::SharedPtr grid_map);

        void callback_grid_map(const grid_map_msgs::msg::GridMap::SharedPtr grid_map);

        void callback_activate_signal(const std_msgs::msg::Bool::SharedPtr is_activate);

        void publish_candidate_paths(const std::vector<mppi::cpu::StateSeq>& state_seq_batch,
                                     const std::vector<double>& weights,
                                     const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher) const;

        void publish_traj(const mppi::cpu::StateSeq& state_seq,
                          const std::string& name_space,
                          const std::string& rgb,
                          const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher) const;

        void publish_path(const mppi::cpu::StateSeq& state_seq,
                          const std::string& name_space,
                          const std::string& rgb,
                          const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher) const;

        void publish_control_covs(const mppi::cpu::StateSeq& mean,
                                  const mppi::cpu::ControlSeqCovMatrices& cov_matrices,
                                  const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher) const;

        void publish_state_seq_dists(const mppi::cpu::StateSeq& state_seq,
                                     const mppi::cpu::XYCovMatrices& cov_matrices,
                                     const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher) const;
    };

}  // namespace mppi
