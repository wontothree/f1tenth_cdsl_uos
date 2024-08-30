#include "local_planner/local_planner_ros.hpp"

LocalPlanner::LocalPlanner() : Node("local_planner_node")
{
    // subscribe
    is_localize_less_mode_ = false;
    topic_name_local_costmap_ = "/local_costmap";
    topic_name_odometry_ = "/odom";

    if (is_localize_less_mode_) {
        sub_local_costmap_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
            topic_name_local_costmap_, 
            10, 
            [this](const grid_map_msgs::msg::GridMap::SharedPtr local_costmap) { this->callback_local_costmap(local_costmap); }
        );
        sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
            topic_name_odometry_, 
            10, 
            [this](const nav_msgs::msg::Odometry::SharedPtr odometry) { this->callback_odometry(odometry); }
        );
    } else {
        // ...
    }

    // callback_local_costmap
    is_local_costmap_received_ = false;

    // callback_odometry
    is_odometry_received_ = false;

    robot_state_.x = 0.0;
    robot_state_.y = 0.0;
    robot_state_.yaw = 0.0;
    robot_state_.vel = 0.0;
    robot_state_.steer = 0.0;

    // callback_timer
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&LocalPlanner::callback_timer, this));
    mpc_mode_ = "svg_mppi";
    if (mpc_mode_ == "1") {
        // mpc_solver_
    } else if (mpc_mode_ == "2") {
        // mpc_solver_
    } else if (mpc_mode_ == "3") {
        // mpc_solver_
    } else if (mpc_mode_ == "svg_mppi") {
        // mpc_solver_ = std::make_unique<SVGuideMPPI>(params.common, params.svg_mppi);
    } else {
        // error message
        // exit(1)
    }

    control_sampling_time_ = 0.1; // s
    steer_1st_delay_ = 0.1; // s
}

void LocalPlanner::callback_local_costmap([[maybe_unused]] const grid_map_msgs::msg::GridMap::SharedPtr local_costmap)
{
    // grid_map_msgs::msg::GridMap 2 
    is_local_costmap_received_ = true;
}

void LocalPlanner::callback_odometry(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
    robot_state_.x = 0.0;
    robot_state_.y = 0.0;
    robot_state_.yaw = 0.0;
    robot_state_.vel = odometry->twist.twist.linear.x;

    is_odometry_received_ = true;
}

void LocalPlanner::callback_timer()
{
    // status check
    if (is_localize_less_mode_) {
        if (!is_local_costmap_received_ || !is_odometry_received_) {
            // warning message
            return;
        }
    } else {
        // ...
    }

    //
    // mpc_solver_->set_local_costmap(local_costmap_);
    // if (!is_localize_less_mode_) {
    //     mpc_solver_->set_global_costmap(global_costmap_); // global costmap or global plan
    // }

    // steer observer (steering update)
    const double control_input_steer_ = ackermann_msg_.drive.steering_angle;
    robot_state_.steer = robot_state_.steer + (control_input_steer_ - robot_state_.steer) * (1 - std::exp(-control_sampling_time_ / steer_1st_delay_));
    
    // // slove MPC
    // mpc::State initial_state_ = mpc::State::Zero();
    // initial_state_[STATE_SPACE::x] = robot_state_.x;
    // initial_state_[STATE_SPACE::y] = robot_state_.y;
    // initial_state_[STATE_SPACE::yaw] = robot_state_.yaw;
    // initial_state_[STATE_SPACE::vel] = robot_state_.vel;
    // initial_state_[STATE_SPACE::steer] = robot_state_.steer;
    // const auto [update_control_seq, collision_rate] = mpc_solver_->solver(initial_state_);

    // // predict state seq **************

    // // Predict state sequence
    // const auto [best_state_seq, state_cost, collision_cost, input_error] = mpc_solver_ptr_->get_predictive_seq(initial_state, updated_control_seq);

    // // Extract the steering angle
    // const double steering_angle = updated_control_seq(0, CONTROL_SPACE::steer);

    // // Adjust steering angle
    // ackermann_msg_.drive.steering_angle = std::atan2(std::sin(steering_angle), std::cos(steering_angle));

    // // Determine the speed command
    // double speed_cmd = 0.0;
    // if (constant_speed_mode_ || is_localize_less_mode_) {
    //     // Use constant speed if in localize_less_mode or constant_speed_mode
    //     speed_cmd = reference_speed_;
    // } else {
    //     if (reference_sdf_.isInside(grid_map::Position(robot_state_.x, robot_state_.y))) {
    //         // Get the speed from the reference sdf map
    //         speed_cmd = reference_sdf_.atPosition(speed_field_layer_name_, grid_map::Position(robot_state_.x, robot_state_.y));
    //     } else {
    //         // Default to reference speed if outside reference sdf map
    //         speed_cmd = reference_speed_;
    //         RCLCPP_WARN(this->get_logger(),
    //                     "Robot is out of reference sdf map. Use constant speed mode.");
    //     }
    // }

    // // Set the speed command
    // ackermann_msg_.drive.speed = speed_cmd;

    // // Publish the control command
    // pub_ackermann_cmd_->publish(ackermann_msg_);

    // // Compute calculation time if needed
    // const double calculation_time = stop_watch_.lap();

    // // ****************
}
