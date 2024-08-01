#include "mppi_controller/mppi_controller_ros.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto mppi_controller = std::make_shared<mppi::MPPIControllerROS>();
    rclcpp::spin(mppi_controller);
    rclcpp::shutdown();
    return 0;
}
