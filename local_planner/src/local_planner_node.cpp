#include "local_planner/local_planner.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalCostmapGenerator>());
    rclcpp::shutdown();
    return 0;
}
