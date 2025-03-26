#include <memory>

#include "diffBotSimulation/vff_avoidance.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto avoidance_node = std::make_shared<diffBotSimulation::AvoidanceNode>();
    rclcpp::spin(avoidance_node);

    rclcpp::shutdown();
    return 0;
}