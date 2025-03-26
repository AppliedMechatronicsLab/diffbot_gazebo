#ifndef VFF_AVOIDANCE_HPP
#define VFF_AVOIDANCE_HPP

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "visualization_msgs/msg/marker_array.hpp"

struct VFFVectors 
{
    std::vector<float> attractive;
    std::vector<float> repulse;
    std::vector<float> result;
};

typedef enum {RED, GREEN, BLUE, NUM_COLORS} VFFColors;
namespace diffBotSimulation
{
class AvoidanceNode : public rclcpp::Node 
{
    public:
        AvoidanceNode();

        void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
        void control_cycle();

    protected:
        VFFVectors get_vff(const sensor_msgs::msg::LaserScan &scan);

        visualization_msgs::msg::MarkerArray get_debug_vff(const VFFVectors & vff_vectors);
        visualization_msgs::msg::Marker make_marker(
            const std::vector<float> & vector, VFFColors vff_color);
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vff_debug_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
};
} // namespace diffBotSimulation

#endif