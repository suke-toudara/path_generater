#ifndef WAYPOINT_PATH_NODE_HPP
#define WAYPOINT_PATH_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vector>
#include "simple_line_path.hpp"

namespace path_generater
{    
class PathGenerater : public rclcpp::Node
{
public:
    PathGenerater();
    void set_interpolation_method(InterpolationMethod method);

private:
    void waypoint_cb(const geometry_msgs::msg::PoseStamped & waypoints);
    void generate_path(const geometry_msgs::msg::PoseStamped & waypoints);
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    SimplelinePath sim_line_path;

};
} // namespace path_generater

#endif // WAYPOINT_PATH_NODE_HPP
