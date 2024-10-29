#ifndef PATH_GENERATER_HPP
#define PATH_GENERATER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vector>
#include "simple_path_generater/simple_line_path.hpp"
//#include "simple_path_generater/spline_path.hpp"
#include "simple_path_generater/sp.hpp"


namespace path_generater
{    
class PathGenerater : public rclcpp::Node
{
public:
    explicit PathGenerater(const rclcpp::NodeOptions & options);

private:
    void waypoint_cb(const geometry_msgs::msg::PoseArray & waypoints);
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
    void generate_path(const geometry_msgs::msg::PoseArray & waypoints);
    
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    geometry_msgs::msg::PoseWithCovarianceStamped robot_pose_;
    SimplelinePath simp_line_path;
    SplinePath spline_path;
};
} // namespace path_generater

#endif // PATH_GENERATER_HPP
