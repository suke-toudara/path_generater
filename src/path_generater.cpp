#include <path_generater/path_generater.hpp>
#include <cmath>

namespace path_generater
{
PathGenerater::PathGenerater(const rclcpp::NodeOptions & option) 
: Node("path_generater",option)
{
    waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/waypoint", 10, std::bind(&PathGenerater::waypoint_cb, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/estimated_pose", 10, std::bind(&PathGenerater::pose_callback, this, std::placeholders::_1));
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("generated_path", 10);
    
}

void PathGenerater::waypoint_cb(const geometry_msgs::msg::PoseArray & waypoints)
{
    generate_path(waypoints);
}

void PathGenerater::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
    robot_pose_ = msg;
}

void PathGenerater::generate_path(const geometry_msgs::msg::PoseArray & waypoints)
{
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = "map";
    //simp_line_path.generatePath(waypoints,robot_pose_,path);
    spline_path.generatePath(waypoints,robot_pose_,path);
    // switch (interpolation_method_)
    // {
    // case InterpolationMethod::LINEAR:
    //     linear_interpolation(waypoints_, path);
    //     break;
    // case InterpolationMethod::BEZIER:
    //     bezier_interpolation(waypoints_, path);
    //     break;

    // case InterpolationMethod::SPLINE:
    //     spline_interpolation(waypoints_, path);
    //     break;
    // }
    path_pub_->publish(path);
}
} // namespace path_generater

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_generater::PathGenerater)

