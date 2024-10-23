#ifndef WAYPOINT_PATH_NODE_HPP
#define WAYPOINT_PATH_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vector>

enum class InterpolationMethod
{
    LINEAR,
    BEZIER,
    SPLINE
};

class WaypointPathNode : public rclcpp::Node
{
public:
    WaypointPathNode();
    
    // メソッドの選択を設定
    void set_interpolation_method(InterpolationMethod method);

private:
    void waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void generate_path();
    
    // 補完方法ごとの処理
    void linear_interpolation(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &end, nav_msgs::msg::Path &path);
    void bezier_interpolation(const std::vector<geometry_msgs::msg::PoseStamped> &control_points, nav_msgs::msg::Path &path);
    void spline_interpolation(const std::vector<geometry_msgs::msg::PoseStamped> &points, nav_msgs::msg::Path &path);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    InterpolationMethod interpolation_method_;
};

#endif // WAYPOINT_PATH_NODE_HPP
