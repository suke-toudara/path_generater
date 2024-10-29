#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "spline.h"

class SplinePath
{
public:
    void generatePath(
        const geometry_msgs::msg::PoseArray &waypoints, 
        const geometry_msgs::msg::PoseWithCovarianceStamped & robot_pose,
        nav_msgs::msg::Path &path_msg 
    ){
        std::vector<geometry_msgs::msg::PoseStamped> way_points = interpolateWaypoints(waypoints,robot_pose);
        for (const auto &point : way_points)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = point.pose.position.x;
            pose.pose.position.y = point.pose.position.y;
            pose.pose.position.z = point.pose.position.z;
            pose.pose.orientation.w = point.pose.orientation.z; 
            path_msg.poses.push_back(pose);
        }
    }
private:
    std::vector<geometry_msgs::msg::PoseStamped> interpolateWaypoints(
        const geometry_msgs::msg::PoseArray & waypoints,
        const geometry_msgs::msg::PoseWithCovarianceStamped & robot_pose
    )
    {
        std::vector<double> X; 
        std::vector<double> Y;
        for (const auto &pose : waypoints.poses) {
            X.push_back(pose.position.x);
            Y.push_back(pose.position.y);
        }

        std::vector<double> t_values(waypoints.poses.size());
        for (size_t i = 0; i < t_values.size(); ++i) {
            t_values[i] = static_cast<double>(i);
        }

        tk::spline::spline_type type = tk::spline::cspline;
        tk::spline sx, sy;
        sx.set_points(t_values,X,type);
        sy.set_points(t_values,Y,type);

        std::vector<geometry_msgs::msg::PoseStamped> result;
        for (double t = 0.0; t <= static_cast<double>(t_values.back()); t += 0.1) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = waypoints.header;
            pose.pose.position.x = sx(t);
            pose.pose.position.y = sy(t);
            pose.pose.position.z = 0.0;
            result.push_back(pose);
        }        
        return result;
    }
};
