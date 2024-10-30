#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>


geometry_msgs::msg::Quaternion createQuaternionFromYaw(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    return q;
}

class SimplelinePath
{
public:
    void generatePath(
        const geometry_msgs::msg::PoseArray &waypoints, 
        const geometry_msgs::msg::PoseWithCovarianceStamped & robot_pose,
        nav_msgs::msg::Path &path_msg 
    )
    {
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
        
        std::vector<geometry_msgs::msg::PoseStamped> result;
        geometry_msgs::msg::PoseStamped start;
        start.header = waypoints.header;
        start.pose.position.x = robot_pose.pose.pose.position.x;
        start.pose.position.y = robot_pose.pose.pose.position.y;
        start.pose.position.z = robot_pose.pose.pose.position.z; 
        start.pose.orientation = robot_pose.pose.pose.orientation;
        result.push_back(start);    
        auto old_pos = robot_pose.pose.pose.position;
        
        for (const auto &waypoint : waypoints.poses) {      
            double yaw = std::atan2(waypoint.position.y - old_pos.y, waypoint.position.x - old_pos.x);
            
            geometry_msgs::msg::PoseStamped look_target_dir;
            look_target_dir.header = waypoints.header;
            look_target_dir.pose.position.x  = old_pos.x;
            look_target_dir.pose.position.y  = old_pos.y;
            look_target_dir.pose.position.z  = old_pos.z; 
            look_target_dir.pose.orientation = createQuaternionFromYaw(yaw);
            result.push_back(look_target_dir);

            geometry_msgs::msg::PoseStamped go_straight;
            go_straight.header = waypoints.header;
            go_straight.pose.position.x  = waypoint.position.x;
            go_straight.pose.position.y  = waypoint.position.y;
            go_straight.pose.position.z  = waypoint.position.z;
            go_straight.pose.orientation = createQuaternionFromYaw(yaw);
            result.push_back(go_straight);            
            old_pos = waypoint.position;
        }

        return result;
    }
};
