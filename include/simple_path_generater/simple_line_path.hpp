#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

class SimplelinePath : public rclcpp::Node
{
public:
    SimplelinePath();
private:
    void generatePath(
        geometry_msgs::msg::PoseStamped &waypoints, 
        nav_msgs::msg::Path &path_msg
    )
    {
        std::vector<geometry_msgs::msg::PoseStamped> way_points = interpolateWaypoints(waypoints);

        for (const auto &point : way_points)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x =  point.pose.position.x();
            pose.pose.position.y = point.pose.position.y();
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = point.pose.orientation.z; 
            path_msg.poses.push_back(pose);
        }
    }
private:

    // Yawから四元数を生成する関数
    geometry_msgs::msg::Quaternion createQuaternionFromYaw(double yaw)
    {
        geometry_msgs::msg::Quaternion q;
        q.w = std::cos(yaw * 0.5);
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw * 0.5);
        return q;
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> interpolateWaypoints(
        const geometry_msgs::msg::PoseStamped & waypoints
    )
    {
        std::vector<geometry_msgs::msg::PoseStamped> result;
        int n = waypoints.size();
        double interpolation_step_m_ = 0.3; // 補完間隔[m]

        for (int i = 0; i < n - 1; ++i)
        {
            double delta_x = waypoints[i + 1].pose.position.x - waypoints[i].pose.position.x;
            double delta_y = waypoints[i + 1].pose.position.y - waypoints[i].pose.position.y;
            double distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);
            double yaw_start = getYawFromQuaternion(waypoints[i].pose.orientation);
            double yaw_end = getYawFromQuaternion(waypoints[i + 1].pose.orientation);
            int num_steps = static_cast<int>(distance / interpolation_step);

            for (int step = 0; step <= num_steps; ++step)
            {
                double t = static_cast<double>(step) / num_steps;

                // 線形補間
                double x = waypoints[i].pose.position.x + t * delta_x;
                double y = waypoints[i].pose.position.y + t * delta_y;
                double yaw = yaw_start + t * (yaw_end - yaw_start); // 回転の線形補間

                // PoseStampedを作成
                geometry_msgs::msg::PoseStamped pose;
                // pose.header = header;
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = 0.0; 
                pose.pose.orientation = createQuaternionFromYaw(yaw);
                result.push_back(pose);
            }
        }

        return result;
    }

    
};
