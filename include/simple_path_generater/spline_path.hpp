#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

class SplinePath
{
public:
    void generatePath(
        const geometry_msgs::msg::PoseArray &waypoints, 
        const geometry_msgs::msg::PoseWithCovarianceStamped & robot_pose,
        nav_msgs::msg::Path &path_msg 
    ){
        std::vector<geometry_msgs::msg::PoseStamped> way_points = interpolateWaypoints(waypoints,robot_pose);
        int i = 0;
        int n = waypoints.poses.size() - 1;
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
        int n = waypoints.poses.size() - 1;
        std::vector<geometry_msgs::msg::PoseStamped> result;
        Eigen::VectorXd a_x(n), b_x(n), c_x(n), d_x(n);
        Eigen::VectorXd a_y(n), b_y(n), c_y(n), d_y(n);

        calculateSplineCoefficients(waypoints, a_x, b_x, c_x, d_x, a_y, b_y, c_y, d_y);

        double step_size = 0.1; // 補間の間隔（適宜調整してください）
        for (int i = 0; i < n; ++i)
        {
            double x_start = waypoints.poses[i].position.x;
            double x_end = waypoints.poses[i + 1].position.x;
            double y_start = waypoints.poses[i].position.y;
            double y_end = waypoints.poses[i + 1].position.y;

            // 区間の長さを計算
            double interval_length = std::sqrt(std::pow(x_end - x_start, 2) + std::pow(y_end - y_start, 2));
            int num_steps = static_cast<int>(std::ceil(interval_length / step_size));

            for (int j = 0; j <= num_steps; ++j)
            {
                double t = static_cast<double>(j) / num_steps; 
                double x = evaluateSpline(t, a_x[i], b_x[i], c_x[i], d_x[i]);
                double y = evaluateSpline(t, a_y[i], b_y[i], c_y[i], d_y[i]);

                geometry_msgs::msg::PoseStamped pose;
                pose.header = waypoints.header;
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = waypoints.poses[i].position.z; 
                pose.pose.orientation = waypoints.poses[i].orientation;

                result.push_back(pose);
            }
        }
        return result;
    }
    
    double evaluateSpline(double t, double a, double b, double c, double d)
    {
        // スプラインの評価
        return a + b * t + c * t * t + d * t * t * t;
    }

    void calculateSplineCoefficients(
        const geometry_msgs::msg::PoseArray &waypoints, 
        Eigen::VectorXd &a_x, Eigen::VectorXd &b_x, Eigen::VectorXd &c_x, Eigen::VectorXd &d_x,
        Eigen::VectorXd &a_y, Eigen::VectorXd &b_y, Eigen::VectorXd &c_y, Eigen::VectorXd &d_y
    )
    {
        int n = waypoints.poses.size() - 1;  
        Eigen::VectorXd h(n), alpha_x(n), alpha_y(n);
        Eigen::VectorXd l(n + 1), mu(n), z_x(n + 1), z_y(n + 1), c_temp_x(n + 1), c_temp_y(n + 1);

        // h、alpha_x、alpha_yの計算
        for (int i = 0; i < n; ++i)
        {
            h(i) = waypoints.poses[i + 1].position.x - waypoints.poses[i].position.x;
            alpha_x(i) = (waypoints.poses[i + 1].position.x - waypoints.poses[i].position.x) / (3.0 * h(i));
            alpha_y(i) = (waypoints.poses[i + 1].position.y - waypoints.poses[i].position.y) / (3.0 * h(i));
        }

        // 前進ステップ
        l(0) = 1.0;
        mu(0) = 0.0;
        z_x(0) = 0.0;
        z_y(0) = 0.0;
        for (int i = 1; i < n; ++i)
        {
            l(i) = 2.0 * (2.0 - h(i - 1));
            mu(i) = h(i - 1) / l(i);
            z_x(i) = (alpha_x(i - 1) - h(i - 1) * z_x(i - 1)) / l(i);
            z_y(i) = (alpha_y(i - 1) - h(i - 1) * z_y(i - 1)) / l(i);
        }

        // 逆進ステップ
        l(n) = 1.0;
        z_x(n) = 0.0;
        z_y(n) = 0.0;
        c_temp_x(n) = 0.0;
        c_temp_y(n) = 0.0;

        for (int i = n - 1; i >= 0; --i)
        {
            c_temp_x(i) = z_x(i) - mu(i) * c_temp_x(i + 1);
            b_x(i) = (waypoints.poses[i + 1].position.x - waypoints.poses[i].position.x) / h(i) - h(i) * (c_temp_x(i + 1) + 2.0 * c_temp_x(i)) / 3.0;
            d_x(i) = (c_temp_x(i + 1) - c_temp_x(i)) / (3.0 * h(i));
            a_x(i) = waypoints.poses[i].position.x;
            c_x(i) = c_temp_x(i);

            c_temp_y(i) = z_y(i) - mu(i) * c_temp_y(i + 1);
            b_y(i) = (waypoints.poses[i + 1].position.y - waypoints.poses[i].position.y) / h(i) - h(i) * (c_temp_y(i + 1) + 2.0 * c_temp_y(i)) / 3.0;
            d_y(i) = (c_temp_y(i + 1) - c_temp_y(i)) / (3.0 * h(i));
            a_y(i) = waypoints.poses[i].position.y;
            c_y(i) = c_temp_y(i);
        }
    }
};
