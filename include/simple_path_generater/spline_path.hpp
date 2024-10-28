#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

/*
Si​(x)=ai​+bi​(x−xi​)+ci​(x−xi​)2+di​(x−xi​)3
*/
class SplinePathNode 
{
public:
    SplinePathNode();

    void generatePath(
        geometry_msgs::msg::PoseStamped &waypoints, 
        nav_msgs::msg::Path &path_msg
    ){
    }
private:
    std::vector<geometry_msgs::msg::PoseStamped> interpolateWaypoints(
        const geometry_msgs::msg::PoseStamped & waypoints
    )
    {
        std::vector<Eigen::Vector2d> result;

        int n = waypoints_x.size() - 1;

        // スプライン補間するための係数を計算
        Eigen::VectorXd a_x(n), b_x(n), c_x(n), d_x(n);
        Eigen::VectorXd a_y(n), b_y(n), c_y(n), d_y(n);

        calculateSplineCoefficients(waypoints_x, a_x, b_x, c_x, d_x);
        calculateSplineCoefficients(waypoints_y, a_y, b_y, c_y, d_y);

        // 補間された点を生成 (0.1間隔で補間)
        for (int i = 0; i < n; ++i)
        {
            for (double t = 0.0; t < 1.0; t += 0.1)
            {
                double x = a_x(i) + b_x(i) * t + c_x(i) * t * t + d_x(i) * t * t * t;
                double y = a_y(i) + b_y(i) * t + c_y(i) * t * t + d_y(i) * t * t * t;
                result.emplace_back(x, y);
            }
        }

        return result;
    }

    void calculateSplineCoefficients(const std::vector<double> &waypoints, Eigen::VectorXd &a, Eigen::VectorXd &b, Eigen::VectorXd &c, Eigen::VectorXd &d)
    {
        int n = waypoints.size() - 1;
        Eigen::VectorXd h(n), alpha(n), l(n + 1), mu(n), z(n + 1), c_temp(n + 1);

        // hとalphaの計算
        for (int i = 0; i < n; ++i)
        {
            h(i) = waypoints[i + 1] - waypoints[i]; 
            alpha(i) =  (waypoints[i + 1] - waypoints[i]) / (3.0 * h(i));
        }

        // 前進ステップ
        l(0) = 1.0;
        mu(0) = 0.0;
        z(0) = 0.0;
        for (int i = 1; i < n; ++i)
        {
            l(i) = 2.0 * (2.0 - h(i - 1));
            mu(i) = h(i - 1) / l(i);
            z(i) = (alpha(i - 1) - h(i - 1) * z(i - 1)) / l(i);
        }

        // 逆進ステップ
        l(n) = 1.0;
        z(n) = 0.0;
        c_temp(n) = 0.0;
        for (int i = n - 1; i >= 0; --i)
        {
            c_temp(i) = z(i) - mu(i) * c_temp(i + 1);
            b(i) = (waypoints[i + 1] - waypoints[i]) / h(i) - h(i) * (c_temp(i + 1) + 2.0 * c_temp(i)) / 3.0;
            d(i) = (c_temp(i + 1) - c_temp(i)) / (3.0 * h(i));
            a(i) = waypoints[i];
            c(i) = c_temp(i);
        }
    }
};
