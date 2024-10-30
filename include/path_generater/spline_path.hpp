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
        std::vector<double> a_x , a_y;
        std::vector<double> b_x , b_y;
        std::vector<double> c_x , c_y;
        std::vector<double> d_x , d_y;
       
        std::vector<double> x_point; 
        std::vector<double> y_point;
        for (const auto &pose : waypoints.poses) {
            x_point.push_back(pose.position.x);
            y_point.push_back(pose.position.y);
        }
        calculateSplineCoefficients(x_point, a_x, b_x, c_x, d_x);
        calculateSplineCoefficients(y_point, a_y, b_y, c_y, d_y);
        
        double step_num = 10; 
        for (int i = 0; i < n; ++i)
        {
            double start_x = waypoints.poses[i].position.x;
            double start_y = waypoints.poses[i].position.y;
            double x_interval = waypoints.poses[i + 1].position.x - waypoints.poses[i].position.x;
            double y_interval = waypoints.poses[i + 1].position.y - waypoints.poses[i].position.y;
            double x_step = x_interval / step_num;
            double y_step = y_interval / step_num;    
            for (int j = 0; j <= step_num; ++j)
            {
                double x = evaluateSpline(start_x + x_step*j, a_x[i], b_x[i], c_x[i], d_x[i]);
                double y = evaluateSpline(start_y + y_step*j, a_y[i], b_y[i], c_y[i], d_y[i]);
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
        return a + b * t + c * t * t + d * t * t * t;
    }

    void calculateSplineCoefficients(
        std::vector<double> &point, 
        std::vector<double> &a, std::vector<double> &b, std::vector<double> &c, std::vector<double> &d
    )
    {
        std::vector<double> w;
        int n = static_cast<int>(point.size()-1); 
        for (int i = 0; i <= n; i++)
        {
            a.push_back(point[i]);
        }

        for (int i = 0; i <= n; i++)
        {
            if (i == 0)
            {
                c.push_back(0.0);
            }
            else if (i == n)
            {
                c.push_back(0.0);
            }
            else
            {
                c.push_back(3.0*(a[i - 1] - 2.0*a[i] + a[i + 1]));
            }
        }

        for (int i = 0; i < n; i++)
        {
            if (i == 0)
            {
                w.push_back(0.0);
            }
            else
            {
                double tmp = 4.0 - w[i - 1];
                c[i] = (c[i] - c[i - 1]) / tmp;
                w.push_back(1.0 / tmp);
            }
        }

        for (int i = (n - 1); i > 0; i--)
        {
            c[i] = c[i] - c[i + 1] * w[i];
        }

        for (int i = 0; i <= n; i++) {
            if (i == n)
            {
                d.push_back(0.0);
                b.push_back(0.0);
            }
            else
            {
                d.push_back((c[i + 1] - c[i]) / 3.0);
                b.push_back(a[i + 1] - a[i] - c[i] - d[i]);
            }
        }
    }
};
