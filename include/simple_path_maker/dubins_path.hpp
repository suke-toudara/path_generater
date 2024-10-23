#ifndef DUBINS_PATH_HPP
#define DUBINS_PATH_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vector>
#include <cmath>

double mod2pi(double theta) {
    return theta - 2.0 * M_PI * std::floor(theta / (2.0 * M_PI));
}

double pi_2_pi(double angle) {
    while (angle >= M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle <= -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

class DubinsPath
{
public:

  // コンストラクタ
  DubinsPath();
  
  // PoseStamped を渡して Dubins Path を nav_msgs::msg::Path で作成
  nav_msgs::msg::Path generateDubinsPath(
      const geometry_msgs::msg::PoseStamped& start,
      const geometry_msgs::msg::PoseStamped& goal
  );

private:
  std::tuple<double, double, double, std::string> LSL(double alpha, double beta, double d) {
    double sa = std::sin(alpha);
    double sb = std::sin(beta);
    double ca = std::cos(alpha);
    double cb = std::cos(beta);
    double c_ab = std::cos(alpha - beta);
    double tmp0 = d + sa - sb;
    std::string mode = "LSL";
    double p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb));
    if (p_squared < 0) {
        return std::make_tuple(-1, -1, -1, mode); // エラー時
    }
    double tmp1 = std::atan2(cb - ca, tmp0);
    double t = mod2pi(-alpha + tmp1);
    double p = std::sqrt(p_squared);
    double q = mod2pi(beta - tmp1);
    return std::make_tuple(t, p, q, mode);
  }


  double turning_radius_;  // 車両の最小旋回半径

  // 内部で Dubins パスを計算
  std::vector<geometry_msgs::msg::PoseStamped> calculateDubinsPath(
      const geometry_msgs::msg::PoseStamped& start,
      const geometry_msgs::msg::PoseStamped& goal);

  // 座標変換や向きの計算を行う補助関数
  double normalizeAngle(double angle);

}

#endif  // DUBINS_PATH_GENERATOR_HPP
