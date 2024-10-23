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

  std::vector<geometry_msgs::msg::PoseStamped> calculateDubinsPath(
      const geometry_msgs::msg::PoseStamped& start,
      const geometry_msgs::msg::PoseStamped& goal);
  double normalizeAngle(double angle);

  /*
  LSL
  LRL
  LSR
  RSR
  RLR
  RSL
  */
  double DubinsPathGenerator::calculateLSL(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal)
  {
    // 始点と終点の座標と角度を取得
    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;
    double D = std::sqrt(dx * dx + dy * dy);
    double d = D / turning_radius_;
    double theta = std::atan2(dy, dx);
    double alpha = normalizeAngle(start.pose.orientation.z - theta);
    double beta = normalizeAngle(goal.pose.orientation.z - theta);
    return d + std::abs(alpha) + std::abs(beta);  // LSL 経路長
  }

  double DubinsPathGenerator::RSR(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal)
  {
    // 始点と終点の座標と角度を取得
    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;
    double D = std::sqrt(dx * dx + dy * dy);
    double d = D / turning_radius_;
    double theta = std::atan2(dy, dx);
    double alpha = normalizeAngle(start.pose.orientation.z - theta);
    double beta = normalizeAngle(goal.pose.orientation.z - theta);
    return d + std::abs(alpha) + std::abs(beta);  // RSR 経路長
  }

  double DubinsPathGenerator::LSR(
     const geometry_msgs::msg::PoseStamped& start,
     const geometry_msgs::msg::PoseStamped& goal)
  {
    // LSR 計算のロジック
    // （LSLと同様の計算を行い、曲がり方を変更する）
    return 0.0;  // 実際のLSR計算をここに追加
  }

  double DubinsPathGenerator::RSL(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal)
  {
    // RSL 計算のロジック
    // （RSRと同様の計算を行い、曲がり方を変更する）
    return 0.0;  // 実際のRSL計算をここに追加
  }
}

#endif  // DUBINS_PATH_GENERATOR_HPP
