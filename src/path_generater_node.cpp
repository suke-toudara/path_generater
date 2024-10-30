#include <path_generater/path_generater.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<path_generater::PathGenerater>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}