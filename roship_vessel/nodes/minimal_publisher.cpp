#include "minimal_publisher_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roship_vessel::MinimalPublisherNode>());
  rclcpp::shutdown();
  return 0;

}
