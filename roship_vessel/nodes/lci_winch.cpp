#include "lci_winch_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<roship_vessel::LciWinchNode>());
    rclcpp::shutdown();
    return 0;
}
