#include "nmea_gga_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<roship_vessel::NmeaGgaNode>());
    rclcpp::shutdown();
    return 0;
}
