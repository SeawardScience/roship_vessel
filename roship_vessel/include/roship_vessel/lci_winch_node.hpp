#pragma once

#include "vessel_device_node.hpp"

#include <std_msgs/msg/float64.hpp>

#include <cstdint>
#include <string>

NS_HEAD

/**
 * @brief ROS2 node for the Dynacon LCI-90i winch monitor.
 *
 * Subscribes to a roship_io connection that carries multiplexed vessel data
 * and uses strip_filter to extract LCI-90i messages, stripping the
 * "WNCH <timestamp>Z DYNACONLCI90i" header before parsing.
 *
 * Default strip_filter:  "WNCH*DYNACONLCI90i"
 *
 * Message format (payload after header strip):
 *   01RD,<device_ts>,<tension_lbs>,<speed_m_min>,<payout_m>,<checksum>
 *
 * Published topics (all node-private, ~/…):
 *   ~/tension  [std_msgs/Float64]  pounds
 *   ~/speed    [std_msgs/Float64]  m/min  (positive = paying out)
 *   ~/payout   [std_msgs/Float64]  meters
 */
class LciWinchNode : public VesselDeviceNode
{
public:
    LciWinchNode();

protected:
    void onRawData(const io_interfaces::msg::RawPacket::SharedPtr msg) override;
    void onDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) override;

private:
    struct ParsedData {
        double      tension_lbs     = 0.0;
        double      speed_m_per_min = 0.0;
        double      payout_m        = 0.0;
        std::string device_timestamp;
        uint32_t    checksum        = 0;
        bool        checksum_valid  = true;
    };

    bool     validate_checksum_ = true;
    uint64_t parse_errors_      = 0;
    uint64_t checksum_errors_   = 0;
    ParsedData last_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tension_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr payout_pub_;

    bool parsePayload(const std::string& raw, ParsedData& out);
};

NS_FOOT
