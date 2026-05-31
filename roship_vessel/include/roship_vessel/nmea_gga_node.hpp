#pragma once

#include "package_defs.hpp"
#include "vessel_device_node.hpp"

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <string>
#include <cstdint>

NS_HEAD

class NmeaGgaNode : public VesselDeviceNode
{
public:
    NmeaGgaNode();

protected:
    void onRawData(const io_interfaces::msg::RawPacket::SharedPtr msg) override;
    void onDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) override;

private:
    struct ParsedData {
        double lat       = 0.0;  // decimal degrees
        double lon       = 0.0;  // decimal degrees
        double altitude  = 0.0;  // meters WGS84 (MSL + geoid separation)
        int    fix_quality = 0;
        int    num_sats  = 0;
        double hdop      = 0.0;
    };

    bool parsePayload(const std::string& raw, ParsedData& out);

    ParsedData last_;
    uint64_t parse_errors_    = 0;
    uint64_t checksum_errors_ = 0;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
};

NS_FOOT
