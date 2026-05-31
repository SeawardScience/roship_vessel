#pragma once

#include "package_defs.hpp"
#include "vessel_device_node.hpp"

#include <sensor_msgs/msg/imu.hpp>

#include <string>
#include <cstdint>

NS_HEAD

/// Parses NMEA HDT (heading true) and HDG (heading magnetic with deviation/variation)
/// sentences.  Publishes orientation-only sensor_msgs/Imu (REP-103 ENU convention).
/// Angular velocity and linear acceleration covariances are set to -1 (unknown).
class NmeaHdtNode : public VesselDeviceNode
{
public:
    NmeaHdtNode();

protected:
    void onRawData(const io_interfaces::msg::RawPacket::SharedPtr msg) override;
    void onDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) override;

private:
    struct ParsedData {
        double heading_true = 0.0;  // degrees true
        bool   is_hdt       = true; // false = derived from HDG
    };

    bool parsePayload(const std::string& raw, ParsedData& out);

    ParsedData last_;
    uint64_t parse_errors_    = 0;
    uint64_t checksum_errors_ = 0;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

NS_FOOT
