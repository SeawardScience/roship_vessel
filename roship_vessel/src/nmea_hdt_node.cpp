#include "nmea_hdt_node.hpp"
#include "nmea_utils.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

NS_HEAD

static constexpr double DEG_TO_RAD = M_PI / 180.0;

NmeaHdtNode::NmeaHdtNode()
    : VesselDeviceNode("nmea_hdt")
{
    // Matches HDT (true) and HDG (magnetic with deviation/variation)
    if (params_.message_filter.empty())
        this->set_parameter(rclcpp::Parameter("message_filter", "*HD[TG],*"));

    if (params_.publish_ros_std)
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("~/imu", 10);
}

bool NmeaHdtNode::parsePayload(const std::string& raw, ParsedData& out)
{
    std::string sentence;
    if (!nmea::findSentence(raw, sentence)) {
        RCLCPP_WARN(get_logger(), "HDT: no NMEA sentence found — dropping");
        ++parse_errors_;
        return false;
    }

    if (!nmea::validateChecksum(sentence)) {
        RCLCPP_WARN(get_logger(), "HDT: checksum invalid — dropping");
        ++checksum_errors_;
        return false;
    }

    auto f = nmea::split(sentence);
    if (f.size() < 2) {
        RCLCPP_WARN(get_logger(), "HDT: too few fields — dropping");
        ++parse_errors_;
        return false;
    }

    bool is_hdt = f[0].find("HDT") != std::string::npos;
    bool is_hdg = f[0].find("HDG") != std::string::npos;

    if (!is_hdt && !is_hdg) {
        RCLCPP_WARN(get_logger(), "HDT: unexpected sentence type '%s' — dropping", f[0].c_str());
        ++parse_errors_;
        return false;
    }

    try {
        if (is_hdt) {
            // $XXHDT,hhh.hh,T
            out.heading_true = std::stod(f[1]);
            out.is_hdt = true;
        } else {
            // $XXHDG,compass,dev,E/W,var,E/W
            if (f.size() < 5) {
                RCLCPP_WARN(get_logger(), "HDG: too few fields — dropping");
                ++parse_errors_;
                return false;
            }
            double compass   = std::stod(f[1]);
            double deviation = f[2].empty() ? 0.0 : std::stod(f[2]);
            double variation = f[4].empty() ? 0.0 : std::stod(f[4]);
            char   dev_dir   = (f.size() > 3 && !f[3].empty()) ? f[3][0] : 'E';
            char   var_dir   = (f.size() > 5 && !f[5].empty()) ? f[5][0] : 'E';
            // True = Compass + Deviation + Variation (east positive)
            double dev_signed = (dev_dir == 'E') ?  deviation : -deviation;
            double var_signed = (var_dir == 'E') ?  variation : -variation;
            out.heading_true = compass + dev_signed + var_signed;
            out.is_hdt = false;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "HDT/HDG: parse error — %s", e.what());
        ++parse_errors_;
        return false;
    }

    // Normalise to [0, 360)
    out.heading_true = std::fmod(out.heading_true, 360.0);
    if (out.heading_true < 0.0) out.heading_true += 360.0;

    return true;
}

void NmeaHdtNode::onRawData(const io_interfaces::msg::RawPacket::SharedPtr msg)
{
    std::string payload(msg->data.begin(), msg->data.end());

    ParsedData d;
    if (!parsePayload(payload, d)) return;
    last_ = d;

    if (!imu_pub_) return;

    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = this->now();
    imu.header.frame_id = "base_link";

    // REP-103 ENU: compass heading (0=N, CW+) → yaw (0=E, CCW+)
    double yaw_rad = (90.0 - d.heading_true) * DEG_TO_RAD;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);
    imu.orientation = tf2::toMsg(q);

    imu.angular_velocity_covariance[0]    = -1.0;
    imu.linear_acceleration_covariance[0] = -1.0;

    imu_pub_->publish(imu);
}

void NmeaHdtNode::onDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    stat.add("heading_true_deg", last_.heading_true);
    stat.add("source",           last_.is_hdt ? "HDT" : "HDG (computed)");
    stat.add("parse_errors",     parse_errors_);
    stat.add("checksum_errors",  checksum_errors_);

    if (checksum_errors_ > 0)
        stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Checksum errors");
    if (parse_errors_ > 0)
        stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Parse errors");
}

NS_FOOT
