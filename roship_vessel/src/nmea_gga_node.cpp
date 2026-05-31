#include "nmea_gga_node.hpp"
#include "nmea_utils.hpp"

NS_HEAD

NmeaGgaNode::NmeaGgaNode()
    : VesselDeviceNode("nmea_gga")
{
    if (params_.message_filter.empty())
        this->set_parameter(rclcpp::Parameter("message_filter", "*GGA,*"));

    if (params_.publish_ros_std)
        fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("~/fix", 10);
}

bool NmeaGgaNode::parsePayload(const std::string& raw, ParsedData& out)
{
    std::string sentence;
    if (!nmea::findSentence(raw, sentence)) {
        RCLCPP_WARN(get_logger(), "GGA: no NMEA sentence found — dropping");
        ++parse_errors_;
        return false;
    }

    if (!nmea::validateChecksum(sentence)) {
        RCLCPP_WARN(get_logger(), "GGA: checksum invalid — dropping");
        ++checksum_errors_;
        return false;
    }

    auto f = nmea::split(sentence);

    // $XXGGA,time,lat,N/S,lon,E/W,quality,sats,hdop,alt,M,geoid,M,,*hh
    if (f.size() < 10) {
        RCLCPP_WARN(get_logger(), "GGA: too few fields (%zu) — dropping", f.size());
        ++parse_errors_;
        return false;
    }

    // Sentence type contains "GGA"
    if (f[0].find("GGA") == std::string::npos) {
        RCLCPP_WARN(get_logger(), "GGA: unexpected sentence type '%s' — dropping", f[0].c_str());
        ++parse_errors_;
        return false;
    }

    try {
        out.fix_quality = f[6].empty() ? 0 : std::stoi(f[6]);
        if (out.fix_quality == 0) return false;  // no fix, silently skip

        out.lat  = nmea::coordToDecimal(f[2], f[3].empty() ? 'N' : f[3][0]);
        out.lon  = nmea::coordToDecimal(f[4], f[5].empty() ? 'E' : f[5][0]);
        out.num_sats = f[7].empty() ? 0 : std::stoi(f[7]);
        out.hdop     = f[8].empty() ? 0.0 : std::stod(f[8]);

        double alt_msl   = f[9].empty()  ? 0.0 : std::stod(f[9]);
        double geoid_sep = (f.size() > 11 && !f[11].empty()) ? std::stod(f[11]) : 0.0;
        out.altitude = alt_msl + geoid_sep;  // WGS84
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "GGA: parse error — %s", e.what());
        ++parse_errors_;
        return false;
    }

    return true;
}

void NmeaGgaNode::onRawData(const io_interfaces::msg::RawPacket::SharedPtr msg)
{
    std::string payload(msg->data.begin(), msg->data.end());

    ParsedData d;
    if (!parsePayload(payload, d)) return;
    last_ = d;

    if (!fix_pub_) return;

    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp    = this->now();
    fix.header.frame_id = "world";
    fix.latitude        = d.lat;
    fix.longitude       = d.lon;
    fix.altitude        = d.altitude;
    fix.status.status   = nmea::fixQualityToStatus(d.fix_quality);
    fix.status.service  = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    fix_pub_->publish(fix);
}

void NmeaGgaNode::onDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    stat.add("lat",              last_.lat);
    stat.add("lon",              last_.lon);
    stat.add("altitude_m",       last_.altitude);
    stat.add("fix_quality",      last_.fix_quality);
    stat.add("num_satellites",   last_.num_sats);
    stat.add("hdop",             last_.hdop);
    stat.add("parse_errors",     parse_errors_);
    stat.add("checksum_errors",  checksum_errors_);

    if (checksum_errors_ > 0)
        stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Checksum errors");
    if (parse_errors_ > 0)
        stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Parse errors");
}

NS_FOOT
