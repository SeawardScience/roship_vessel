#include "lci_winch_node.hpp"

#include <sstream>
#include <vector>

NS_HEAD

// ── checksum ──────────────────────────────────────────────────────────────────
// Sum of ASCII values of all bytes from the start of the payload through the
// last comma (inclusive).  Control characters stripped before calling are
// therefore excluded.
static uint32_t computeChecksum(const std::string& data)
{
    size_t last_comma = data.rfind(',');
    if (last_comma == std::string::npos) return 0;
    uint32_t sum = 0;
    for (size_t i = 0; i <= last_comma; ++i)
        sum += static_cast<uint8_t>(data[i]);
    return sum;
}

// ── LciWinchNode ──────────────────────────────────────────────────────────────

LciWinchNode::LciWinchNode()
    : VesselDeviceNode("lci_winch")
{
    // Apply sensible defaults unless the user has already set them via YAML
    if (params_.strip_filter.empty())
        this->set_parameter(rclcpp::Parameter("strip_filter",     "WNCH*DYNACONLCI90i"));
    if (params_.hardware_id == "vessel_device")
        this->set_parameter(rclcpp::Parameter("hardware_id",      "Dynacon LCI-90i"));
    if (params_.stale_timeout_ms == 3000)
        this->set_parameter(rclcpp::Parameter("stale_timeout_ms", 5000));

    this->declare_parameter("validate_checksum", validate_checksum_);
    this->get_parameter("validate_checksum", validate_checksum_);

    if (params_.publish_ros_std) {
        tension_pub_ = create_publisher<std_msgs::msg::Float64>("~/tension", 10);
        speed_pub_   = create_publisher<std_msgs::msg::Float64>("~/speed",   10);
        payout_pub_  = create_publisher<std_msgs::msg::Float64>("~/payout",  10);
    }
}

// ── parsing ───────────────────────────────────────────────────────────────────

bool LciWinchNode::parsePayload(const std::string& raw, ParsedData& out)
{
    // Strip leading non-printable bytes (e.g. <0x1e><0x01> framing chars)
    size_t start = 0;
    while (start < raw.size() && static_cast<uint8_t>(raw[start]) < 0x20)
        ++start;
    std::string payload = raw.substr(start);

    // Split on comma
    std::vector<std::string> fields;
    {
        std::istringstream ss(payload);
        std::string tok;
        while (std::getline(ss, tok, ','))
            fields.push_back(tok);
    }

    if (fields.size() < 6) {
        RCLCPP_WARN(get_logger(),
            "LCI-90i: expected 6 fields, got %zu — dropping", fields.size());
        ++parse_errors_;
        return false;
    }

    // Field 0: message ID must contain "01RD"
    if (fields[0].find("01RD") == std::string::npos) {
        RCLCPP_WARN(get_logger(),
            "LCI-90i: unexpected message ID '%s' — dropping", fields[0].c_str());
        ++parse_errors_;
        return false;
    }

    try {
        out.tension_lbs      = std::stod(fields[2]);
        out.speed_m_per_min  = std::stod(fields[3]);
        out.payout_m         = std::stod(fields[4]);
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "LCI-90i: numeric parse error — %s", e.what());
        ++parse_errors_;
        return false;
    }

    out.device_timestamp = fields[1];

    try {
        out.checksum = static_cast<uint32_t>(std::stoul(fields[5]));
    } catch (...) {
        out.checksum = 0;
    }

    if (validate_checksum_) {
        uint32_t computed = computeChecksum(payload);
        out.checksum_valid = (computed == out.checksum);
        if (!out.checksum_valid) {
            // RCLCPP_WARN(get_logger(),
            //     "LCI-90i: checksum mismatch — got %u, computed %u",
            //     out.checksum, computed);
            ++checksum_errors_;
        }
    } else {
        out.checksum_valid = true;
    }

    return true;
}

// ── callbacks ─────────────────────────────────────────────────────────────────

void LciWinchNode::onRawData(const io_interfaces::msg::RawPacket::SharedPtr msg)
{
    std::string payload(msg->data.begin(), msg->data.end());

    ParsedData d;
    if (!parsePayload(payload, d)) return;
    last_ = d;

    if (!tension_pub_) return;

    std_msgs::msg::Float64 val;
    val.data = d.tension_lbs;     tension_pub_->publish(val);
    val.data = d.speed_m_per_min; speed_pub_->publish(val);
    val.data = d.payout_m;        payout_pub_->publish(val);
}

void LciWinchNode::onDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    stat.add("tension_lbs",      last_.tension_lbs);
    stat.add("speed_m_per_min",  last_.speed_m_per_min);
    stat.add("payout_m",         last_.payout_m);
    stat.add("device_timestamp", last_.device_timestamp);
    stat.add("parse_errors",     parse_errors_);
    stat.add("checksum_errors",  checksum_errors_);

    if (checksum_errors_ > 0)
        stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                          "Checksum errors detected");
    if (parse_errors_ > 0)
        stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                          "Parse errors detected");
}

NS_FOOT
