#include "vessel_device_node.hpp"

#include <fnmatch.h>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

NS_HEAD

// ── Params ───────────────────────────────────────────────────────────────────

void VesselDeviceNode::Params::declare(rclcpp::Node* node)
{
    node->declare_parameter("connection_topic",    connection_topic);
    node->declare_parameter("publish_ros_std",     publish_ros_std);
    node->declare_parameter("publish_diagnostics", publish_diagnostics);
    node->declare_parameter("hardware_id",         hardware_id);
    node->declare_parameter("poll_ms",             poll_ms);
    node->declare_parameter("stale_timeout_ms",    stale_timeout_ms);
    node->declare_parameter("message_filter",      message_filter);
    node->declare_parameter("strip_filter",        strip_filter);
}

void VesselDeviceNode::Params::update(rclcpp::Node* node)
{
    node->get_parameter("connection_topic",    connection_topic);
    node->get_parameter("publish_ros_std",     publish_ros_std);
    node->get_parameter("publish_diagnostics", publish_diagnostics);
    node->get_parameter("hardware_id",         hardware_id);
    node->get_parameter("poll_ms",             poll_ms);
    node->get_parameter("stale_timeout_ms",    stale_timeout_ms);
    node->get_parameter("message_filter",      message_filter);
    node->get_parameter("strip_filter",        strip_filter);
}

// ── VesselDeviceNode ─────────────────────────────────────────────────────────

VesselDeviceNode::VesselDeviceNode(const std::string& node_name)
    : Node(node_name), diagnostics_(this)
{
    params_.declare(this);
    params_.update(this);

    diagnostics_.setHardwareID(params_.hardware_id);
    diagnostics_.add("Comms",  this, &VesselDeviceNode::commsCallback);
    diagnostics_.add("Device", this, &VesselDeviceNode::deviceCallback);

    setupSubscription();
    setupPollTimer();

    param_cb_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            bool reconnect = false;
            for (const auto& p : params) {
                if (p.get_name() == "connection_topic") {
                    params_.connection_topic = p.as_string();
                    reconnect = true;
                } else if (p.get_name() == "publish_ros_std") {
                    params_.publish_ros_std = p.as_bool();
                } else if (p.get_name() == "publish_diagnostics") {
                    params_.publish_diagnostics = p.as_bool();
                } else if (p.get_name() == "hardware_id") {
                    params_.hardware_id = p.as_string();
                    diagnostics_.setHardwareID(params_.hardware_id);
                } else if (p.get_name() == "poll_ms") {
                    params_.poll_ms = static_cast<int>(p.as_int());
                    setupPollTimer();
                } else if (p.get_name() == "stale_timeout_ms") {
                    params_.stale_timeout_ms = static_cast<int>(p.as_int());
                } else if (p.get_name() == "message_filter") {
                    params_.message_filter = p.as_string();
                } else if (p.get_name() == "strip_filter") {
                    params_.strip_filter = p.as_string();
                }
            }

            if (reconnect) {
                data_received_   = false;
                packets_rx_      = 0;
                packets_matched_ = 0;
                setupSubscription();
            }

            return result;
        });
}

// ── helpers ───────────────────────────────────────────────────────────────────

// Returns the literal string after the last wildcard in a glob pattern.
// This is the "anchor" that marks the end of the header to strip.
static std::string stripAnchor(const std::string& pattern)
{
    size_t last_wild = pattern.find_last_of("*?");
    if (last_wild == std::string::npos) return pattern;  // no wildcards — whole pattern is literal
    return pattern.substr(last_wild + 1);
}

bool VesselDeviceNode::matchesFilter(const io_interfaces::msg::RawPacket::SharedPtr& msg) const
{
    if (params_.message_filter.empty()) return true;
    // Interpret raw bytes as a string for pattern matching.
    // fnmatch stops at the first null byte, which is fine for text protocols.
    std::string data_str(msg->data.begin(), msg->data.end());
    return fnmatch(params_.message_filter.c_str(), data_str.c_str(), FNM_NOESCAPE) == 0;
}

io_interfaces::msg::RawPacket::SharedPtr VesselDeviceNode::applyStripFilter(
    const io_interfaces::msg::RawPacket::SharedPtr& msg) const
{
    if (params_.strip_filter.empty()) return msg;

    std::string anchor = stripAnchor(params_.strip_filter);
    if (anchor.empty()) {
        // strip_filter ends with a wildcard — no fixed anchor, pass through unchanged
        return msg;
    }

    std::string data_str(msg->data.begin(), msg->data.end());

    size_t pos = data_str.find(anchor);
    if (pos == std::string::npos) {
        return nullptr;  // anchor not found — not our message, drop it
    }

    size_t payload_start = pos + anchor.size();
    // Skip any whitespace separator between header and payload (space or tab)
    while (payload_start < data_str.size() &&
           (data_str[payload_start] == ' ' || data_str[payload_start] == '\t')) {
        ++payload_start;
    }

    auto stripped = std::make_shared<io_interfaces::msg::RawPacket>();
    stripped->header = msg->header;
    stripped->data.assign(
        msg->data.begin() + static_cast<std::ptrdiff_t>(payload_start),
        msg->data.end());
    return stripped;
}

bool VesselDeviceNode::isDataStale() const
{
    if (!data_received_) return true;
    double age_ms = (this->now() - last_data_time_).nanoseconds() / 1e6;
    return age_ms > static_cast<double>(params_.stale_timeout_ms);
}

bool VesselDeviceNode::isConnected() const
{
    return !isDataStale();
}

// ── private setup ─────────────────────────────────────────────────────────────

void VesselDeviceNode::setupSubscription()
{
    data_sub_.reset();

    if (params_.connection_topic.empty()) {
        RCLCPP_WARN(get_logger(),
            "connection_topic is empty — no raw data subscription created");
        return;
    }

    data_sub_ = this->create_subscription<io_interfaces::msg::RawPacket>(
        params_.connection_topic, 10,
        [this](const io_interfaces::msg::RawPacket::SharedPtr msg) {
            ++packets_rx_;
            auto payload = applyStripFilter(msg);
            if (!payload) return;
            if (!matchesFilter(payload)) return;
            ++packets_matched_;
            last_data_time_ = this->now();
            data_received_  = true;
            last_payload_.assign(payload->data.begin(), payload->data.end());
            onRawData(payload);
        });

    RCLCPP_INFO(get_logger(), "Subscribed to %s", params_.connection_topic.c_str());
}

void VesselDeviceNode::setupPollTimer()
{
    poll_timer_.reset();

    if (params_.poll_ms <= 0) return;

    poll_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(params_.poll_ms),
        [this]() { onPoll(); });
}

// ── diagnostics ───────────────────────────────────────────────────────────────

void VesselDeviceNode::commsCallback(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    if (!params_.publish_diagnostics) {
        stat.summary(DiagStatus::OK, "Diagnostics disabled");
        return;
    }

    if (params_.connection_topic.empty()) {
        stat.summary(DiagStatus::WARN, "No connection_topic set");
    } else if (!data_received_) {
        stat.summary(DiagStatus::WARN, "No data received");
    } else {
        double age_s = (this->now() - last_data_time_).nanoseconds() / 1e9;
        stat.add("last_data_age_s", age_s);

        if (age_s * 1000.0 > static_cast<double>(params_.stale_timeout_ms)) {
            std::ostringstream msg;
            msg << "Data stale (" << std::fixed << std::setprecision(1) << age_s << " s)";
            stat.summary(DiagStatus::WARN, msg.str());
        } else {
            stat.summary(DiagStatus::OK, "Receiving data");
        }
    }

    stat.add("connection_topic",  params_.connection_topic);
    stat.add("packets_rx",        packets_rx_);
    stat.add("packets_matched",   packets_matched_);
    stat.add("last_payload",      last_payload_);
}

void VesselDeviceNode::deviceCallback(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    if (!params_.publish_diagnostics) {
        stat.summary(DiagStatus::OK, "Diagnostics disabled");
        return;
    }

    // Default summary — subclass may override in onDiagnostics()
    stat.summary(DiagStatus::OK, "OK");
    onDiagnostics(stat);

    // Stale data trumps whatever the subclass reported
    if (isDataStale()) {
        stat.summary(DiagStatus::STALE, "Data stale");
    }
}

NS_FOOT
