#pragma once

#include "package_defs.hpp"

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <io_interfaces/msg/raw_packet.hpp>

#include <string>
#include <memory>
#include <cstdint>

NS_HEAD

/**
 * @brief Base class for all shipboard device nodes.
 *
 * Handles parameter management, /diagnostics publishing, and raw-data
 * subscription to a roship_io connection node.  Subclasses override the
 * virtual hooks to parse device data and publish typed ROS messages.
 *
 * Diagnostics tasks
 * -----------------
 * "Comms"  — owned by this base class.  Reports WARN if no data has been
 *             received yet, or if the last packet is older than stale_timeout_ms.
 *             Reports OK while data is flowing.
 *
 * "Device" — calls onDiagnostics() so subclasses can add key/values and set
 *             their own severity.  Overrides to STALE automatically when the
 *             comms link is stale, regardless of what onDiagnostics() set.
 *
 * Topic naming convention
 * -----------------------
 * Standard ROS messages are published as node-private topics using the `~/`
 * prefix, which ROS2 expands to `<namespace>/<node_name>/<topic>`.
 * Example: a node named `/vessel/gps` publishes `~/fix` → `/vessel/gps/fix`.
 *
 * Minimal subclass
 * ----------------
 * @code
 * class MyDevice : public roship_vessel::VesselDeviceNode {
 * public:
 *     MyDevice() : VesselDeviceNode("my_device") {}
 * protected:
 *     void onRawData(const io_interfaces::msg::RawPacket::SharedPtr msg) override {
 *         // parse msg->data, publish typed messages
 *     }
 *     void onDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) override {
 *         stat.add("packets_rx", packets_rx_);
 *     }
 * };
 * @endcode
 */
class VesselDeviceNode : public rclcpp::Node
{
public:
    /**
     * @brief All configurable parameters for VesselDeviceNode.
     */
    struct Params {
        /// roship_io connection topic to subscribe to (e.g. /serial_gps/from_device).
        /// Leave empty to run without a raw-data source.
        std::string connection_topic = "";

        /// Publish typed sensor_msgs / nav_msgs on ~/… topics.
        bool publish_ros_std = true;

        /// Publish connection and device health to /diagnostics.
        bool publish_diagnostics = true;

        /// Hardware ID string shown in /diagnostics output.
        std::string hardware_id = "vessel_device";

        /// Polling timer interval in ms.  0 = data-driven only (no timer).
        int poll_ms = 0;

        /// Age in ms after which a device is considered stale.
        /// Comms task goes WARN; Device task goes STALE.
        int stale_timeout_ms = 3000;

        /// Glob filter applied to the string representation of incoming packet
        /// data.  Only packets whose data matches are forwarded to onRawData()
        /// and count toward staleness.  Empty = pass all packets.
        ///
        /// Uses POSIX fnmatch rules:
        ///   *   matches any sequence of characters
        ///   ?   matches any single character
        ///   […] matches a character class
        ///
        /// Examples (NMEA):
        ///   "$GPGGA,*"   — only GGA sentences from a GP talker
        ///   "$GP*"       — all GP talker sentences
        ///   "$??GGA,*"   — GGA from any two-character talker
        std::string message_filter = "";

        /// Strip-and-filter: glob pattern whose non-wildcard suffix ("anchor")
        /// marks the end of a prepended header.  When a packet matches, everything
        /// up to and including the anchor is removed before the payload is passed
        /// to onRawData().  Packets that do not match are dropped (same as
        /// message_filter).  Empty = disabled.
        ///
        /// The anchor is the literal string after the last * or ? in the pattern.
        /// A single optional whitespace character after the anchor is also stripped.
        ///
        /// Example:
        ///   data:         "WINCH 2020-10-20T00:00:12Z DYNACONLCI90i 01RD,..."
        ///   strip_filter: "WINCH*DYNACONLCI90i"
        ///   anchor:       "DYNACONLCI90i"
        ///   onRawData sees: "01RD,..."
        std::string strip_filter = "";

        void declare(rclcpp::Node* node);
        void update(rclcpp::Node* node);
    };

    explicit VesselDeviceNode(const std::string& node_name);

    /// True when data has been received recently (within stale_timeout_ms).
    bool isConnected() const;

protected:
    Params params_;
    diagnostic_updater::Updater diagnostics_;

    /// True when the last received packet is older than stale_timeout_ms,
    /// or no packet has been received yet.
    bool isDataStale() const;

    // ── override hooks ───────────────────────────────────────────────────────

    /// Called for every RawPacket received from the roship_io connection.
    virtual void onRawData(const io_interfaces::msg::RawPacket::SharedPtr msg) { (void)msg; }

    /// Called periodically when params_.poll_ms > 0.  Use for active polling
    /// devices (e.g. Modbus, request/response protocols).
    virtual void onPoll() {}

    /// Called inside the "Device" /diagnostics task.  Append device-specific
    /// key/value pairs or set the status level here.  The summary will be
    /// overridden to STALE if the comms link is stale — no need to handle
    /// that case manually.
    virtual void onDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) { (void)stat; }

private:
    bool     data_received_   = false;
    rclcpp::Time last_data_time_;
    uint64_t packets_rx_      = 0;   ///< Total packets received on connection_topic
    uint64_t packets_matched_ = 0;   ///< Packets that passed message_filter
    std::string last_payload_;        ///< Last payload string delivered to onRawData()

    rclcpp::Subscription<io_interfaces::msg::RawPacket>::SharedPtr data_sub_;
    rclcpp::TimerBase::SharedPtr poll_timer_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_;

    bool matchesFilter(const io_interfaces::msg::RawPacket::SharedPtr& msg) const;
    /// Returns the stripped payload packet, or nullptr if strip_filter is set
    /// but the packet does not match (caller should drop the packet).
    io_interfaces::msg::RawPacket::SharedPtr applyStripFilter(
        const io_interfaces::msg::RawPacket::SharedPtr& msg) const;
    void setupSubscription();
    void setupPollTimer();
    void commsCallback(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void deviceCallback(diagnostic_updater::DiagnosticStatusWrapper& stat);
};

NS_FOOT
