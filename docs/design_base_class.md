# roship_vessel — Base Class Design

## Purpose

`roship_vessel` translates raw I/O streams managed by `roship_io` into typed
ROS standard messages for common shipboard devices: winches, NMEA instruments
(GPS, IMU, compass, depth), thrusters, and similar.

It provides `VesselDeviceNode` — a base class that every device node in this
ecosystem extends.

---

## Architecture

```
roship_io connection node          (serial / UDP / MQTT)
    │
    │  <connection_topic>  [io_interfaces/msg/RawPacket]
    ▼
VesselDeviceNode                   (this package)
    │  1. applyStripFilter  — strip prepended header, drop non-matching packets
    │  2. matchesFilter     — optional payload glob filter
    │  3. onRawData()       — subclass parses and publishes typed msgs
    │  4. onPoll()          — optional timer for polled/request-response devices
    │
    ├─ ~/tension, ~/fix, ~/imu …   [std_msgs, sensor_msgs, nav_msgs]
    └─ /diagnostics                [diagnostic_msgs/DiagnosticArray]
```

`roship_io` owns the bytes.  `roship_vessel` owns the meaning.

---

## VesselDeviceNode

**Header:** `roship_vessel/include/roship_vessel/vessel_device_node.hpp`
**Source:** `roship_vessel/src/vessel_device_node.cpp`

### Parameters

All parameters are declared in the constructor and can be updated at runtime
via `ros2 param set`.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `connection_topic` | string | `""` | Full topic name of the roship_io connection's `from_device` output |
| `strip_filter` | string | `""` | Glob anchor for stripping a prepended header (see below) |
| `message_filter` | string | `""` | Glob filter applied to the stripped payload; empty = pass all |
| `publish_ros_std` | bool | `true` | Enable typed topic publishers |
| `publish_diagnostics` | bool | `true` | Enable `/diagnostics` output |
| `hardware_id` | string | `"vessel_device"` | Hardware ID shown in `/diagnostics` |
| `poll_ms` | int | `0` | Polling timer period in ms; 0 = disabled |
| `stale_timeout_ms` | int | `3000` | Age in ms before device is declared stale |

### Override Hooks

| Method | When called | Typical use |
|--------|-------------|-------------|
| `onRawData(msg)` | Every packet that passes both filters | Parse bytes, publish typed msgs |
| `onPoll()` | Every `poll_ms` ms (if > 0) | Send a request to a polled device |
| `onDiagnostics(stat)` | Each `/diagnostics` cycle (~1 Hz) | Add key/value pairs, set severity |

### strip_filter

Many shipboard data streams are multiplexed — multiple device streams share one
serial or UDP connection, with each message prefixed by a header identifying the
source (e.g. dsLog format: `WNCH\t<timestamp>Z\t<device_id>\t<payload>`).

`strip_filter` is a glob pattern.  The literal string after the last `*` or `?`
is the **anchor** — the end of the header.  When a packet is received, the base
class searches for the anchor string.  If found, everything up to and including
the anchor (plus any trailing whitespace) is removed before the packet is
delivered to `onRawData()`.  If the anchor is not found, the packet is dropped.

```
strip_filter:  "WNCH*RPC-90x"
anchor:        "RPC-90x"

raw packet:    "WNCH\t2026-05-29T22:16:53Z\tRPC-90x\t\x1e\x0101RD,..."
onRawData sees:                                        "\x1e\x0101RD,..."
```

### message_filter

Applied to the **stripped payload** after `strip_filter`.  Uses POSIX `fnmatch`
rules: `*` matches any string, `?` matches any single byte (including control
characters).  Empty = pass all packets.

```
# Match only 01RD read records (two control-char bytes then "01RD,")
message_filter: "??01RD,*"
```

### Diagnostics

Two tasks are registered automatically:

**Comms** — connection and data-flow health:
- `OK` — data arriving within `stale_timeout_ms`
- `WARN` — no `connection_topic` set, no data received, or data stale
- Key values: `connection_topic`, `packets_rx`, `packets_matched`, `last_data_age_s`

**Device** — subclass-reported status:
- Calls `onDiagnostics()` so subclasses can add values and set severity
- Automatically overrides to `STALE` when data is stale — subclasses do not
  need to handle this themselves

```cpp
void onDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) override {
    stat.add("parse_errors", parse_errors_);
    if (parse_errors_ > 0)
        stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                          "Parse errors detected");
}
```

### Topic naming

Subclasses publish on node-private topics using the `~/` prefix.  ROS2 expands
`~/foo` to `/<namespace>/<node_name>/foo`.

```
node launched as /vessel/lci_winch
  ~/tension  →  /vessel/lci_winch/tension  [std_msgs/Float64]
  ~/payout   →  /vessel/lci_winch/payout   [std_msgs/Float64]
```

---

## Subclassing Guide

### Data-driven device (e.g. NMEA stream, dsLog winch)

```cpp
class MyDeviceNode : public roship_vessel::VesselDeviceNode {
public:
    MyDeviceNode() : VesselDeviceNode("my_device") {
        // Set defaults before base class setupSubscription runs
        if (params_.strip_filter.empty())
            this->set_parameter(rclcpp::Parameter("strip_filter", "PREFIX*DEVICEID"));

        if (params_.publish_ros_std)
            value_pub_ = create_publisher<std_msgs::msg::Float64>("~/value", 10);
    }

protected:
    void onRawData(const io_interfaces::msg::RawPacket::SharedPtr msg) override {
        // msg->data contains the stripped payload bytes
        std::string payload(msg->data.begin(), msg->data.end());
        // parse, publish
    }

    void onDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) override {
        stat.add("parse_errors", parse_errors_);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr value_pub_;
    uint64_t parse_errors_ = 0;
};
```

### Polled device (e.g. request/response protocol)

Set `poll_ms > 0` via YAML.  Use `onPoll()` to send requests and `onRawData()`
to handle responses.

```cpp
class PolledDevice : public roship_vessel::VesselDeviceNode {
protected:
    void onPoll() override {
        // publish a command to the roship_io to_device topic
    }
    void onRawData(const io_interfaces::msg::RawPacket::SharedPtr msg) override {
        // parse response
    }
};
```

---

## Implemented Device Nodes

### `lci_winch` — Dynacon RPC/LCI winch monitor

**Source:** `include/roship_vessel/lci_winch_node.hpp`, `src/lci_winch_node.cpp`
**Executable:** `nodes/lci_winch.cpp`
**Config:** `config/lci_winch.yaml`
**Launch:** `launch/lci_winch_udp.launch.xml`

Parses the dsLog `01RD` winch record format:

```
<stripped payload>  →  \x1e\x01 01RD , <device_ts> , <tension> , <speed> , <payout> , <checksum>
```

| Field | Units | Topic |
|-------|-------|-------|
| tension | pounds | `~/tension` |
| speed | m/min (positive = paying out) | `~/speed` |
| payout | meters | `~/payout` |

Checksum is an ASCII-sum of all bytes from the start of the payload through the
last comma (inclusive).  Set `validate_checksum: false` to silence warnings on
noisy links.

Control characters `\x1e\x01` prefixed to the message ID are stripped
automatically before parsing.

---

## Build

```bash
colcon build --packages-select roship_vessel_interfaces roship_vessel
```

## Launch (params file)

```bash
# Note: --ros-args is required; --params-file alone is silently ignored
ros2 run roship_vessel lci_winch --ros-args --params-file config/lci_winch.yaml
```

```bash
ros2 launch roship_vessel lci_winch_udp.launch.xml vessel_ns:=nautilus
```
