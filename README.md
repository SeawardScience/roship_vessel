# roship_vessel

ROS2 package for integrating common shipboard devices — winches, NMEA instruments,
and similar — into the roship ecosystem.  It sits one layer above `roship_io`,
which handles raw transport (serial, UDP, MQTT).  `roship_vessel` turns bytes into
typed ROS messages and diagnostics.

## Architecture

```
roship_io  (serial / UDP / MQTT connection node)
    │  <connection_topic>  [io_interfaces/RawPacket]
    ▼
VesselDeviceNode  — base class (this package)
    │  strip header, filter, parse
    ▼
Device node subclass
    ├─ ~/tension, ~/speed, ~/payout …  [std_msgs/Float64]
    └─ /diagnostics                    [diagnostic_msgs/DiagnosticArray]
```

## Packages

| Package | Contents |
|---------|----------|
| `roship_vessel` | `VesselDeviceNode` base class + device node implementations |
| `roship_vessel_interfaces` | Shared message definitions |

## Build

```bash
colcon build --packages-select roship_vessel_interfaces roship_vessel
```

## Included device nodes

### `lci_winch` — Dynacon RPC/LCI winch monitor

Parses the dsLog multiplexed winch stream (`01RD` records) and publishes:

| Topic | Type | Description |
|-------|------|-------------|
| `~/tension` | `std_msgs/Float64` | Cable tension (pounds) |
| `~/speed` | `std_msgs/Float64` | Pay-in/out speed (m/min) |
| `~/payout` | `std_msgs/Float64` | Cable paid out (meters) |

**Quick start:**

```bash
ros2 run roship_vessel lci_winch --ros-args \
  -p connection_topic:=/udp_device/from_device \
  -p strip_filter:="WNCH*RPC-90x"
```

Or with a launch file:

```bash
ros2 launch roship_vessel lci_winch_udp.launch.xml
```

Key parameters (see `config/lci_winch.yaml` for full list):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `connection_topic` | `""` | `from_device` topic from the roship_io connection node |
| `strip_filter` | `"WNCH*RPC-90x"` | Glob anchor that identifies and strips the dsLog header |
| `validate_checksum` | `true` | Verify the ASCII-sum checksum field |
| `stale_timeout_ms` | `5000` | Age (ms) before the device is declared stale |

## Writing a new device node

See `docs/design_base_class.md` for the full guide.  The short version:

```cpp
class MyDevice : public roship_vessel::VesselDeviceNode {
public:
    MyDevice() : VesselDeviceNode("my_device") {
        if (params_.publish_ros_std)
            pub_ = create_publisher<std_msgs::msg::Float64>("~/value", 10);
    }
protected:
    void onRawData(const io_interfaces::msg::RawPacket::SharedPtr msg) override {
        // parse msg->data, publish
    }
    void onDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) override {
        stat.add("parse_errors", parse_errors_);
    }
};
```

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).
