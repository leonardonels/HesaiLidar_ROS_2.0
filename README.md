# HesaiLidar ROS 2 Driver

ROS 2 driver for Hesai LiDAR sensors (OT128, XT, AT, QT, Pandar, FT, JT series).
Built on [HesaiLidar_SDK_2.0](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0).

## System Requirements

- **Ubuntu 22.04** with **ROS 2 Humble**
- Boost: `sudo apt-get install libboost-all-dev`
- YAML: `sudo apt-get install libyaml-cpp-dev`
- CUDA (optional, for GPU acceleration): tested with CUDA 11.x+

## Supported LiDAR Models

| Pandar       | OT    | QT       | XT          | AT       | FT    | JT    |
|:-------------|:------|:---------|:------------|:---------|:------|:------|
| Pandar40P    | OT128 | PandarQT | PandarXT    | AT128E2X | FT120 | JT16  |
| Pandar64     | -     | QT128C2X | PandarXT-16 | AT128P   | -     | JT128 |
| Pandar128E3X | -     | -        | XT32M2X     | ATX      | -     | -     |

## Installation & Build

Clone the repository (with submodules):

    git clone --recurse-submodules https://github.com/leonardonels/HesaiLidar_ROS_2.0.git

From the ROS 2 workspace root, build with:

    colcon build --packages-select hesai_ros_driver --symlink-install
    source install/setup.bash

Launch the driver with:

    ros2 launch hesai_ros_driver hesai.launch.py

## Configuration

The driver is configured via `config/config.yaml`. Here is a minimal example for live LiDAR:

```yaml
rviz2: false
lidar:
  - driver:
      use_gpu: true
      source_type: 1                                  # 1=live LiDAR, 2=pcap, 3=rosbag, 4=serial
      lidar_udp_type:
        device_ip_address: 192.168.1.201
        udp_port: 2368
        ptc_port: 9347
        use_ptc_connected: true
        correction_file_path: "/path/to/correction.csv"
        firetimes_path: "/path/to/firetime.csv"

      use_timestamp_type: 0
      transform_flag: false
      x: 0
      y: 0
      z: 1.1
      roll: 0
      pitch: 0
      yaw: 0
      enable_packet_loss_tool: true
      distance_correction_flag: false

    ros:
      ros_frame_id: hesai_lidar
      ros_send_point_cloud_topic: /lidar_points
      ros_send_imu_topic: /lidar_imu
      ros_send_ptp_topic: /lidar_ptp
      send_packet_ros: false
      send_point_cloud_ros: true
      send_imu_ros: true
      real_time_timestamp: true

      # Temperature publishing (OT128 example)
      temperature:
        enable: false                                 # Set to true to enable
        source: auto                                  # auto | ptc | udp_tail
        topic: /lidar/temperature
        publish_sensor_msgs: false                    # Also publish per-sensor Temperature msgs
        sensor_msgs_prefix: /lidar/temperature/
        ptc_poll_period_s: 1.0
        warn_c: 75.0
        error_c: 90.0
        include_imu: true
        udp_status_id_map: {}                         # See "Temperature Publishing" section below
```

## Usage Modes

### Live LiDAR

Set `source_type: 1` and configure `lidar_udp_type`. Ensure `correction_file_path` points to the
calibration file to avoid parsing failures. Pre-check IP reachability:

    ping 192.168.1.201

Then launch:

    ros2 launch hesai_ros_driver hesai.launch.py

### PCAP Replay

Set `source_type: 2` and configure `pcap_type`:

```yaml
pcap_type:
  pcap_path: "/path/to/file.pcap"
  correction_file_path: "/path/to/correction.csv"
  firetimes_path: "/path/to/firetime.csv"
  pcap_play_synchronization: true
  pcap_play_in_loop: true
  play_rate_: 1.0
```

### ROS Bag Playback

Set `source_type: 3` and configure `rosbag_type`:

```yaml
rosbag_type:
  correction_file_path: "/path/to/correction.csv"
  firetimes_path: "/path/to/firetime.csv"
```

Record a rosbag from live or pcap:

    ros2 bag record /lidar_packets

Playback:

    ros2 bag play path/to/rosbag_folder

### Serial Port (JT series)

Set `source_type: 4` and configure `serial_type` with the device port names.



## Temperature Publishing (OT128)

This driver publishes OT128 board and IMU temperatures to ROS 2 via `diagnostic_msgs/DiagnosticArray`.
The feature is **disabled by default** and is configured in the `temperature:` block under `ros:`.

### How it works

The OT128 exposes temperatures from two independent sources depending on the input mode:

| Source       | When Used | Transport | Data |
|:-------------|:----------|:----------|:-----|
| **PTC**      | Live LiDAR (`source_type: 1 or 5`) | TCP port 9347 | 8 board temps from `GetLidarStatus (0x09)` |
| **UDP Tail** | PCAP/RosBag (`source_type: 2 or 3`) | Part of every UDP packet | Rotating 3 status fields + IMU temp |

The driver **auto-selects** the appropriate path via `source: auto`. You can force a path with
`source: ptc` or `source: udp_tail`; forcing an incompatible path logs a warning and disables
the feature gracefully.

### Configuration

Enable and configure in `config.yaml`:

```yaml
temperature:
  enable: true                                  # Master on/off
  source: auto                                  # auto | ptc | udp_tail
  topic: /lidar/temperature                     # DiagnosticArray topic
  publish_sensor_msgs: true                     # (Optional) per-sensor Temperature msgs
  sensor_msgs_prefix: /lidar/temperature/       # Prefix for per-sensor topics
  
  # PTC-specific (live only)
  ptc_poll_period_s: 1.0                        # Poll interval; temperature changes slowly
  
  # Thresholds (both paths)
  warn_c: 75.0                                  # DiagnosticStatus WARN level
  error_c: 90.0                                 # DiagnosticStatus ERROR level
  
  # UDP-tail specific (pcap/rosbag only)
  include_imu: true                             # Include IMU temperature if present
  udp_status_id_map:                            # Status ID → label/scale/offset
    # Example (fill from OT128 manual §3.1.2.5):
    # 1: { label: board_t1, scale: 0.01, offset: 0.0 }
    # 2: { label: board_t2, scale: 0.01, offset: 0.0 }
```

### Topics

**Primary:** one `diagnostic_msgs/DiagnosticArray` on `/lidar/temperature` (configurable).
- Contains one `DiagnosticStatus` with `hardware_id = "OT128"`
- One `KeyValue` per sensor (label, °C value)
- `level` = OK / WARN / ERROR based on hottest sensor

**Optional:** set `publish_sensor_msgs: true` for per-sensor `sensor_msgs/Temperature` msgs
on `/lidar/temperature/<label>`.

### Inspection

View diagnostics:

    ros2 topic echo /lidar/temperature

Monitor with rqt:

    rqt_runtime_monitor  # Integrates with DiagnosticArray

### Limitations & Notes

- **PTC path**: the 49-byte `0x09` response layout is based on **XT32M1X** documentation and
  is **not verified for OT128**. The code performs defensive size checks to fail loudly if the
  layout differs. When hardware available, dump and verify the layout against the OT128 PTC
  manual before trusting temperature values.
- **UDP-tail path**: requires the `udp_status_id_map` from the OT128 manual. Empty by default
  (fail-safe); unknown status IDs publish nothing.
- The PTC channel shares a single TCP socket with PTP diagnostics and correction updates.
  Polling is rate-limited to 1 Hz (`ptc_poll_period_s`) to avoid congestion.

## Multi-LiDAR Fusion

Run multiple drivers from a single launch by stacking multiple driver blocks in `config.yaml`:

```yaml
lidar:
  - driver:
      use_gpu: true
      source_type: 1
      lidar_udp_type:
        device_ip_address: 192.168.1.201
        # ... other config
    ros:
      ros_frame_id: lidar_0
      ros_send_point_cloud_topic: /lidar_0/points
      ros_send_imu_topic: /lidar_0/imu
      # ...

  - driver:
      use_gpu: true
      source_type: 1
      lidar_udp_type:
        device_ip_address: 192.168.1.202
        # ... other config
    ros:
      ros_frame_id: lidar_1
      ros_send_point_cloud_topic: /lidar_1/points
      ros_send_imu_topic: /lidar_1/imu
      # ...
```

Each driver publishes independently to its own topic namespace. Use a node like `pointcloud_to_laserscan`
or a custom fusion node to combine point clouds if needed.

## Troubleshooting

| Issue | Solution |
|:------|:---------|
| No point cloud data | Check `device_ip_address` and `use_ptc_connected` match your setup; verify network connectivity with `ping`. |
| Parsing errors | Ensure `correction_file_path` and `firetimes_path` are correct and accessible. |
| Temperature not publishing (PTC) | Confirm `use_ptc_connected: true` and `source_type: 1` or `5`. Check TCP port 9347 is reachable: `nc -zv device_ip 9347`. |
| Temperature not publishing (UDP tail) | Ensure `udp_status_id_map` is not empty (it's a fail-safe default). Verify IMU block is in packet if `include_imu: true`. |
| CUDA errors | Disable GPU with `use_gpu: false` or verify CUDA version compatibility. |

## References

- [HesaiLidar_SDK_2.0](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0) — underlying C++ SDK
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- Hesai PTC/TCP API manual — required for temperature status ID mapping (fill `udp_status_id_map`)

