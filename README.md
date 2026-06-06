# Introduction to HesaiLidar_ROS_2.0
This repository includes the ROS Driver for Hesai LiDAR sensor manufactured by Hesai Technology. 
Developed based on [HesaiLidar_SDK_2.0](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0), After launched, the project will monitor UDP packets from Lidar,parse data and publish point cloud frames into ROS topic

## Support Lidar type

| Pandar       | OT    | QT       | XT          | AT       | FT    | JT    |
|:-------------|:------|:---------|:------------|:---------|:------|:------|
| Pandar40P    | OT128 | PandarQT | PandarXT    | AT128E2X | FT120 | JT16  |
| Pandar64     | -     | QT128C2X | PandarXT-16 | AT128P   | -     | JT128 |
| Pandar128E3X | -     | -        | XT32M2X     | ATX      | -     | -     |

### Installation dependencies

Install ROS related dependency libraries, please refer to: http://wiki.ros.org
    
- Ubuntu 16.04 - ROS Kinetic desktop
- Ubuntu 18.04 - ROS Melodic desktop
- Ubuntu 20.04 - ROS Noetic desktop
- Ubuntu 18.04 - ROS2 Dashing desktop
- Ubuntu 20.04 - ROS2 Foxy desktop
- Ubuntu 22.04 - ROS2 Humble desktop
- Ubuntu 24.04 - ROS2 Jazzy desktop

### Install Boost

    sudo apt-get update
    sudo apt-get install libboost-all-dev

### Install Yaml

    sudo apt-get update
    sudo apt-get install -y libyaml-cpp-dev

### Clone

    git clone --recurse-submodules https://github.com/leonardonels/HesaiLidar_ROS_2.0.git
    

### Compile and run

- ros1

    Create an `src` folder, copy the source code of the ros driver into it, and then run the following command:
        
        catkin_make
        source devel/setup.bash
        roslaunch hesai_ros_driver start.launch

- ros2

    Create an `src` folder, copy the source code of the ros driver into it, and then run the following command:
        
        colcon build --symlink-install
        . install/setup.bash

    For ROS2-Dashing     

        ros2 launch hesai_ros_driver dashing_start.py
        
    For other ROS2 version

        ros2 launch hesai_ros_driver hesai.launch.py

### Introduction to the configuration file `ros_config.yaml` parameters

```yaml
rviz2: false                                          # Set to true to also open rviz2 with lidar_points
lidar:
  - driver:
      use_gpu: true
      source_type: 1                                  # The type of data source, 1: real-time lidar connection, 2: pcap, 3: packet rosbag, 4: serial    
      # Depending on the type of source_type, fill in the corresponding configuration block (lidar_udp_type, pcap_type, serial_type)
      lidar_udp_type:
        device_ip_address: 192.168.1.201              # host_ip_address. If empty(""), the source ip of the udp point cloud is used
        udp_port: 2368                                # UDP destination port
        ptc_port: 9347                                # PTC port of lidar
        multicast_ip_address: 255.255.255.255

        use_ptc_connected: true                       # Set to false when ptc connection is not used
        host_ptc_port: 0                              # PTC source port, 0 means auto
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

        recv_point_cloud_timeout: -1                  # The timeout of receiving point cloud
        ptc_connect_timeout: -1                       # The timeout of ptc connection

        host_ip_address: ""
        fault_message_port: 0

        standby_mode: -1                              # The standby mode: [-1] is invalit [0] in operation [1] standby
        speed: -1                                     # The speed: [-1] invalit, you must make sure your set has been supported by the lidar you are using
        ptc_mode: 0                                   # The ptc mode: [0] tcp [1] tcp_ssl
        # tcp_ssl use
        certFile: ""                                  # Represents the path of the user's certificate
        privateKeyFile: ""                            # Represents the path of the user's private key 
        caFile: ""                                    # Represents the path of the CA certificate 

      pcap_type:
        pcap_path: "Your pcap file"                         # The path of pcap file
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

        pcap_play_synchronization: true                     # pcap play rate synchronize with the host time
        pcap_play_in_loop: false
        play_rate_: 1.0                                     # pcap play rate 
      
      rosbag_type:
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

      serial_type:
        rs485_com: "Your serial port name for receiving point cloud"  # if using JT16, Port to receive the point cloud
        rs232_com: "Your serial port name for sending cmd"            # if using JT16, Port to send cmd
        point_cloud_baudrate: 3125000
        correction_save_path: ""                                      # turn on when you need to store angle calibration files(from lidar)
        correction_file_path: "Your correction file path"             # The path of correction file
      
      # public module
      use_timestamp_type: 0                 # 0 use point cloud timestamp; 1 use receive timestamp
      frame_start_azimuth: 0                # Frame azimuth for Pandar128, range from 1 to 359, set it less than 0 if you do not want to use it      
      # parser thread num
      thread_num: 4
      # transform param
      transform_flag: false
      x: 0
      y: 0
      z: 0
      roll: 0
      pitch: 0
      yaw: 0
      # fov config, [fov_start, fov_end] range [1, 359], [-1, -1]means use default
      fov_start: -1
      fov_end:  -1
      channel_fov_filter_path: "Your channel fov filter file path"  # The path of channel_fov_filter file, like channel 0 filter [10, 20] and [30,40]
      multi_fov_filter_ranges: "" # compare to channel_fov_filter_path, multi_fov_filter_ranges for all channels. example config "[20,30];[40,200]"
      # other config
      enable_packet_loss_tool: true         # enable the udp packet loss detection tool
      distance_correction_flag: false       # set to true when optical centre correction needs to be turned on
      xt_spot_correction: false             # Set to TRUE when XT S point cloud layering correction is required
      device_udp_src_port: 0                # Filter point clouds for specified source ports in case of multiple lidar, setting >=1024
      device_fault_port: 0                  # Filter fault message for specified source ports in case of multiple lidar, setting >=1024
      frame_frequency: 0                    # The frequency that point cloud sends.
      default_frame_frequency: 10.0         # The default frequency that point cloud sends.
      echo_mode_filter: 0                   # return mode filter

    ros:
      ros_frame_id: hesai_lidar                       # Frame id of packet message and point cloud message
      ros_recv_packet_topic: /lidar_packets           # Topic used to receive lidar packets from rosbag
      ros_send_packet_topic: /lidar_packets           # Topic used to send lidar raw packets through ROS
      ros_send_point_cloud_topic: /lidar_points       # Topic used to send point cloud through ROS
      ros_send_imu_topic: /lidar_imu                  # Topic used to send lidar imu message
      ros_send_packet_loss_topic: /lidar_packets_loss # Topic used to monitor packets loss condition through ROS
      send_packet_ros: false                          # true: Send packets through ROS 
      send_point_cloud_ros: true                      # true: Send point cloud through ROS    
      send_imu_ros: true                              # true: Send imu through ROS    

      # OT128 temperature publishing (see "LiDAR temperature publishing" below)
      temperature:
        enable: false                                # Master switch (off by default)
        source: auto                                 # auto | ptc | udp_tail
        topic: /lidar/temperature                    # diagnostic_msgs/DiagnosticArray topic
        publish_sensor_msgs: false                   # Also publish one sensor_msgs/Temperature per sensor
        sensor_msgs_prefix: /lidar/temperature/      # Topic prefix for the per-sensor messages
        ptc_poll_period_s: 1.0                        # PTC poll interval (seconds), ptc source only
        warn_c: 75.0                                 # Hottest-sensor threshold for DiagnosticStatus WARN
        error_c: 90.0                                # Hottest-sensor threshold for DiagnosticStatus ERROR
        include_imu: true                            # Include IMU temperature (udp_tail source)
        udp_status_id_map: {}                        # Maps UDP-tail status IDs to sensors (fill from OT128 manual)
```


### Real time playback

In the configuration file, set `source_type` to `1`, then configure the parameters under `lidar_udp_type`. Generally, you only need to configure `device_ip_address`, `udp_port`, and `ptc_port`. If the point cloud destination IP is multicast, you need to configure `device_ip_address`. It is recommended to configure `correction_file_path` to prevent point cloud parsing failures when the lidar angle calibration file acquisition fails. Then run start.launch.

### Parsing PCAP file

In the configuration file, set `source_type` to `2`, then configure the parameters under `pcap_type`. Generally, you need to configure `pcap_path`, `correction_file_path`, and `firetime_file_path`. For detailed information about `pcap_play_synchronization` and `pcap_play_in_loop` functionality, please refer to the parameter introduction section in the SDK README. Then run start.launch.

### Record and playback ROSBAG file

- Record ：

    When playing or parsing PCAP in real-time, set `send_packet_ros` to `true`, start another terminal and enter the following command to record the data packet ROSBAG.
        
        rosbag record ros_send_packet_topic

- Playback ：

    First, replay the recorded rosbag file `test.bag` using the following command.
        
        rosbag play test.bag

    Set the `source_type` in the configuration file to `3`, then configure the parameters under `rosbag_type`. Generally, you need to configure `correction_file_path` , `firetime_file_path` and `ros_recv_packet_topic`(the topic name of rosbag, under `ros`), then run start.launch.

### Parsing serial data

In the configuration file, set `source_type` to `4`, then configure the parameters under `serial_type`. Generally, you need to configure `rs485_com` and `rs232_com`. It is recommended to configure `correction_file_path` (required if `rs232_com` is not used). For other parameters, please refer to the parameter introduction section in the SDK README. Then run start.launch.

### LiDAR temperature publishing

This fork can publish OT128 board/IMU temperatures to ROS 2. It is configured by the
`temperature:` block under `ros:` and is **disabled by default** (`enable: false`).

The data source is auto-selected by `source: auto`:

- **`ptc`** — used for a live LiDAR (`source_type: 1` / `5`). Temperatures are polled
  over the PTC command `GetLidarStatus (0x09)` on port `9347`, every
  `ptc_poll_period_s` seconds. Requires `use_ptc_connected: true`.
- **`udp_tail`** — used for pcap / rosbag replay, where there is no live PTC socket.
  Temperatures are read from the UDP packet tail: the rotating status ID/data slots
  (mapped to named sensors via `udp_status_id_map`) plus the optional IMU temperature
  (`include_imu`).

You can also force a source with `source: ptc` or `source: udp_tail`. Forcing `ptc` on a
source without a PTC connection (e.g. a pcap) logs a warning and disables the feature
rather than crashing.

**Topics**

- Primary: one `diagnostic_msgs/DiagnosticArray` on `topic` (default `/lidar/temperature`),
  with one `KeyValue` per sensor. The `DiagnosticStatus` level is `OK` / `WARN` / `ERROR`
  computed from the hottest sensor against `warn_c` / `error_c`.
- Optional: set `publish_sensor_msgs: true` to also publish one `sensor_msgs/Temperature`
  per sensor under `sensor_msgs_prefix` (e.g. `/lidar/temperature/imu`).

Inspect with:

    ros2 topic echo /lidar/temperature

**`udp_status_id_map`** maps each UDP-tail status ID to a sensor; it is empty by default,
so unknown IDs publish nothing (fail-safe). Fill it from the OT128 manual §3.1.2.5; each
entry converts the raw 16-bit field to °C as `value = raw * scale + offset`:

```yaml
      temperature:
        enable: true
        udp_status_id_map:
          1: { label: bottom_board_t1, scale: 0.01, offset: 0.0 }
          2: { label: top_board_t1,    scale: 0.01, offset: 0.0 }
```

> ⚠️ **Caveat:** the PTC `0x09` response layout used by the `ptc` path (49 bytes, 8 ×
> `uint32` temperatures in 0.01 °C, with the sensor labels) is **XT32M1X-derived**
> (`XT32M1X_TCP_API.pdf`) and is **not guaranteed for the OT128**. The parser validates
> the response size defensively and skips short responses, but before trusting the
> *values* on a live OT128 you should dump the raw `0x09` payload and re-verify the byte
> offsets / sensor count / labels against the OT128 PTC/TCP API.

### Realize multi lidar fusion

According to the configuration of a single lidar, multiple drivers can be created in `config.yaml`, as shown in the following example

```yaml
lidar:
  - driver:
      use_gpu: true
      source_type: 1                                  # The type of data source, 1: real-time lidar connection, 2: pcap, 3: packet rosbag, 4: serial    
      # Depending on the type of source_type, fill in the corresponding configuration block (lidar_udp_type, pcap_type, serial_type)
      lidar_udp_type:
        device_ip_address: 192.168.1.201              # host_ip_address. If empty(""), the source ip of the udp point cloud is used
        udp_port: 2368                                # UDP destination port
        ptc_port: 9347                                # PTC port of lidar
        multicast_ip_address: 255.255.255.255

        use_ptc_connected: true                       # Set to false when ptc connection is not used
        host_ptc_port: 0                              # PTC source port, 0 means auto
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

        recv_point_cloud_timeout: -1                  # The timeout of receiving point cloud
        ptc_connect_timeout: -1                       # The timeout of ptc connection

        host_ip_address: ""
        fault_message_port: 0

        standby_mode: -1                              # The standby mode: [-1] is invalit [0] in operation [1] standby
        speed: -1                                     # The speed: [-1] invalit, you must make sure your set has been supported by the lidar you are using
        ptc_mode: 0                                   # The ptc mode: [0] tcp [1] tcp_ssl
        # tcp_ssl use
        certFile: ""                                  # Represents the path of the user's certificate
        privateKeyFile: ""                            # Represents the path of the user's private key 
        caFile: ""                                    # Represents the path of the CA certificate 

      pcap_type:
        pcap_path: "Your pcap file"                         # The path of pcap file
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

        pcap_play_synchronization: true                     # pcap play rate synchronize with the host time
        pcap_play_in_loop: false
        play_rate_: 1.0                                     # pcap play rate 
      
      rosbag_type:
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

      serial_type:
        rs485_com: "Your serial port name for receiving point cloud"  # if using JT16, Port to receive the point cloud
        rs232_com: "Your serial port name for sending cmd"            # if using JT16, Port to send cmd
        point_cloud_baudrate: 3125000
        correction_save_path: ""                                      # turn on when you need to store angle calibration files(from lidar)
        correction_file_path: "Your correction file path"             # The path of correction file
      
      # public module
      use_timestamp_type: 0                 # 0 use point cloud timestamp; 1 use receive timestamp
      frame_start_azimuth: 0                # Frame azimuth for Pandar128, range from 1 to 359, set it less than 0 if you do not want to use it      
      # parser thread num
      thread_num: 4
      # transform param
      transform_flag: false
      x: 0
      y: 0
      z: 0
      roll: 0
      pitch: 0
      yaw: 0
      # fov config, [fov_start, fov_end] range [1, 359], [-1, -1]means use default
      fov_start: -1
      fov_end:  -1
      channel_fov_filter_path: "Your channel fov filter file path"  # The path of channel_fov_filter file, like channel 0 filter [10, 20] and [30,40]
      multi_fov_filter_ranges: "" # compare to channel_fov_filter_path, multi_fov_filter_ranges for all channels. example config "[20,30];[40,200]"
      # other config
      enable_packet_loss_tool: true         # enable the udp packet loss detection tool
      distance_correction_flag: false       # set to true when optical centre correction needs to be turned on
      xt_spot_correction: false             # Set to TRUE when XT S point cloud layering correction is required
      device_udp_src_port: 0                # Filter point clouds for specified source ports in case of multiple lidar, setting >=1024
      device_fault_port: 0                  # Filter fault message for specified source ports in case of multiple lidar, setting >=1024
      frame_frequency: 0                    # The frequency that point cloud sends.
      default_frame_frequency: 10.0         # The default frequency that point cloud sends.
      echo_mode_filter: 0                   # return mode filter

    ros:
      ros_frame_id: hesai_lidar                       # Frame id of packet message and point cloud message
      ros_recv_packet_topic: /lidar_packets           # Topic used to receive lidar packets from rosbag
      ros_send_packet_topic: /lidar_packets           # Topic used to send lidar raw packets through ROS
      ros_send_point_cloud_topic: /lidar_points       # Topic used to send point cloud through ROS
      ros_send_imu_topic: /lidar_imu                  # Topic used to send lidar imu message
      ros_send_packet_loss_topic: /lidar_packets_loss # Topic used to monitor packets loss condition through ROS
      send_packet_ros: false                          # true: Send packets through ROS 
      send_point_cloud_ros: true                      # true: Send point cloud through ROS    
      send_imu_ros: true                              # true: Send imu through ROS    
  - driver:
      use_gpu: true
      source_type: 1                                  # The type of data source, 1: real-time lidar connection, 2: pcap, 3: packet rosbag, 4: serial    
      # Depending on the type of source_type, fill in the corresponding configuration block (lidar_udp_type, pcap_type, serial_type)
      lidar_udp_type:
        device_ip_address: 192.168.1.202              # host_ip_address. If empty(""), the source ip of the udp point cloud is used
        udp_port: 2369                                # UDP destination port
        ptc_port: 9347                                # PTC port of lidar
        multicast_ip_address: 255.255.255.255

        use_ptc_connected: true                       # Set to false when ptc connection is not used
        host_ptc_port: 0                              # PTC source port, 0 means auto
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

        recv_point_cloud_timeout: -1                  # The timeout of receiving point cloud
        ptc_connect_timeout: -1                       # The timeout of ptc connection

        host_ip_address: ""
        fault_message_port: 0

        standby_mode: -1                              # The standby mode: [-1] is invalit [0] in operation [1] standby
        speed: -1                                     # The speed: [-1] invalit, you must make sure your set has been supported by the lidar you are using
        ptc_mode: 0                                   # The ptc mode: [0] tcp [1] tcp_ssl
        # tcp_ssl use
        certFile: ""                                  # Represents the path of the user's certificate
        privateKeyFile: ""                            # Represents the path of the user's private key 
        caFile: ""                                    # Represents the path of the CA certificate 

      pcap_type:
        pcap_path: "Your pcap file"                         # The path of pcap file
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

        pcap_play_synchronization: true                     # pcap play rate synchronize with the host time
        pcap_play_in_loop: false
        play_rate_: 1.0                                     # pcap play rate 
      
      rosbag_type:
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

      serial_type:
        rs485_com: "Your serial port name for receiving point cloud"  # if using JT16, Port to receive the point cloud
        rs232_com: "Your serial port name for sending cmd"            # if using JT16, Port to send cmd
        point_cloud_baudrate: 3125000
        correction_save_path: ""                                      # turn on when you need to store angle calibration files(from lidar)
        correction_file_path: "Your correction file path"             # The path of correction file
      
      # public module
      use_timestamp_type: 0                 # 0 use point cloud timestamp; 1 use receive timestamp
      frame_start_azimuth: 0                # Frame azimuth for Pandar128, range from 1 to 359, set it less than 0 if you do not want to use it      
      # parser thread num
      thread_num: 4
      # transform param
      transform_flag: false
      x: 0
      y: 0
      z: 0
      roll: 0
      pitch: 0
      yaw: 0
      # fov config, [fov_start, fov_end] range [1, 359], [-1, -1]means use default
      fov_start: -1
      fov_end:  -1
      channel_fov_filter_path: "Your channel fov filter file path"  # The path of channel_fov_filter file, like channel 0 filter [10, 20] and [30,40]
      multi_fov_filter_ranges: "" # compare to channel_fov_filter_path, multi_fov_filter_ranges for all channels. example config "[20,30];[40,200]"
      # other config
      enable_packet_loss_tool: true         # enable the udp packet loss detection tool
      distance_correction_flag: false       # set to true when optical centre correction needs to be turned on
      xt_spot_correction: false             # Set to TRUE when XT S point cloud layering correction is required
      device_udp_src_port: 0                # Filter point clouds for specified source ports in case of multiple lidar, setting >=1024
      device_fault_port: 0                  # Filter fault message for specified source ports in case of multiple lidar, setting >=1024
      frame_frequency: 0                    # The frequency that point cloud sends.
      default_frame_frequency: 10.0         # The default frequency that point cloud sends.
      echo_mode_filter: 0                   # return mode filter

    ros:
      ros_frame_id: hesai_lidar                       # Frame id of packet message and point cloud message
      ros_recv_packet_topic: /lidar_packets_2           # Topic used to receive lidar packets from rosbag
      ros_send_packet_topic: /lidar_packets_2           # Topic used to send lidar raw packets through ROS
      ros_send_point_cloud_topic: /lidar_points_2       # Topic used to send point cloud through ROS
      ros_send_imu_topic: /lidar_imu_2                  # Topic used to send lidar imu message
      ros_send_packet_loss_topic: /lidar_packets_loss_2 # Topic used to monitor packets loss condition through ROS
      send_packet_ros: false                          # true: Send packets through ROS 
      send_point_cloud_ros: true                      # true: Send point cloud through ROS    
      send_imu_ros: true                              # true: Send imu through ROS    
```
