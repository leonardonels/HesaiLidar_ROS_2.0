/************************************************************************************************
  Copyright(C)2023 Hesai Technology Co., Ltd.
  All code in this repository is released under the terms of the following [Modified BSD License.]
  Modified BSD License:
  Redistribution and use in source and binary forms,with or without modification,are permitted
  provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice,this list of conditions
   and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice,this list of conditions and
   the following disclaimer in the documentation and/or other materials provided with the distribution.
  *Neither the names of the University of Texas at Austin,nor Austin Robot Technology,nor the names of
   other contributors maybe used to endorse or promote products derived from this software without
   specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGH THOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES,INCLUDING,BUT NOT LIMITED TO,THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT,INDIRECT,INCIDENTAL,SPECIAL,EXEMPLARY,OR CONSEQUENTIAL DAMAGES(INCLUDING,BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE,DATA,OR PROFITS;OR BUSINESS INTERRUPTION)HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY,WHETHER IN CONTRACT,STRICT LIABILITY,OR TORT(INCLUDING NEGLIGENCE
  OR OTHERWISE)ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF
  SUCHDAMAGE.
************************************************************************************************/

/*
 * File: source_driver_ros2.cpp
 * Author: Zhang Yu <zhangyu@hesaitech.com>
 * Description: Source Driver implementation for ROS2
 * Created on June 12, 2023, 10:46 AM
 */

#include "hesai_ros_driver/driver/source_driver_ros2.hpp"

#include <iomanip>
#include <limits>

/*
 * Initializes the Hesai LiDAR driver from a YAML configuration node.
 *
 * Reads all driver parameters (network, decoder, ROS topic names, filters),
 * creates the ROS2 node, sets up publishers and subscribers based on the
 * configured source type, registers SDK callbacks for point cloud / packet /
 * IMU / PTP / correction / firetime / packet-loss delivery, initialises the
 * HesaiLidarSdk, and optionally sets up a BARQ shared-memory writer for
 * low-latency inter-process point cloud transport.
 */
void SourceDriver::Init(const YAML::Node& config)
{
  DriveYamlParam yaml_param;
  yaml_param.GetDriveYamlParam(config, driver_param);
  frame_id_ = driver_param.input_param.frame_id;

  if (driver_param.custom_param.real_time_timestamp)
  {
    rclcpp::Clock clock(RCL_ROS_TIME);
    driver_start_timestamp_ = clock.now().seconds();
  }else{
    driver_start_timestamp_ = 0.0;
  }

  node_ptr_.reset(new rclcpp::Node("hesai_ros_driver_node"));
  auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
  // depth is already 5 from the profile, override if needed:
  // qos.keep_last(10);

  if (driver_param.input_param.send_point_cloud_ros) {
#ifdef ENABLE_ZERO_COPY
    if (driver_param.custom_param.zero_copy_enabled) {
      bounded_pub_ = node_ptr_->create_publisher<mmr_base::msg::BoundedPointcloud>(driver_param.input_param.ros_send_point_topic, qos); 
      zero_copy_enabled_ = true;
    } else {
#endif
      pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(driver_param.input_param.ros_send_point_topic, qos);
#ifdef ENABLE_ZERO_COPY

    }
#endif
  }
  if (driver_param.input_param.send_imu_ros) {
    imu_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Imu>(driver_param.input_param.ros_send_imu_topic, qos);
  }

  if (driver_param.input_param.ros_send_packet_loss_topic != NULL_TOPIC) {
    loss_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::LossPacket>(driver_param.input_param.ros_send_packet_loss_topic, qos);
  }

  if (driver_param.input_param.source_type == DATA_FROM_LIDAR) {
    if (driver_param.input_param.ros_send_ptp_topic != NULL_TOPIC) {
      ptp_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::Ptp>(driver_param.input_param.ros_send_ptp_topic, qos);
    }

    if (driver_param.input_param.ros_send_correction_topic != NULL_TOPIC) {
      crt_pub_ = node_ptr_->create_publisher<std_msgs::msg::UInt8MultiArray>(driver_param.input_param.ros_send_correction_topic, qos);
    }
  }
  if (! driver_param.input_param.firetimes_path.empty() ) {
    if (driver_param.input_param.ros_send_firetime_topic != NULL_TOPIC) {
      firetime_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::Firetime>(driver_param.input_param.ros_send_firetime_topic, qos);
    }
  }

  if (driver_param.input_param.send_packet_ros) {
    pkt_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::UdpFrame>(driver_param.input_param.ros_send_packet_topic, qos);
  }

  if (driver_param.input_param.source_type == DATA_FROM_ROS_PACKET) {
    pkt_sub_ = node_ptr_->create_subscription<hesai_ros_driver::msg::UdpFrame>(driver_param.input_param.ros_recv_packet_topic, qos,
                              std::bind(&SourceDriver::ReceivePacket, this, std::placeholders::_1));
    if (driver_param.input_param.ros_recv_correction_topic != NULL_TOPIC) {
      crt_sub_ = node_ptr_->create_subscription<std_msgs::msg::UInt8MultiArray>(driver_param.input_param.ros_recv_correction_topic, qos,
                              std::bind(&SourceDriver::ReceiveCorrection, this, std::placeholders::_1));
    }
    driver_param.decoder_param.enable_udp_thread = false;
    subscription_spin_thread_ = new boost::thread(boost::bind(&SourceDriver::SpinRos2,this));
  }
  driver_ptr_.reset(new HesaiLidarSdk<LidarPointXYZIRT>());
  driver_param.decoder_param.enable_parser_thread = true;
#ifdef ENABLE_BARQ
  if (driver_param.custom_param.BARQ_enable) {
    const size_t kMaxPoints = 300000;
    barq_max_size_ = sizeof(BARQFrameHeader) + kMaxPoints * sizeof(BARQPoint) + 512;
    barq_writer_   = std::make_unique<BARQ::Writer>(driver_param.custom_param.BARQ_topic, barq_max_size_, false);
    if (barq_writer_->init()) {
      barq_enabled_ = true;
      RCLCPP_INFO(node_ptr_->get_logger(), "BARQ writer initialized (%zu bytes).", barq_max_size_);
    } else {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to initialize BARQ writer.");
      barq_writer_.reset();
    }
  }
#endif
  if (driver_param.input_param.send_point_cloud_ros || barq_enabled_) {
    driver_ptr_->RegRecvCallback([this](const LidarDecodedFrame<LidarPointXYZIRT>& frame) {
      if (driver_param.input_param.send_point_cloud_ros)
        this->SendPointCloudWithRos(frame);
#ifdef ENABLE_BARQ
      if (barq_enabled_)
        this->SendPointCloudWithBarq(frame);
#endif
    });
  }
  if (driver_param.input_param.send_imu_ros) {
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendImuConfig, this, std::placeholders::_1));
  }
  if (driver_param.input_param.send_packet_ros) {
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPacket, this, std::placeholders::_1, std::placeholders::_2)) ;
  }
  if (driver_param.input_param.ros_send_packet_loss_topic != NULL_TOPIC) {
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPacketLoss, this, std::placeholders::_1, std::placeholders::_2));
  }
  if (driver_param.input_param.source_type == DATA_FROM_LIDAR) {
    if (driver_param.input_param.ros_send_correction_topic != NULL_TOPIC) {
      driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendCorrection, this, std::placeholders::_1));
    }
    if (driver_param.input_param.ros_send_ptp_topic != NULL_TOPIC) {
      driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPTP, this, std::placeholders::_1, std::placeholders::_2));
    }
  }

  // --- OT128 temperature publishing ------------------------------------------
  // Auto-selects PTC for live sources and the UDP packet tail for pcap/replay,
  // with a manual override. A forced-source mismatch (e.g. ptc on pcap) disables
  // the feature with a warning rather than null-deref'ing the absent PTC socket.
  if (driver_param.custom_param.temp_enable) {
    const auto src = driver_param.input_param.source_type;
    const bool is_live = (src == DATA_FROM_LIDAR || src == DATA_FROM_LIDAR_TCP);
    std::string eff = driver_param.custom_param.temp_source;
    if (eff == "auto") {
      eff = is_live ? "ptc" : "udp_tail";
    }

    bool ok = true;
    if (eff == "ptc" && !is_live) {
      RCLCPP_WARN(node_ptr_->get_logger(),
        "temperature.source=ptc but data source is not a live lidar (no PTC socket); disabling temperature feature.");
      ok = false;
    } else if (eff != "ptc" && eff != "udp_tail") {
      RCLCPP_WARN(node_ptr_->get_logger(),
        "temperature.source='%s' is invalid (expected auto|ptc|udp_tail); disabling temperature feature.", eff.c_str());
      ok = false;
    }

    if (ok) {
      temp_enabled_ = true;
      temp_effective_source_ = eff;
      temp_diag_pub_ = node_ptr_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          driver_param.custom_param.temp_topic, qos);
      if (eff == "ptc") {
        driver_ptr_->SetStatusPollPeriod(driver_param.custom_param.temp_ptc_poll_period_s);
        driver_ptr_->RegRecvCallback(std::function<void(const LidarStatus&)>(
            std::bind(&SourceDriver::OnLidarStatus, this, std::placeholders::_1)));
      } else {  // udp_tail
        // This reuses the same (UdpFrame_t, double) callback slot as SendPacket,
        // so fold both in if packet publishing is also enabled (otherwise the
        // later registration would silently clobber the earlier one).
        const bool also_send_packet = driver_param.input_param.send_packet_ros;
        driver_ptr_->RegRecvCallback(std::function<void(const UdpFrame_t&, double)>(
            [this, also_send_packet](const UdpFrame_t& f, double ts) {
              if (also_send_packet) this->SendPacket(f, ts);
              this->ExtractTailTemps(f, ts);
            }));
      }
      RCLCPP_INFO(node_ptr_->get_logger(),
        "Temperature publishing enabled (source=%s, topic=%s).",
        eff.c_str(), driver_param.custom_param.temp_topic.c_str());
    }
  }

  if (!driver_ptr_->Init(driver_param))
  {
    std::cout << "Driver Initialize Error...." << std::endl;
    exit(-1);
  }
}

/*
 * Starts the HesaiLidarSdk processing pipeline.
 *
 * After Init() has configured the SDK and registered all callbacks, this
 * kicks off the internal receive and decode threads so that data begins
 * flowing from the LiDAR (or pcap / rosbag) to the ROS2 topics.
 */
void SourceDriver::Start()
{
  driver_ptr_->Start();
}

/*
 * Destructor. Delegates cleanup to Stop().
 */
SourceDriver::~SourceDriver()
{
  Stop();
}

/*
 * Stops the HesaiLidarSdk and releases BARQ shared-memory resources.
 *
 * Safe to call multiple times; the BARQ writer is reset to nullptr after
 * destruction so subsequent calls are no-ops for that part.
 */
void SourceDriver::Stop()
{
  driver_ptr_->Stop();

#ifdef ENABLE_BARQ
  if (barq_writer_) {
    barq_writer_->destroy();
    barq_writer_.reset();
    barq_enabled_ = false;
  }
#endif
}

/*
 * Publishes a raw UDP frame (a vector of raw LiDAR packets) on the
 * configured ROS2 packet topic.
 *
 * Registered as an SDK callback and invoked each time the SDK assembles a
 * complete UDP frame. Downstream subscribers can use these packets for
 * rosbag recording or off-line replay via DATA_FROM_ROS_PACKET.
 */
void SourceDriver::SendPacket(const UdpFrame_t& msg, double timestamp)
{
  pkt_pub_->publish(ToRosMsg(msg, timestamp));
}

#ifdef ENABLE_BARQ
/*
 * Serializes a decoded point cloud frame into a vector of bytes suitable for BARQ shared-memory publishing.
 *
 * This function converts the point cloud data into a format that can be efficiently written to BARQ shared memory.
 * In the future, this could be optimise to remove unwanted fields like ring or timestamp if they are not needed by the consumer
 * In the future, this could be optimized further by implementing the serialization directly in the SDK's decode thread to avoid the extra copy.
 */
std::vector<uint8_t> SourceDriver::SerializePointCloudForBarq(const LidarDecodedFrame<LidarPointXYZIRT>& frame)
{
  const size_t payload_size = sizeof(BARQFrameHeader) + frame.points_num * sizeof(BARQPoint);
  std::vector<uint8_t> buf(payload_size);
  uint8_t* ptr = buf.data();

  BARQFrameHeader hdr;
  hdr.width      = frame.points_num;
  hdr.height     = 1;
  hdr.point_step = sizeof(BARQPoint);
  hdr.timestamp  = frame.frame_start_timestamp;
  std::memcpy(ptr, &hdr, sizeof(hdr));
  ptr += sizeof(hdr);

  for (uint32_t i = 0; i < frame.points_num; ++i) {
    const auto& src = frame.points[i];
    BARQPoint dst;
    dst.x         = src.x;
    dst.y         = src.y;
    dst.z         = src.z;
    dst.intensity = static_cast<float>(src.intensity);
    dst.ring      = src.ring;
    dst.timestamp = src.timestamp;
    std::memcpy(ptr, &dst, sizeof(dst));
    ptr += sizeof(dst);
  }

  return buf;
}

void SourceDriver::SendPointCloudWithBarq(const LidarDecodedFrame<LidarPointXYZIRT>& frame)
{
  if (!barq_enabled_ || !barq_writer_) return;
  std::vector<uint8_t> buf = SerializePointCloudForBarq(frame);
  const size_t payload_size = sizeof(BARQFrameHeader) + frame.points_num * sizeof(BARQPoint); 
  barq_writer_->write(buf.data(), payload_size);
}
#endif

/*
 * Converts a decoded point cloud frame to sensor_msgs::PointCloud2 and
 * publishes it on the ROS2 point cloud topic. Optionally also writes the
 * serialised point data into a BARQ shared-memory segment for zero-copy
 * consumption by co-located processes.
 *
 * If latency_testing mode is active the header timestamp is overwritten
 * with the current ROS clock time so that end-to-end latency can be
 * measured by the subscriber.
 */
void SourceDriver::SendPointCloudWithRos(const LidarDecodedFrame<LidarPointXYZIRT>& msg)
{  
  sensor_msgs::msg::PointCloud2 ros_msg;
#ifdef ENABLE_ZERO_COPY
  mmr_base::msg::BoundedPointcloud bounded_msg;
  if(zero_copy_enabled_){
    bounded_msg = ToBoundedMsg(msg, frame_id_);
  }else{
#endif
    ros_msg = ToRosMsg(msg, frame_id_);
#ifdef ENABLE_ZERO_COPY
  }
#endif
  // For latency testing only, we set the timestamp to the current time when the point cloud is received
  if (driver_param.custom_param.latency_testing) {
    auto now = rclcpp::Clock(RCL_ROS_TIME).now();
    ros_msg.header.stamp = now;
  }
#ifdef ENABLE_ZERO_COPY
  if(zero_copy_enabled_){
    bounded_pub_->publish(bounded_msg);
  }else{
#endif
    pub_->publish(ros_msg);
#ifdef ENABLE_ZERO_COPY
  }
#endif
}

/*
 * Publishes the LiDAR correction / calibration data as a UInt8MultiArray
 * on the configured correction topic.
 *
 * Only relevant when the source is DATA_FROM_LIDAR and a correction topic
 * has been configured. A subscriber (e.g. a rosbag recorder) can persist
 * this so that DATA_FROM_ROS_PACKET replay has access to the calibration
 * without a live LiDAR connection.
 */
void SourceDriver::SendCorrection(const u8Array_t& msg)
{
  crt_pub_->publish(ToRosMsg(msg));
}

/*
 * Publishes cumulative packet-loss counters as a LossPacket message.
 *
 * The SDK tracks total packets expected vs. total packets actually received;
 * this callback forwards those counters to a ROS2 topic so that monitoring
 * tools can detect degraded network links in real time.
 */
void SourceDriver::SendPacketLoss(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count)
{
  loss_pub_->publish(ToRosMsg(total_packet_count, total_packet_loss_count));
}

/*
 * Publishes the PTP synchronisation status (lock offset and 16-byte status
 * array) on the configured PTP topic.
 *
 * Allows external nodes to monitor whether the LiDAR's internal clock is
 * PTP-locked and how large the offset is.
 */
void SourceDriver::SendPTP(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status)
{
  ptp_pub_->publish(ToRosMsg(ptp_lock_offset, ptp_status));
}

/*
 * Publishes the 512-element firetime correction array as a Firetime message.
 *
 * Firetime corrections compensate for the per-channel laser firing delay
 * within a single measurement cycle. Publishing them lets downstream nodes
 * apply or verify the correction.
 */
void SourceDriver::SendFiretime(const double *firetime_correction_)
{
  firetime_pub_->publish(ToRosMsg(firetime_correction_));
}

/*
 * Publishes IMU data (linear acceleration + angular velocity) as a
 * sensor_msgs::Imu message, converting from the LiDAR's native units
 * (g, deg/s) to SI units (m/s^2, rad/s).
 */
void SourceDriver::SendImuConfig(const LidarImuData& msg)
{
  imu_pub_->publish(ToRosMsg(msg));
}

/*
 * PTC temperature path. Receives a parsed LidarStatus (polled over PTC 0x09 on
 * a wall-clock interval) and publishes its 8 board/laser temperatures.
 *
 * NOTE: the 8-sensor / 0.01 C layout and the labels below are XT32M1X-derived
 * (XT32M1X_TCP_API.pdf) and are NOT guaranteed for the OT128 -- verify against
 * the OT128 PTC/TCP API before trusting the values (see LidarStatus in the SDK).
 */
void SourceDriver::OnLidarStatus(const hesai::lidar::LidarStatus& status)
{
  if (!temp_enabled_ || temp_diag_pub_ == nullptr || !status.valid) {
    return;
  }
  static const char* kLabels[8] = {
    "bottom_board_t1", "bottom_board_t2",
    "laser_emitting_rt_l1", "laser_emitting_rt_l2",
    "laser_receiving_rt_r", "laser_receiving_rt2",
    "top_circuit_rt3", "top_circuit_rt4"
  };
  std::map<std::string, double> temps;
  for (int i = 0; i < 8; ++i) {
    temps[kLabels[i]] = status.temperature[i] / 100.0;  // 0.01 C -> C
  }
  PublishTemps(temps, node_ptr_->now());
}

/*
 * UDP-tail temperature path. For sources with no live PTC socket (pcap/replay),
 * the only temperature data present is in the UDP packet tail: rotating status
 * ID/data slots (mapped to sensors via udp_status_id_map) plus an optional IMU
 * temperature. Pointer arithmetic here mirrors Udp1_4Parser packet navigation;
 * every dereference is bounds-checked against packet_len so a malformed or
 * non-v1.4 packet is skipped rather than read out of bounds.
 *
 * Status fields rotate across frames (only a few IDs per packet), so values are
 * accumulated into temp_cache_ and the latest cache is published each frame.
 */
void SourceDriver::ExtractTailTemps(const UdpFrame_t& frame, double timestamp)
{
  if (!temp_enabled_ || temp_diag_pub_ == nullptr) {
    return;
  }
  const auto& id_map = driver_param.custom_param.udp_status_id_map;
  const bool include_imu = driver_param.custom_param.temp_include_imu;

  for (const auto& packet : frame) {
    const uint8_t* p = packet.buffer;
    const size_t len = packet.packet_len;
    if (len < sizeof(HS_LIDAR_PRE_HEADER) + sizeof(HS_LIDAR_HEADER_ME_V4)) {
      continue;
    }
    const auto* pre = reinterpret_cast<const HS_LIDAR_PRE_HEADER*>(p);
    // Only OT128/v1.4 packets (magic EE FF 01 04: delimiter 0xffee, ver 1.4).
    if (!pre->IsValidDelimiter() || pre->GetVersionMajor() != 1 || pre->GetVersionMinor() != 4) {
      continue;
    }
    const auto* header = reinterpret_cast<const HS_LIDAR_HEADER_ME_V4*>(p + sizeof(HS_LIDAR_PRE_HEADER));
    const uint8_t st = header->m_u8Status;
    const size_t body_size =
        (sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) + header->unitSize() * header->GetLaserNum()) * header->GetBlockNum()
        + sizeof(HS_LIDAR_BODY_CRC_ME_V4);
    const size_t tail_offset =
        sizeof(HS_LIDAR_PRE_HEADER) + sizeof(HS_LIDAR_HEADER_ME_V4) + body_size
        + (hasFunctionSafety(st) ? sizeof(HS_LIDAR_FUNC_SAFETY_ME_V4) : 0);
    if (tail_offset + sizeof(HS_LIDAR_TAIL_ME_V4) > len) {
      continue;
    }
    const auto* tail = reinterpret_cast<const HS_LIDAR_TAIL_ME_V4*>(p + tail_offset);

    // Three rotating reserved status slots; map known IDs to named sensors.
    const uint8_t  ids[3]  = { tail->GetStsID0(), tail->GetStsID1(), tail->GetStsID2() };
    const uint16_t data[3] = { tail->GetData0(),  tail->GetData1(),  tail->GetData2()  };
    for (int i = 0; i < 3; ++i) {
      auto it = id_map.find(ids[i]);
      if (it != id_map.end()) {
        temp_cache_[it->second.label] = data[i] * it->second.scale + it->second.offset;
      }
    }

    if (include_imu && hasImu(st)) {
      const size_t imu_offset = tail_offset + sizeof(HS_LIDAR_TAIL_ME_V4)
          + (hasSeqNum(st) ? sizeof(HS_LIDAR_TAIL_SEQ_NUM_ME_V4) : 0);
      if (imu_offset + sizeof(HS_LIDAR_TAIL_IMU_ME_V4) <= len) {
        const auto* imu = reinterpret_cast<const HS_LIDAR_TAIL_IMU_ME_V4*>(p + imu_offset);
        temp_cache_["imu"] = imu->GetIMUTemperature();
      }
    }
  }

  if (temp_cache_.empty()) {
    return;
  }
  // Use the frame timestamp when available (seconds), else the node clock.
  const rclcpp::Time stamp = (timestamp > 0.0)
      ? rclcpp::Time(static_cast<int64_t>(timestamp * 1e9))
      : node_ptr_->now();
  PublishTemps(temp_cache_, stamp);
}

/*
 * Shared publisher for both temperature paths. Emits one aggregated
 * diagnostic_msgs/DiagnosticArray (one KeyValue per sensor, level computed from
 * the hottest sensor vs. the configured warn/error thresholds) and, when
 * temp_publish_sensor_msgs is set, one sensor_msgs/Temperature per sensor
 * (publishers created lazily as new labels appear).
 */
void SourceDriver::PublishTemps(const std::map<std::string, double>& temps, const rclcpp::Time& stamp)
{
  if (temps.empty() || temp_diag_pub_ == nullptr) {
    return;
  }
  const double warn_c  = driver_param.custom_param.temp_warn_c;
  const double error_c = driver_param.custom_param.temp_error_c;

  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = stamp;

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "OT128/temperature";
  st.hardware_id = "OT128";

  double max_c = -std::numeric_limits<double>::infinity();
  for (const auto& kv : temps) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = kv.first;
    std::ostringstream val;
    val << std::fixed << std::setprecision(1) << kv.second;
    item.value = val.str();
    st.values.push_back(item);
    if (kv.second > max_c) max_c = kv.second;
  }

  if (max_c >= error_c) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  } else if (max_c >= warn_c) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  } else {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  }
  std::ostringstream msg;
  msg << "max " << std::fixed << std::setprecision(1) << max_c << " C";
  st.message = msg.str();

  arr.status.push_back(st);
  temp_diag_pub_->publish(arr);

  if (driver_param.custom_param.temp_publish_sensor_msgs) {
    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    for (const auto& kv : temps) {
      auto pub_it = temp_sensor_pubs_.find(kv.first);
      if (pub_it == temp_sensor_pubs_.end()) {
        const std::string topic = driver_param.custom_param.temp_sensor_msgs_prefix + kv.first;
        pub_it = temp_sensor_pubs_.emplace(
            kv.first,
            node_ptr_->create_publisher<sensor_msgs::msg::Temperature>(topic, qos)).first;
      }
      sensor_msgs::msg::Temperature tmsg;
      tmsg.header.stamp = stamp;
      tmsg.header.frame_id = frame_id_;
      tmsg.temperature = kv.second;
      tmsg.variance = 0.0;
      pub_it->second->publish(tmsg);
    }
  }
}

/*
 * Converts an SDK LidarDecodedFrame into a sensor_msgs::PointCloud2.
 *
 * Handles both single-return and multi-return frame modes. Filters out
 * non-finite points, applies the optional bubble filter (spherical distance
 * threshold around the sensor origin) and cube filter (axis-aligned box),
 * and packs the surviving points into the XYZIRF64 field layout
 * (x, y, z, intensity, ring, timestamp).
 *
 * The header timestamp is set from the frame's start timestamp, adjusted
 * by driver_start_timestamp_ when real_time_timestamp mode is active.
 */
sensor_msgs::msg::PointCloud2 SourceDriver::ToRosMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id)
{
  sensor_msgs::msg::PointCloud2 ros_msg;
  uint32_t points_number = (frame.fParam.IsMultiFrameFrequency() == 0) ? frame.points_num : frame.multi_points_num;
  uint32_t packet_number = (frame.fParam.IsMultiFrameFrequency() == 0) ? frame.packet_num : frame.multi_packet_num;
  LidarPointXYZIRT *pPoints = (frame.fParam.IsMultiFrameFrequency() == 0) ? frame.points : frame.multi_points;
  int frame_index = (frame.fParam.IsMultiFrameFrequency() == 0) ? frame.frame_index : frame.multi_frame_index;
  double frame_start_timestamp = (frame.fParam.IsMultiFrameFrequency() == 0) ? frame.frame_start_timestamp : frame.multi_frame_start_timestamp;
  double frame_end_timestamp = (frame.fParam.IsMultiFrameFrequency() == 0) ? frame.frame_end_timestamp : frame.multi_frame_end_timestamp;
  const char *prefix = (frame.fParam.IsMultiFrameFrequency() == 0) ? "raw" : "multi";
  size_t n_real_points = frame.points_num;
  int fields = 6;
  ros_msg.fields.clear();
  ros_msg.fields.reserve(fields);

#ifdef VERBOSE_LOGGING
  RCLCPP_INFO(node_ptr_->get_logger(), "Converting to ROS PointCloud2 message with %u points", frame.points_num);
#endif
  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "ring", 1, sensor_msgs::msg::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64, offset);

  ros_msg.point_step = offset;
  ros_msg.is_dense = false;
  ros_msg.data.resize(n_real_points * ros_msg.point_step);

  sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");

  for (size_t i = 0; i < frame.points_num; i++)
  {
    LidarPointXYZIRT point = pPoints[i];
    *iter_x_ = point.x;
    *iter_y_ = point.y;
    *iter_z_ = point.z;
    *iter_intensity_ = point.intensity;
    *iter_ring_ = point.ring;
    *iter_timestamp_ = point.timestamp + driver_start_timestamp_;
    ++iter_x_;
    ++iter_y_;
    ++iter_z_;
    ++iter_intensity_;
    ++iter_ring_;
    ++iter_timestamp_;
  }

  ros_msg.width = n_real_points;
  ros_msg.height = 1;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.data.resize(n_real_points * ros_msg.point_step);
  std::cout.flush();
  auto sec = (uint64_t)floor(frame_start_timestamp);
  if (sec <= std::numeric_limits<int32_t>::max()) {
    ros_msg.header.stamp.sec = (uint32_t)floor(frame_start_timestamp + driver_start_timestamp_);
    ros_msg.header.stamp.nanosec = (uint32_t)round((frame_start_timestamp + driver_start_timestamp_ - ros_msg.header.stamp.sec) * 1e9);
  } else {
    printf("does not support timestamps greater than 19 January 2038 03:14:07 (now %lf)\n", frame_start_timestamp);
  }
  ros_msg.header.frame_id = frame_id_;
  return ros_msg;
}

#ifdef ENABLE_ZERO_COPY
mmr_base::msg::BoundedPointcloud SourceDriver::ToBoundedMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id)
{
  auto loaned = bounded_pub_->borrow_loaned_message();
  auto& msg = loaned.get();
  double frame_start_timestamp = (frame.fParam.IsMultiFrameFrequency() == 0) ? frame.frame_start_timestamp : frame.multi_frame_start_timestamp;
  size_t n_real_points = frame.points_num;

  uint8_t* p = msg.data.data();
  for (uint32_t i = 0; i < frame.points_num; ++i) {
    const auto& src = frame.points[i];
    const float    fx = src.x, fy = src.y, fz = src.z, fi = src.intensity;
    const uint16_t r  = src.ring;
    const double   ts = src.timestamp;
    std::memcpy(p +  0, &fx, 4);
    std::memcpy(p +  4, &fy, 4);
    std::memcpy(p +  8, &fz, 4);
    std::memcpy(p + 12, &fi, 4);
    std::memcpy(p + 16, &r,  2);
    std::memcpy(p + 18, &ts, 8);
    p += 26;
  }

  msg.width = n_real_points;
  auto sec = (uint64_t)floor(frame_start_timestamp);
  if (sec <= std::numeric_limits<int32_t>::max()) {
    msg.stamp.sec = (uint32_t)floor(frame_start_timestamp + driver_start_timestamp_);
    msg.stamp.nanosec = (uint32_t)round((frame_start_timestamp + driver_start_timestamp_ - msg.stamp.sec) * 1e9);
  } else {
    printf("does not support timestamps greater than 19 January 2038 03:14:07 (now %lf)\n", frame_start_timestamp);
  }
  return msg;
}
#endif

/*
 * Converts a vector of raw UDP packets into a hesai_ros_driver::msg::UdpFrame.
 *
 * Each UdpPacket_t's raw buffer is copied into a UdpPacket message. The
 * frame-level header timestamp is set from the provided double-precision
 * seconds value.
 */
hesai_ros_driver::msg::UdpFrame SourceDriver::ToRosMsg(const UdpFrame_t& ros_msg, double timestamp) {
  hesai_ros_driver::msg::UdpFrame rs_msg;
  for (size_t i = 0 ; i < ros_msg.size(); i++) {
    hesai_ros_driver::msg::UdpPacket rawpacket;
    rawpacket.size = ros_msg[i].packet_len;
    rawpacket.data.resize(ros_msg[i].packet_len);
    memcpy(&rawpacket.data[0], &ros_msg[i].buffer[0], ros_msg[i].packet_len);
    rs_msg.packets.push_back(rawpacket);
  }
  auto sec = (uint64_t)floor(timestamp);
  if (sec <= std::numeric_limits<int32_t>::max()) {
    rs_msg.header.stamp.sec = (uint32_t)floor(timestamp);
    rs_msg.header.stamp.nanosec = (uint32_t)round((timestamp - rs_msg.header.stamp.sec) * 1e9);
  } else {
    printf("does not support timestamps greater than 19 January 2038 03:14:07 (now %lf)\n", timestamp);
  }
  rs_msg.header.frame_id = frame_id_;
  return rs_msg;
}

/*
 * Converts a correction byte array into a std_msgs::UInt8MultiArray for
 * publishing on the correction topic.
 */
std_msgs::msg::UInt8MultiArray SourceDriver::ToRosMsg(const u8Array_t& correction_string) {
  auto msg = std::make_shared<std_msgs::msg::UInt8MultiArray>();
  msg->data.resize(correction_string.size());
  std::copy(correction_string.begin(), correction_string.end(), msg->data.begin());
  return *msg;
}

/*
 * Converts cumulative packet counters into a LossPacket message for
 * monitoring network reliability.
 */
hesai_ros_driver::msg::LossPacket SourceDriver::ToRosMsg(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count)
{
  hesai_ros_driver::msg::LossPacket msg;
  msg.total_packet_count = total_packet_count;
  msg.total_packet_loss_count = total_packet_loss_count;
  return msg;
}

/*
 * Converts PTP lock offset and 16-byte status vector into a Ptp message.
 */
hesai_ros_driver::msg::Ptp SourceDriver::ToRosMsg(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status)
{
  hesai_ros_driver::msg::Ptp msg;
  msg.ptp_lock_offset = ptp_lock_offset;
  std::copy(ptp_status.begin(), ptp_status.begin() + std::min(16ul, ptp_status.size()), msg.ptp_status.begin());
  return msg;
}

/*
 * Converts a 512-element C-style double array of firetime corrections into
 * a Firetime message whose data field is a fixed float64[512] array.
 */
hesai_ros_driver::msg::Firetime SourceDriver::ToRosMsg(const double *firetime_correction_)
{
  hesai_ros_driver::msg::Firetime msg;
  std::copy(firetime_correction_, firetime_correction_ + 512, msg.data.begin());
  return msg;
}

/*
 * Converts an SDK LidarImuData struct into a sensor_msgs::Imu message.
 *
 * Linear acceleration is converted from g to m/s^2, angular velocity from
 * deg/s to rad/s. The header timestamp uses the IMU sample timestamp
 * offset by driver_start_timestamp_ when real_time_timestamp mode is active.
 */
sensor_msgs::msg::Imu SourceDriver::ToRosMsg(const LidarImuData &imu_config_)
{
  sensor_msgs::msg::Imu ros_msg;
  auto sec = (uint64_t)floor(imu_config_.timestamp);
  if (sec <= std::numeric_limits<int32_t>::max()) {
    ros_msg.header.stamp.sec = (uint32_t)floor(imu_config_.timestamp + driver_start_timestamp_);
    ros_msg.header.stamp.nanosec = (uint32_t)round((imu_config_.timestamp + driver_start_timestamp_- ros_msg.header.stamp.sec) * 1e9);
  } else {
    printf("does not support timestamps greater than 19 January 2038 03:14:07 (now %lf)\n", imu_config_.timestamp);
  }
  ros_msg.header.frame_id = frame_id_;
  double ax = From_g_To_ms2(imu_config_.imu_accel_x);
  double ay = From_g_To_ms2(imu_config_.imu_accel_y);
  double az = From_g_To_ms2(imu_config_.imu_accel_z);
  double gx = From_degs_To_rads(imu_config_.imu_ang_vel_x);
  double gy = From_degs_To_rads(imu_config_.imu_ang_vel_y);
  double gz = From_degs_To_rads(imu_config_.imu_ang_vel_z);

  if (driver_param.decoder_param.transform_param.use_flag) {
    const auto& t = driver_param.decoder_param.transform_param;
    float cosa = std::cos(t.roll),  sina = std::sin(t.roll);
    float cosb = std::cos(t.pitch), sinb = std::sin(t.pitch);
    float cosc = std::cos(t.yaw),   sinc = std::sin(t.yaw);

    auto rotate = [&](double x, double y, double z, double &ox, double &oy, double &oz) {
      ox = cosb*cosc*x + (sina*sinb*cosc - cosa*sinc)*y + (sina*sinc + cosa*sinb*cosc)*z;
      oy = cosb*sinc*x + (cosa*cosc + sina*sinb*sinc)*y + (cosa*sinb*sinc - sina*cosc)*z;
      oz = -sinb*x + sina*cosb*y + cosa*cosb*z;
    };

    rotate(ax, ay, az, ax, ay, az);
    rotate(gx, gy, gz, gx, gy, gz);
  }

  ros_msg.linear_acceleration.x = ax;
  ros_msg.linear_acceleration.y = ay;
  ros_msg.linear_acceleration.z = az;
  ros_msg.angular_velocity.x = gx;
  ros_msg.angular_velocity.y = gy;
  ros_msg.angular_velocity.z = gz;
  return ros_msg;
}

/*
 * Subscription callback: receives UdpFrame messages from a ROS2 topic and
 * feeds each contained packet into the SDK's ring buffer for decoding.
 *
 * Used in DATA_FROM_ROS_PACKET mode to replay rosbag-recorded packets
 * through the same decode pipeline as live data. Sleeps briefly if the
 * ring buffer is full to apply back-pressure.
 */
void SourceDriver::ReceivePacket(const hesai_ros_driver::msg::UdpFrame::SharedPtr msg)
{
  for (size_t i = 0; i < msg->packets.size(); i++) {
    if(driver_ptr_->lidar_ptr_->origin_packets_buffer_.full()) std::this_thread::sleep_for(std::chrono::microseconds(10000));
    driver_ptr_->lidar_ptr_->origin_packets_buffer_.emplace_back(&msg->packets[i].data[0], msg->packets[i].size);
  }
}

/*
 * Subscription callback: receives correction calibration data from a ROS2
 * topic and loads it into the SDK's internal correction engine.
 *
 * Used in DATA_FROM_ROS_PACKET mode so that calibration recorded alongside
 * packets in a rosbag can be applied without a live PTC connection.
 */
void SourceDriver::ReceiveCorrection(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  driver_ptr_->lidar_ptr_->correction_string_.resize(msg->data.size());
  std::copy(msg->data.begin(), msg->data.end(), driver_ptr_->lidar_ptr_->correction_string_.begin());
  while (1) {
    if (! driver_ptr_->lidar_ptr_->LoadCorrectionFromROSbag()) {
      break;
    }
  }
}

/*
 * Converts a linear acceleration value from gravitational units (g) to
 * SI units (m/s^2) using the standard gravity constant 9.80665.
 */
double SourceDriver::From_g_To_ms2(double g)
{
  return g * 9.80665;
}

/*
 * Converts an angular velocity value from degrees per second to radians
 * per second.
 */
double SourceDriver::From_degs_To_rads(double degree)
{
  return degree * M_PI / 180.0;
}
