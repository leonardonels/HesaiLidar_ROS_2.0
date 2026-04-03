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
  if (driver_param.input_param.send_point_cloud_ros) {
    pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(driver_param.input_param.ros_send_point_topic, 10);
  }
  if (driver_param.input_param.send_imu_ros) {
    imu_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Imu>(driver_param.input_param.ros_send_imu_topic, 10);
  }

  if (driver_param.input_param.ros_send_packet_loss_topic != NULL_TOPIC) {
    loss_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::LossPacket>(driver_param.input_param.ros_send_packet_loss_topic, 10);
  }

  if (driver_param.input_param.source_type == DATA_FROM_LIDAR) {
    if (driver_param.input_param.ros_send_ptp_topic != NULL_TOPIC) {
      ptp_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::Ptp>(driver_param.input_param.ros_send_ptp_topic, 10);
    }

    if (driver_param.input_param.ros_send_correction_topic != NULL_TOPIC) {
      crt_pub_ = node_ptr_->create_publisher<std_msgs::msg::UInt8MultiArray>(driver_param.input_param.ros_send_correction_topic, 10);
    }
  }
  if (! driver_param.input_param.firetimes_path.empty() ) {
    if (driver_param.input_param.ros_send_firetime_topic != NULL_TOPIC) {
      firetime_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::Firetime>(driver_param.input_param.ros_send_firetime_topic, 10);
    }
  }

  if (driver_param.input_param.send_packet_ros) {
    pkt_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::UdpFrame>(driver_param.input_param.ros_send_packet_topic, 10);
  }

  if (driver_param.input_param.source_type == DATA_FROM_ROS_PACKET) {
    pkt_sub_ = node_ptr_->create_subscription<hesai_ros_driver::msg::UdpFrame>(driver_param.input_param.ros_recv_packet_topic, 10,
                              std::bind(&SourceDriver::ReceivePacket, this, std::placeholders::_1));
    if (driver_param.input_param.ros_recv_correction_topic != NULL_TOPIC) {
      crt_sub_ = node_ptr_->create_subscription<std_msgs::msg::UInt8MultiArray>(driver_param.input_param.ros_recv_correction_topic, 10,
                              std::bind(&SourceDriver::ReceiveCorrection, this, std::placeholders::_1));
    }
    driver_param.decoder_param.enable_udp_thread = false;
    subscription_spin_thread_ = new boost::thread(boost::bind(&SourceDriver::SpinRos2,this));
  }
  driver_ptr_.reset(new HesaiLidarSdk<LidarPointXYZIRT>());
  driver_param.decoder_param.enable_parser_thread = true;
  if (driver_param.input_param.send_point_cloud_ros) {
    driver_ptr_->RegRecvCallback([this](const hesai::lidar::LidarDecodedFrame<hesai::lidar::LidarPointXYZIRT>& frame) {
      this->SendPointCloud(frame);
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
  if (!driver_ptr_->Init(driver_param))
  {
    std::cout << "Driver Initialize Error...." << std::endl;
    exit(-1);
  }


  if (driver_param.custom_param.BARQ_enable) {
    RCLCPP_INFO(node_ptr_->get_logger(), "BARQ shared memory publishing for point clouds is enabled.");

    // Estimate max point cloud size:
    // Max points for your LiDAR model × point_step (26 bytes for XYZIRF64)
    // 128 beams × 1800 azimuth steps = ~230400 points worst case
    const size_t kMaxPoints = 300000;
    const size_t kPointStep = sizeof(float) * 4
                            + sizeof(uint16_t)
                            + sizeof(double);
    const size_t kHeaderBytes = sizeof(uint32_t) * 3
                              + sizeof(double);
    barq_max_size_ = kMaxPoints * kPointStep + kHeaderBytes + 512;

    barq_writer_ = std::make_unique<BARQ::Writer>("/hesai_pointcloud", barq_max_size_, false);

    if (!barq_writer_->init()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to initialize BARQ writer for /hesai_pointcloud, falling back to ROS2 topics only.");
      barq_writer_.reset();
    } else {
      barq_enabled_ = true;
      RCLCPP_INFO(node_ptr_->get_logger(), "BARQ writer initialized for /hesai_pointcloud with max size %zu bytes.", barq_max_size_);
    }
  } else {
    RCLCPP_INFO(node_ptr_->get_logger(), "BARQ shared memory publishing for point clouds is disabled.");
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

  if (barq_writer_) {
    barq_writer_->destroy();
    barq_writer_.reset();
    barq_enabled_ = false;
  }
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
void SourceDriver::SendPointCloud(const LidarDecodedFrame<LidarPointXYZIRT>& msg)
{
  sensor_msgs::msg::PointCloud2 ros_msg = ToRosMsg(msg, frame_id_);
  // For latency testing, we set the timestamp to the current time when the point cloud is received
  if (driver_param.custom_param.latency_testing) {
    auto now = rclcpp::Clock(RCL_ROS_TIME).now();
    ros_msg.header.stamp = now;
  }
  if (driver_param.input_param.send_point_cloud_ros) pub_->publish(ros_msg);

  // Publish via BARQ shared memory if enabled
  if (barq_enabled_ && barq_writer_) {
    // ros_msg.data already contains the packed binary point data
    // We prepend a small fixed header so readers know width/height/point_step
    // without needing to parse a full ROS2 PointCloud2 message.

    // Layout written to SHM:
    //  [0..3]   uint32  width         (number of valid points)
    //  [4..7]   uint32  height        (always 1 for unorganized cloud)
    //  [8..11]  uint32  point_step    (bytes per point, 26)
    //  [12..19] double  stamp_sec     (header stamp as double seconds)
    //  [20..]   uint8[] point data    (width * point_step bytes)

    const size_t header_bytes = sizeof(uint32_t) * 3 + sizeof(double);
    const size_t payload_size = header_bytes + ros_msg.data.size();

    if (payload_size <= barq_max_size_) {
      std::vector<uint8_t> shm_buf(payload_size);

      uint32_t w  = ros_msg.width;
      uint32_t h  = ros_msg.height;
      uint32_t ps = ros_msg.point_step;
      double   ts = ros_msg.header.stamp.sec
                  + ros_msg.header.stamp.nanosec * 1e-9;

      size_t off = 0;

      std::memcpy(shm_buf.data() + off, &w,  sizeof(w));  off += sizeof(w);
      std::memcpy(shm_buf.data() + off, &h,  sizeof(h));  off += sizeof(h);
      std::memcpy(shm_buf.data() + off, &ps, sizeof(ps)); off += sizeof(ps);
      std::memcpy(shm_buf.data() + off, &ts, sizeof(ts)); off += sizeof(ts);
      std::memcpy(shm_buf.data() + off, ros_msg.data.data(), ros_msg.data.size());

      int readers = barq_writer_->write(shm_buf.data(), payload_size);
    } else {
      std::cout << "Point cloud payload (" << payload_size << " bytes) exceeds BARQ max size (" << barq_max_size_ << " bytes), skipping shared memory publish." << std::endl;
    }
  }
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

  RCLCPP_INFO(node_ptr_->get_logger(), "Converting to ROS PointCloud2 message with %u points", frame.points_num);

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
    if (!std::isfinite(frame.points[i].x) || !std::isfinite(frame.points[i].y) || !std::isfinite(frame.points[i].z)){
      n_real_points--;
      continue;
    }

    // filter out car body points if needed: bubble filter
    if(driver_param.custom_param.bubble_filter){
      double dist = std::sqrt((double)frame.points[i].x * frame.points[i].x + (double)frame.points[i].y * frame.points[i].y + (double)frame.points[i].z * frame.points[i].z);
      if (dist <= driver_param.custom_param.car_filter_distance) {
        n_real_points--;
        continue;
      }
    }

    // filter out car body points if needed: cube filter
    if (driver_param.custom_param.cube_filter) {
      if (std::abs(frame.points[i].x) <= driver_param.custom_param.car_filter_distance_x && std::abs(frame.points[i].y) <= driver_param.custom_param.car_filter_distance_y && std::abs(frame.points[i].z) <= driver_param.custom_param.car_filter_distance_z) {
        n_real_points--;
        continue;
      }
    }

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
  ros_msg.linear_acceleration.x = From_g_To_ms2(imu_config_.imu_accel_x);
  ros_msg.linear_acceleration.y = From_g_To_ms2(imu_config_.imu_accel_y);
  ros_msg.linear_acceleration.z = From_g_To_ms2(imu_config_.imu_accel_z);
  ros_msg.angular_velocity.x = From_degs_To_rads(imu_config_.imu_ang_vel_x);
  ros_msg.angular_velocity.y = From_degs_To_rads(imu_config_.imu_ang_vel_y);
  ros_msg.angular_velocity.z = From_degs_To_rads(imu_config_.imu_ang_vel_z);
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
