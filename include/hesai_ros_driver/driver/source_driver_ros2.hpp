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
 * File: source_driver_ros2.hpp
 * Author: Zhang Yu <zhangyu@hesaitech.com>
 * Description: Source Driver for ROS2
 * Created on June 12, 2023, 10:46 AM
 */

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <chrono>
#ifdef LATENCY_TESTING
#include <mmr_base/msg/latency_sample.hpp>
#endif
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <map>
#include <sstream>
#include <hesai_ros_driver/msg/udp_frame.hpp>
#include <hesai_ros_driver/msg/udp_packet.hpp>
#include <hesai_ros_driver/msg/ptp.hpp>
#include <hesai_ros_driver/msg/firetime.hpp>
#include <hesai_ros_driver/msg/loss_packet.hpp>

#include <fstream>
#include <memory>
#include <chrono>
#include <string>
#include <functional>
#include <boost/thread.hpp>
#include "hesai_ros_driver/driver/source_drive_common.hpp"

#ifdef ENABLE_BARQ
// BARQ (Burst Access Reader Queue) is a lightweight shared memory library for high-throughput, low-latency data exchange between processes.
#include "barq/barq.hpp"
#include "barq/barq_pcl.hpp"
#endif
#ifdef ENABLE_ZERO_COPY
#include <mmr_base/msg/bounded_pointcloud.hpp>
#endif

class SourceDriver
{
public:
  typedef std::shared_ptr<SourceDriver> Ptr;
  // Initialize some necessary configuration parameters, create ROS nodes, and register callback functions
  virtual void Init(const YAML::Node& config);
  // Start working
  virtual void Start();
  // Stop working
  virtual void Stop();
  virtual ~SourceDriver();
  SourceDriver(SourceType src_type) {};
  void SpinRos2(){rclcpp::spin(this->node_ptr_);}
  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::shared_ptr<HesaiLidarSdk<LidarPointXYZIRT>> driver_ptr_;
protected:
  // Save Correction file subscribed by "ros_recv_correction_topic"
  void ReceiveCorrection(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  // Save packets subscribed by 'ros_recv_packet_topic'
  void ReceivePacket(const hesai_ros_driver::msg::UdpFrame::SharedPtr msg);
  // Used to publish point clouds through 'ros_send_point_cloud_topic'
  void SendPointCloudWithRos(const LidarDecodedFrame<LidarPointXYZIRT>& msg);
  // Used to publish the original pcake through 'ros_send_packet_topic'
  void SendPacket(const UdpFrame_t&  ros_msg, double timestamp);

  // Used to publish the Correction file through 'ros_send_correction_topic'
  void SendCorrection(const u8Array_t& msg);
  // Used to publish the Packet loss condition
  void SendPacketLoss(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count);
  // Used to publish the Packet loss condition
  void SendPTP(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status);
  // Used to publish the firetime correction 
  void SendFiretime(const double *firetime_correction_);
  // Used to publish the imu packet
  void SendImuConfig(const LidarImuData& msg);

  // --- OT128 temperature publishing ------------------------------------------
  // PTC path: receives a parsed LidarStatus (8 board/laser temps) and publishes.
  void OnLidarStatus(const hesai::lidar::LidarStatus& status);
  // udp_tail path: reads temperatures from the raw UDP packet tail of a frame
  // (rotating status ID slots + optional IMU temp) for sources with no live PTC.
  void ExtractTailTemps(const UdpFrame_t& frame, double timestamp);
  // Shared publisher: builds the DiagnosticArray (+ optional per-sensor
  // Temperature msgs) and computes OK/WARN/ERROR from the configured thresholds.
  void PublishTemps(const std::map<std::string, double>& temps, const rclcpp::Time& stamp);

#ifdef ENABLE_BARQ
  std::vector<uint8_t> SerializePointCloudForBarq(const LidarDecodedFrame<LidarPointXYZIRT>& frame);
  void SendPointCloudWithBarq(const LidarDecodedFrame<LidarPointXYZIRT>& frame);
#endif

  // Convert ptp lock offset, status into ROS message
  hesai_ros_driver::msg::Ptp ToRosMsg(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status);
  // Convert packet loss condition into ROS message
  hesai_ros_driver::msg::LossPacket ToRosMsg(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count);
  // Convert correction string into ROS messages
  std_msgs::msg::UInt8MultiArray ToRosMsg(const u8Array_t& correction_string);
  // Convert double[512] to float64[512]
  hesai_ros_driver::msg::Firetime ToRosMsg(const double *firetime_correction_);
  // Convert point clouds into ROS messages
  sensor_msgs::msg::PointCloud2 ToRosMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id);
#ifdef ENABLE_ZERO_COPY
  /* Fill a LOANED BoundedPointcloud from the decoded frame and publish it.
     Deliberately not a ToBoundedMsg() returning by value: that is what this used
     to be, and it made the "zero copy" path the most expensive one in the
     driver. A 3.4 MB std::array cannot be returned, assigned or const-ref
     published without being copied, so every frame paid a 3.4 MB memset for the
     caller's local, a 3.4 MB copy out of the loan, a 3.4 MB move-assign and a
     3.4 MB copy back into a fresh loan at publish -- ~190 MB/s of memory traffic
     at 19 Hz that the plain ROS path never paid, charged to this process. The
     loan has to be filled in place and published with std::move. */
  void SendBoundedPointcloud(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id);
#endif
  /* Publish the out-of-band frame tick, if custom_param.frame_tick_topic is set.
     `stamp` is the cloud's capture stamp -- the same value that propagates down
     the stack -- so a consumer can join a tick to a downstream output. time_ref
     carries CLOCK_MONOTONIC at publish; source is "hesai:<transport>:<width>". */
  /* `mono_ns` is CLOCK_MONOTONIC read by the CALLER, immediately before it
     publishes the cloud. Passed in rather than read here so that the tick and
     the latency sample below carry the SAME instant -- two readings taken a few
     microseconds apart would silently disagree about when the frame was sent. */
  void PublishFrameTick(const builtin_interfaces::msg::Time& stamp, uint32_t width,
                        int64_t mono_ns);

  static int64_t MonotonicNs()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

#ifdef LATENCY_TESTING
  /* The same per-frame stats message every stack node publishes, with t_in left
     at zero: the driver is the origin, so it has nothing to receive. That
     uniformity is the point -- the monitor gets one message type for every hop
     of the chain instead of a special case for the first one, and it is what a
     per-transport publish (one sample per path the driver wrote) will extend.
     The TimeReference tick above stays: as_demo's replayer synchronisation
     waits on it, and it carries the transport name and point count that this
     message does not. */
  void PublishLatencySample(const builtin_interfaces::msg::Time& stamp, int64_t t_out);
#endif
  // Convert packets into ROS messages
  hesai_ros_driver::msg::UdpFrame ToRosMsg(const UdpFrame_t& ros_msg, double timestamp);
  // Convert imu, imu into ROS message
  sensor_msgs::msg::Imu ToRosMsg(const LidarImuData& firetime_correction_);
  // Convert Linear Acceleration from g to m/s^2
  double From_g_To_ms2(double g);
  // Convert Angular Velocity from degree/s to radian/s
  double From_degs_To_rads(double degree);
  std::string frame_id_;
  // store the driver start time when real_time_timestamp is true
  double driver_start_timestamp_;
  // store driver parameters including custom fields (bubble/cube filters)
  hesai::lidar::CustomDriverParam driver_param;

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr crt_sub_;
  rclcpp::Subscription<hesai_ros_driver::msg::UdpFrame>::SharedPtr pkt_sub_;
  rclcpp::Publisher<hesai_ros_driver::msg::UdpFrame>::SharedPtr pkt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
#ifdef ENABLE_ZERO_COPY
  rclcpp::Publisher<mmr_base::msg::BoundedPointcloud>::SharedPtr bounded_pub_;
#endif
  rclcpp::Publisher<hesai_ros_driver::msg::Firetime>::SharedPtr firetime_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr crt_pub_;
  rclcpp::Publisher<hesai_ros_driver::msg::LossPacket>::SharedPtr loss_pub_;
  rclcpp::Publisher<hesai_ros_driver::msg::Ptp>::SharedPtr ptp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  // Null unless custom_param.frame_tick_topic is set. See PublishFrameTick.
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr tick_pub_;
  std::string tick_source_;
  /* Exactly one send path may tick, and this says which. The driver can write a
     ROS2 cloud and a BARQ frame from the same callback, so without an owner both
     would tick and every downstream consumer would see two t0s per frame. The
     ROS/zero-copy path owns it whenever send_point_cloud_ros is on; BARQ owns it
     only when it is the sole transport, which is the configuration a BARQ
     measurement should be taken in anyway. */
  bool tick_owner_barq_ = false;
#ifdef LATENCY_TESTING
  rclcpp::Publisher<mmr_base::msg::LatencySample>::SharedPtr latency_sample_pub_;
  uint32_t latency_seq_ = 0;
#endif

  // Temperature publishers + state. temp_diag_pub_ is the primary aggregated
  // output; temp_sensor_pubs_ holds one optional sensor_msgs/Temperature pub per
  // label. temp_cache_ keeps the latest value per label because udp_tail status
  // fields rotate across frames (only a few IDs appear in each packet).
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr temp_diag_pub_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr> temp_sensor_pubs_;
  std::map<std::string, double> temp_cache_;
  // Resolved at Init: true => publish, "ptc" or "udp_tail" effective source.
  bool temp_enabled_ = false;
  std::string temp_effective_source_;

  //spin thread while Receive data from ROS topic
  boost::thread* subscription_spin_thread_;

  bool barq_enabled_ = false;
  bool zero_copy_enabled_ = false;
#ifdef ENABLE_BARQ
  // BARQ writer for shared memory publishing of point clouds (optional, alongside ROS2 topics)
  std::unique_ptr<BARQ::Writer> barq_writer_;
  std::string barq_topic_;
  size_t barq_max_size_ = 0;
#endif
};