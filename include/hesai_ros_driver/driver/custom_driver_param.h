// inherit from HesaiLidar_SDK_2.0/libhesai/driver_param.h
// then add car points filter options
/************************************************************************************************
 * Car points filter options
 ************************************************************************************************/
#pragma once
#include <cstdint>
#include <map>
#include <string>
#include "driver_param.h"

namespace hesai {
namespace lidar {

// One entry of the UDP-tail status-ID -> temperature mapping. The raw 16-bit
// status data is converted to degrees Celsius as: value * scale + offset.
struct TempSensorEntry
{
  std::string label;
  double scale = 1.0;
  double offset = 0.0;
};

typedef struct CustomParam
{
  bool real_time_timestamp = false;
  bool latency_testing = false;
  bool zero_copy_enabled = false;
  // Out-of-band frame tick. Empty (the default, and what the car runs) publishes
  // nothing. When set, one sensor_msgs/TimeReference goes out immediately after
  // every point cloud, carrying the cloud's capture stamp in header.stamp and
  // CLOCK_MONOTONIC-at-publish in time_ref. It exists so a profiler can time a
  // frame from the instant it was PUBLISHED rather than from the instant the
  // profiler itself managed to receive it -- the latter is transport-dependent,
  // which makes it useless for comparing transports. See as_demo's monitor.
  std::string frame_tick_topic = "";
#ifdef ENABLE_BARQ
  bool BARQ_enable = false;
  std::string BARQ_topic = "/lidar_points";
#endif
  bool bubble_filter = false;
  float car_filter_distance_sq = 0.0;

  // OT128 temperature publishing (see config.yaml `temperature:` block).
  bool        temp_enable = false;
  std::string temp_source = "auto";          // auto | ptc | udp_tail
  std::string temp_topic = "/lidar/temperature";
  bool        temp_publish_sensor_msgs = false;
  std::string temp_sensor_msgs_prefix = "/lidar/temperature/";
  double      temp_ptc_poll_period_s = 1.0;
  double      temp_warn_c = 75.0;
  double      temp_error_c = 90.0;
  bool        temp_include_imu = true;
  // Maps UDP-tail status IDs (rotating reserved slots) to named temperature
  // sensors. Empty by default (fail-safe: unknown IDs publish nothing) -- fill
  // from the OT128 manual section 3.1.2.5.
  std::map<uint8_t, TempSensorEntry> udp_status_id_map;
} CustomParam;

// New type that extends the SDK DriverParam with custom fields
struct CustomDriverParam : public hesai::lidar::DriverParam
{
  CustomParam custom_param;
};

} // namespace lidar
} // namespace hesai