```cpp
struct LidarPointXYZIRT
{
    float x; 
    float y;             
    float z;             
    uint8_t intensity;  
    uint16_t ring;
    double timestamp;  
};

template <typename PointT>
class LidarDecodedFrame
{
    public:
    LidarDecodedFrame(uint16_t maxPacketNum = 5000, uint16_t maxNumPerPacket = 1024) {
        resetMalloc(maxPacketNum, maxNumPerPacket);
        lidar_state = -1;
        work_mode = -1;
        packet_num = 0;
        frame_index = 0;
        block_num = 0;
        laser_num = 0; 
        channel_num = 0;
        per_points_num = 0;
        distance_unit = 0.0;
        return_mode = 0;
        points_num = 0;
        frame_start_timestamp = 0;
        frame_end_timestamp = 0;
        scan_complete = false;
        multi_packet_num = 0;
        multi_points_num = 0;
    };
    ~LidarDecodedFrame() {
        if (total_memory) {
            delete[] total_memory;
            total_memory = nullptr;
            packetData = nullptr;
            points = nullptr;
            funcSafety = nullptr;
        }
        if (valid_points) {
          delete[] valid_points;
          valid_points = nullptr;
        }
        if (point_cloud_raw_data) {
          delete[] point_cloud_raw_data;
          point_cloud_raw_data = nullptr;
        }
        if (multi_frame_buffer) {
          delete[] multi_frame_buffer;
          multi_frame_buffer = nullptr;
          multi_points = nullptr;
        }
    }
    void resetMalloc(uint16_t maxPacketNum, uint16_t maxNumPerPacket) {
      maxPacketPerFrame = maxPacketNum;
      maxPointPerPacket = maxNumPerPacket;
      if (total_memory) {
          delete[] total_memory;
          total_memory = nullptr;
          packetData = nullptr;
          points = nullptr;
          funcSafety = nullptr;
      }
      if (valid_points) {
        delete[] valid_points;
      }
      total_memory = new uint8_t[sizeof(PacketDecodeData) * maxPacketPerFrame
                                   + sizeof(PointT) * maxPacketPerFrame * maxPointPerPacket
                                   + sizeof(FunctionSafety) * maxPacketPerFrame];
      memset(total_memory, 0, sizeof(PacketDecodeData) * maxPacketPerFrame
                                   + sizeof(PointT) * maxPacketPerFrame * maxPointPerPacket
                                   + sizeof(FunctionSafety) * maxPacketPerFrame);
      uint32_t offset = 0;
      packetData = reinterpret_cast<PacketDecodeData* >(total_memory + offset);
      offset += (sizeof(PacketDecodeData) * maxPacketPerFrame);
      points = reinterpret_cast <PointT* >(total_memory + offset);
      offset += (sizeof(PointT) * maxPacketPerFrame * maxPointPerPacket);
      funcSafety = reinterpret_cast <FunctionSafety* >(total_memory + offset);
      offset += (sizeof(FunctionSafety) * maxPacketPerFrame);
      valid_points = new uint32_t[maxPacketPerFrame];
      if (fParam.IsMultiFrameFrequency() == 1) {
        multi_rate = fParam.default_frame_frequency / fParam.frame_frequency;
        int multi_num = multi_rate + 1;
        multi_frame_buffer = new uint8_t[sizeof(PointT) * maxPacketPerFrame * maxPointPerPacket * multi_num];
        multi_points = reinterpret_cast <PointT* >(multi_frame_buffer);
        LogInfo("fParam.frame_frequency: %lf, multi_rate : %d", fParam.frame_frequency, multi_rate);
      }
    }
    LidarDecodedFrame(const LidarDecodedFrame&) = delete;
    LidarDecodedFrame& operator=(const LidarDecodedFrame&) = delete;
    void Update() {
        packet_num = 0;
        points_num = 0;
        frame_start_timestamp = 0;
        frame_end_timestamp = 0;
        scan_complete = false;
        frame_index++;
        std::fill(et_echo_vec.begin(), et_echo_vec.end(), false);
    }
    void clearFuncSafety() {
      memset(funcSafety, 0, sizeof(FunctionSafety) * maxPacketPerFrame);
    }
    uint32_t getPointSize() const {
      return sizeof(PointT);
    }
    void MultiFrameUpdate() {
      multi_packet_num = 0;
      multi_points_num = 0;
      multi_frame_start_timestamp = 0;
      multi_frame_end_timestamp = 0;
      multi_frame_index++;
    }
    uint8_t* total_memory = nullptr; 
    uint32_t maxPacketPerFrame;
    uint32_t maxPointPerPacket;
    // configure
    FrameDecodeParam fParam;

    // frame parameter
    int16_t lidar_state;
    int16_t work_mode;
    uint16_t return_mode;
    uint32_t packet_num; 
    uint32_t* valid_points = nullptr;
    int frame_index;
    uint32_t points_num;
    double frame_start_timestamp;
    double frame_end_timestamp;
    std::string software_version = "xx.xx.xx";
    std::string hardware_version = "xx.xx.xx";
    // package parameter
    PacketDecodeData* packetData = nullptr; 
    FunctionSafety* funcSafety = nullptr;
    uint16_t block_num;
    uint16_t laser_num;  // channel number in point cloud
    uint16_t channel_num; // real channel number, when != 0, mean useful channel number
    uint32_t per_points_num; 
    uint8_t reserved[4];
    double distance_unit;
    bool scan_complete;
    // point parameter
    PointT* points = nullptr;
    //special output
    LidarImuData imu_config;
    // point cloud raw data
    bool frame_init_ = false;
    uint32_t point_cloud_size = 0;
    uint8_t* point_cloud_raw_data = nullptr;
    std::vector<bool> et_echo_vec;
    bool clear_every_frame = false;
    uint8_t *multi_frame_buffer = nullptr;
    PointT* multi_points = nullptr;
    uint32_t multi_packet_num = 0;
    uint32_t multi_points_num = 0;
    double multi_frame_start_timestamp = 0;
    double multi_frame_end_timestamp = 0;
    int multi_frame_index = 0;
    int multi_rate = 1;
};
```

```cpp
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
```

NOTE: probbaly i should create a shm_pcl object or whatever should be called
-> a modern and correct approch should be to create a class with a method to_shm or something similar
-> but to mirrir what the original dev have done i should create a struct and have the converting function along side the ros one

Option A
```cpp
struct shm_msg
{
    uint32_t w  = ros_msg.width;
    uint32_t h  = ros_msg.height;
    uint32_t ps = ros_msg.point_step;
    double   ts = ros_msg.header.stamp.sec
                + ros_msg.header.stamp.nanosec * 1e-9;
    "float array for data?"
}
```

Option B
```cpp
class SharedMemoryLidarFrame
{
    public:
    uint32_t w  = ros_msg.width;
    uint32_t h  = ros_msg.height;
    uint32_t ps = ros_msg.point_step;
    double   ts = ros_msg.header.stamp.sec
                + ros_msg.header.stamp.nanosec * 1e-9;
    "flat float array or vector/array of points"
}
```

Option C
```cpp
// ── barq_point.hpp (new file, no ROS dependency, consumers can include this) ──

#pragma once
#include <cstdint>

struct __attribute__((packed)) BARQPoint {
    float    x;
    float    y;
    float    z;
    float    intensity;   // promoted from uint8_t on write
    uint16_t ring;
    double   timestamp;
};  // 26 bytes

struct __attribute__((packed)) BARQFrameHeader {
    uint32_t width;       // number of valid points
    uint32_t height;      // always 1
    uint32_t point_step;  // always sizeof(BARQPoint) = 26
    double   timestamp;   // frame start timestamp (seconds)
};  // 20 bytes

// SHM layout: [BARQFrameHeader][BARQPoint × width]
// Total size:  sizeof(BARQFrameHeader) + width * sizeof(BARQPoint)

// ── source_driver_ros2.hpp: add alongside SendPointCloud ──

#ifdef ENABLE_BARQ
  void SendPointCloudWithBarq(const LidarDecodedFrame<LidarPointXYZIRT>& frame);
#endif

// ── source_driver_ros2.cpp: new free function, parallel to ToRosMsg ──

#ifdef ENABLE_BARQ
void SourceDriver::SendPointCloudWithBarq(const LidarDecodedFrame<LidarPointXYZIRT>& frame)
{
  if (!barq_enabled_ || !barq_writer_) return;

  const size_t payload_size = sizeof(BARQFrameHeader)
                            + frame.points_num * sizeof(BARQPoint);
  if (payload_size > barq_max_size_) {
    RCLCPP_WARN_ONCE(node_ptr_->get_logger(),
      "BARQ payload %zu > max %zu, skipping frame", payload_size, barq_max_size_);
    return;
  }

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

  barq_writer_->write(buf.data(), payload_size);
}
#endif

// ── source_driver_ros2.cpp: Init() — register callbacks independently ──

if (driver_param.input_param.send_point_cloud_ros) {
  driver_ptr_->RegRecvCallback([this](const LidarDecodedFrame<LidarPointXYZIRT>& frame) {
    this->SendPointCloudWithRos(frame);
  });
}
#ifdef ENABLE_BARQ
if (barq_enabled_) {
  driver_ptr_->RegRecvCallback([this](const LidarDecodedFrame<LidarPointXYZIRT>& frame) {
    this->SendPointCloudWithBarq(frame);
  });
}
#endif
```

OT128 -> `LidarDecodedFrame<LidarPointXYZIRT>`                          -> `sensor_msgs::msg::PointCloud2`
                                                                        -> `std::vector<uint8_t> shm_buf(payload_size)`
                                                -> car_filter (cuda?)

OT128 ->CPU (UDP recv) → CPU (ring buffer) -> GPU (decode) -> CPU (frame assembly) -> CPU (filter) -> CPU (share)
To move from actual bubble/cube filter implementations to cuda i need to move back the filter before the decode process 
OT128 ->CPU (UDP recv) → CPU (ring buffer) -> GPU (decode) -> GPU (cuda_kernel_filter) -> CPU (frame assembly) -> CPU (share)