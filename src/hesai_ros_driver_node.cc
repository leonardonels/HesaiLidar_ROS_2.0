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
 * File: hesai_ros_driver_node.cc
 * Author: Zhang Yu <zhangyu@hesaitech.com>
 * Description: Hesai sdk node for CPU
 * Created on June 12, 2023, 10:46 AM
 */

#include "hesai_ros_driver/node_manager.h"
#include <signal.h>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>
#include "Version.h"

std::mutex g_mtx;
std::condition_variable g_cv;

bool sig_recv = false;
static void sigHandler(int sig)
{
  sig_recv = true;
  g_cv.notify_all();
}

int main(int argc, char** argv)
{
  std::cout << "-------- Hesai Lidar ROS V" << VERSION_MAJOR << "." << VERSION_MINOR << "." << VERSION_TINY << " --------" << std::endl;
  signal(SIGINT, sigHandler);  ///< bind ctrl+c signal with the sigHandler function
  rclcpp::init(argc, argv);

  std::string config_path = (std::string)PROJECT_PATH;
  config_path += "/config/config.yaml";

  // workaround to get config_path from ros parameter
  auto node = rclcpp::Node::make_shared("hesai_ros_driver_node");
  std::string path = node->declare_parameter<std::string>("config_path", "");
  node.reset();
  if (!path.empty())
  {
    config_path = path;
  }

  YAML::Node config;
  config = YAML::LoadFile(config_path);
  std::shared_ptr<NodeManager> demo_ptr = std::make_shared<NodeManager>();
  demo_ptr->Init(config);
  demo_ptr->Start();
  // you can chose [!demo_ptr->IsPlayEnded()] or [1]
  // If you chose !demo_ptr->IsPlayEnded(), ROS node will end with the end of the PCAP.
  // If you select 1, the ROS node does not end with the end of the PCAP.
  /* 100us here (upstream) made this thread wake ~6600 times a second to poll a
     flag, for ~1% of a core and nothing else -- measured on this Orin, and it is
     BY FAR the biggest single source of context switches in the process (6592/s
     of ~7700 total; the receive thread's 1ms backpressure loop is the next at
     971/s). It is not a pcap-replay artefact: is_pcap_end is only ever set by
     PcapSource, so on the car this loop polls a permanently-false flag at 10 kHz
     for the whole run.

     That churn is not free to anyone else either. Interleaved with fast_LIMO's
     21 threads it costs fast_LIMO continuity rather than CPU share -- its
     involuntary preemptions go 91/s (no driver) to 151/s (driver replaying a
     pcap), and a filter whose deskew and IMU prior are timing-sensitive is
     exactly what that hurts.

     20ms polls 50 times a second instead. Nothing downstream can tell: the only
     consumer of IsPlayEnded() is this loop, and the function itself already
     sleeps 3 SECONDS once it returns true. */
  while (!demo_ptr->IsPlayEnded() && sig_recv == false)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  demo_ptr->Stop();
  return 0;
}
