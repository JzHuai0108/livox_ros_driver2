//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef LIVOX_DRIVER_NODE_H
#define LIVOX_DRIVER_NODE_H

#include "ros_headers.h"

namespace livox_ros {

class Lddc;

#ifdef BUILDING_ROS1
class DriverNode final : public ros::NodeHandle {
 public:
  DriverNode();
  DriverNode(const DriverNode &) = delete;
  ~DriverNode();
  DriverNode &operator=(const DriverNode &) = delete;

  DriverNode& GetNode() noexcept;
  bool controlCallback(livox_ros_driver2::StartStop::Request &req,
                       livox_ros_driver2::StartStop::Response &res);
  void PointCloudDataPollThread();
  void ImuDataPollThread();
  void setFuture();
  void setLddc(int xfer_format, int multi_topic, int data_src, int output_type,
      double publish_freq, std::string frame_id, bool lidar_bag, bool imu_bag);
  void registerLds(double publish_freq, const std::string &user_config_path);
  void StartRecording(const std::string &bagname);
  void StopRecording();

  std::shared_ptr<Lddc> lddc_ptr_;
  std::shared_ptr<std::thread> pointclouddata_poll_thread_;
  std::shared_ptr<std::thread> imudata_poll_thread_;
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;

  ros::ServiceServer control_service_;
  bool recording_;
  std::string current_filename_;
};

#elif defined BUILDING_ROS2
class DriverNode final : public rclcpp::Node {
 public:
  explicit DriverNode(const rclcpp::NodeOptions& options);
  DriverNode(const DriverNode &) = delete;
  ~DriverNode();
  DriverNode &operator=(const DriverNode &) = delete;

  DriverNode& GetNode() noexcept;

 private:
  void PointCloudDataPollThread();
  void ImuDataPollThread();

  std::unique_ptr<Lddc> lddc_ptr_;
  std::shared_ptr<std::thread> pointclouddata_poll_thread_;
  std::shared_ptr<std::thread> imudata_poll_thread_;
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;
};
#endif

} // namespace livox_ros

#endif // LIVOX_DRIVER_NODE_H