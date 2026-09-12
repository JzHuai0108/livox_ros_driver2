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

#ifndef LIVOX_ROS_DRIVER2_LDDC_TOP_H_
#define LIVOX_ROS_DRIVER2_LDDC_TOP_H_

#include "livox_ros_driver2.h"

#include "driver_node.h"

namespace livox_ros {

/** Send pointcloud message Data to ros subscriber or save them in rosbag file */
typedef enum {
  kOutputToRos = 0,
  kOutputToRosBagFile = 1,
} DestinationOfMessageOutput;

/** The message type of transfer */
typedef enum {
  kPointCloud2Msg = 0,
  kLivoxCustomMsg = 1,
  kPclPxyziMsg = 2,
  kLivoxImuMsg = 3,
} TransferType;

/** Type-Definitions based on ROS versions */
#ifdef BUILDING_ROS1
using Publisher = ros::Publisher;
using PublisherPtr = ros::Publisher*;
using PointCloud2 = sensor_msgs::PointCloud2;
using PointField = sensor_msgs::PointField;
using CustomMsg = livox_ros_driver2::CustomMsg;
using CustomPoint = livox_ros_driver2::CustomPoint;
using ImuMsg = sensor_msgs::Imu;
#elif defined BUILDING_ROS2
template <typename MessageT> using Publisher = rclcpp::Publisher<MessageT>;
using PublisherPtr = std::shared_ptr<rclcpp::PublisherBase>;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using CustomMsg = livox_ros_driver2::msg::CustomMsg;
using CustomPoint = livox_ros_driver2::msg::CustomPoint;
using ImuMsg = sensor_msgs::msg::Imu;
#endif

}  // namespace livox_ros

#endif // LIVOX_ROS_DRIVER2_LDDC_TOP_H_
