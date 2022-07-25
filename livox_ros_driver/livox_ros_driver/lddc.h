//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
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
#ifndef LIVOX_ROS_DRIVER_LDDC_H_
#define LIVOX_ROS_DRIVER_LDDC_H_

#include "lds.h"
#include "livox_sdk.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <pcl_ros/point_cloud.h>
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>

namespace livox_ros {

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

/** Lidar data distribute control */
typedef enum {
  kPointCloud2Msg = 0,
  kLivoxCustomMsg = 1,
  kPclPxyziMsg
} TransferType;

class Lddc {
 public:
  Lddc(
    const int format,
    const int multi_topic,
    const int data_src,
    const int output_type,
    double frq,
    const std::string& lidar_frame_id,
    const std::string& imu_frame_id,
    const bool lidar_bag,
    const bool imu_bag);
  ~Lddc();

  int RegisterLds(Lds *lds);
  void DistributeLidarData(void);
  void CreateBagFile(const std::string &file_name);
  void PrepareExit(void);

  uint8_t GetTransferFormat(void) { return transfer_format_; }
  uint8_t IsMultiTopic(void) { return use_multi_topic_; }
  void SetRosNodeHandlers(ros::NodeHandle *node, ros::NodeHandle *private_node);
  void SetImuCovariances();

  void SetRosPub(ros::Publisher *pub) { global_pub_ = pub; };
  void SetPublishFrq(uint32_t frq) { publish_frq_ = frq; }

  Lds *lds_;

 private:
  int32_t GetPublishStartTime(LidarDevice *lidar, LidarDataQueue *queue,
                              uint64_t *start_time,
                              StoragePacket *storage_packet);
  uint32_t PublishPointcloud2(LidarDataQueue *queue, uint32_t packet_num,
                              uint8_t handle);
  uint32_t PublishPointcloudData(LidarDataQueue *queue, uint32_t packet_num,
                                 uint8_t handle);
  uint32_t PublishCustomPointcloud(LidarDataQueue *queue, uint32_t packet_num,
                                   uint8_t handle);
  uint32_t PublishImuData(LidarDataQueue *queue, uint32_t packet_num,
                          uint8_t handle);

  ros::Publisher *GetCurrentPublisher(uint8_t handle);
  ros::Publisher *GetCurrentImuPublisher(uint8_t handle);
  void PollingLidarPointCloudData(uint8_t handle, LidarDevice *lidar);
  void PollingLidarImuData(uint8_t handle, LidarDevice *lidar);
  void InitPointcloud2MsgHeader(sensor_msgs::PointCloud2& cloud);
  void FillPointsToPclMsg(boost::shared_ptr<PointCloud>& pcl_msg, \
      LivoxPointXyzrtl* src_point, uint32_t num);
  void FillPointsToCustomMsg(livox_ros_driver::CustomMsg& livox_msg, \
      LivoxPointXyzrtl* src_point, uint32_t num, uint32_t offset_time, \
      uint32_t point_interval, uint32_t echo_num);
  uint8_t transfer_format_;
  uint8_t use_multi_topic_;
  uint8_t data_src_;
  uint8_t output_type_;
  double publish_frq_;
  uint32_t publish_period_ns_;
  std::string lidar_frame_id_;
  bool enable_lidar_bag_;
  bool enable_imu_bag_;
  ros::Publisher *private_pub_[kMaxSourceLidar];
  ros::Publisher *global_pub_;
  ros::Publisher *private_imu_pub_[kMaxSourceLidar];
  ros::Publisher *global_imu_pub_;

  // Covariances
  std::vector<double> angular_velocity_cov_;
  std::vector<double> linear_acceleration_cov_;
  const std::vector<double> kDefaultAngularVelocityCov_{9, 0.0};
  const std::vector<double> kDefaultLinearAccelerationCov_{9, 0.0};

  ros::NodeHandle *cur_node_;
  ros::NodeHandle *private_node_;
  sensor_msgs::Imu imu_data_;
  rosbag::Bag *bag_;

  const double kGravity_{ 9.80665 };
};

}  // namespace livox_ros
#endif
