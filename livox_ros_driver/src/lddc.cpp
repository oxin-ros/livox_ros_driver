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

#include "lddc.h"

#include <inttypes.h>
#include <math.h>
#include <stdint.h>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosfmt/full.h>

#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>
#include "lds_lidar.h"
#include "lds_lvx.h"
#include "ldcc_config.h"

namespace livox_ros
{

    /** Lidar Data Distribute Control--------------------------------------------*/
    Lddc::Lddc(const LdccConfig &LdccConfig) :
        config_(LdccConfig)
    {
        publish_period_ns_ = kNsPerSecond / config_.frequency;
        lds_ = nullptr;
        cur_node_ = nullptr;
        private_node_ = nullptr;
        bag_ = nullptr;

        // Initialise IMU message with constant values
        imu_data_.header.frame_id.assign(config_.imu_frame_id);
    };

    Lddc::~Lddc()
    {
        if (lds_)
        {
            lds_->PrepareExit();
        }
    }

    void Lddc::SetRosNodeHandlers(ros::NodeHandle *node, ros::NodeHandle *private_node)
    {
        cur_node_ = node;
        private_node_ = private_node;
        ConfigureLidarPublishers();
        ConfigureImuPublishers();
    }

    void Lddc::SetImuCovariances()
    {
        // Fill in covariance matrices
        private_node_->param("angular_velocity_cov", angular_velocity_cov_, kDefaultAngularVelocityCov_);
        private_node_->param("linear_acceleration_cov", linear_acceleration_cov_, kDefaultLinearAccelerationCov_);

        // Given that the IMU doesn't produce an orientation estimate,
        // the element 0 of the associated covariance matrix is -1
        imu_data_.orientation_covariance[0] = -1;

        std::copy(angular_velocity_cov_.begin(), angular_velocity_cov_.end(),
                  imu_data_.angular_velocity_covariance.begin());

        std::copy(linear_acceleration_cov_.begin(), linear_acceleration_cov_.end(),
                  imu_data_.linear_acceleration_covariance.begin());
    }

    int32_t Lddc::GetPublishStartTime(LidarDevice *lidar, LidarDataQueue *queue,
                                      uint64_t *start_time,
                                      StoragePacket *storage_packet)
    {
        QueuePrePop(queue, storage_packet);
        uint64_t timestamp =
            GetStoragePacketTimestamp(storage_packet, lidar->data_src);
        uint32_t remaining_time = timestamp % publish_period_ns_;
        uint32_t diff_time = publish_period_ns_ - remaining_time;
        /** Get start time, down to the period boundary */
        if (diff_time > (publish_period_ns_ / 4))
        {
            // ROS_INFO("0 : %u", diff_time);
            *start_time = timestamp - remaining_time;
            return 0;
        }
        else if (diff_time <= lidar->packet_interval_max)
        {
            *start_time = timestamp;
            return 0;
        }
        else
        {
            /** Skip some packets up to the period boundary*/
            // ROS_INFO("2 : %u", diff_time);
            do
            {
                if (QueueIsEmpty(queue))
                {
                    break;
                }
                QueuePopUpdate(queue); /* skip packet */
                QueuePrePop(queue, storage_packet);
                uint32_t last_remaning_time = remaining_time;
                timestamp = GetStoragePacketTimestamp(storage_packet, lidar->data_src);
                remaining_time = timestamp % publish_period_ns_;
                /** Flip to another period */
                if (last_remaning_time > remaining_time)
                {
                    // ROS_INFO("Flip to another period, exit");
                    break;
                }
                diff_time = publish_period_ns_ - remaining_time;
            } while (diff_time > lidar->packet_interval);

            /* the remaning packets in queue maybe not enough after skip */
            return -1;
        }
    }

    void Lddc::InitPointcloud2MsgHeader(sensor_msgs::PointCloud2 &cloud)
    {
        cloud.header.frame_id.assign(config_.lidar_frame_id);
        cloud.header.stamp = ros::Time::now();
        cloud.height = 1;
        cloud.width = 0;
        cloud.fields.resize(6);
        cloud.fields[0].offset = 0;
        cloud.fields[0].name = "x";
        cloud.fields[0].count = 1;
        cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        cloud.fields[1].offset = 4;
        cloud.fields[1].name = "y";
        cloud.fields[1].count = 1;
        cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        cloud.fields[2].offset = 8;
        cloud.fields[2].name = "z";
        cloud.fields[2].count = 1;
        cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        cloud.fields[3].offset = 12;
        cloud.fields[3].name = "intensity";
        cloud.fields[3].count = 1;
        cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        cloud.fields[4].offset = 16;
        cloud.fields[4].name = "tag";
        cloud.fields[4].count = 1;
        cloud.fields[4].datatype = sensor_msgs::PointField::UINT8;
        cloud.fields[5].offset = 17;
        cloud.fields[5].name = "line";
        cloud.fields[5].count = 1;
        cloud.fields[5].datatype = sensor_msgs::PointField::UINT8;
        cloud.point_step = sizeof(LivoxPointXyzrtl);
    }

    uint32_t Lddc::PublishPointcloud2(LidarDataQueue *queue, uint32_t packet_num,
                                      uint8_t handle)
    {
        uint64_t timestamp = 0;
        uint64_t last_timestamp = 0;
        uint32_t published_packet = 0;

        StoragePacket storage_packet;
        LidarDevice *lidar = &lds_->lidars_[handle];
        if (GetPublishStartTime(lidar, queue, &last_timestamp, &storage_packet))
        {
            /* the remaning packets in queue maybe not enough after skip */
            return 0;
        }

        sensor_msgs::PointCloud2 cloud;
        InitPointcloud2MsgHeader(cloud);
        cloud.data.resize(packet_num * kMaxPointPerEthPacket *
                          sizeof(LivoxPointXyzrtl));
        cloud.point_step = sizeof(LivoxPointXyzrtl);

        uint8_t *point_base = cloud.data.data();
        uint8_t data_source = lidar->data_src;
        uint32_t line_num = GetLaserLineNumber(lidar->info.type);
        uint32_t echo_num = GetEchoNumPerPoint(lidar->raw_data_type);
        uint32_t is_zero_packet = 0;
        while ((published_packet < packet_num) && !QueueIsEmpty(queue))
        {
            QueuePrePop(queue, &storage_packet);
            LivoxEthPacket *raw_packet =
                reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
            timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
            int64_t packet_gap = timestamp - last_timestamp;
            if ((packet_gap > lidar->packet_interval_max) &&
                lidar->data_is_published)
            {
                // ROS_INFO("Lidar[%d] packet time interval is %ldns", handle,
                //     packet_gap);
                if (kSourceLvxFile != data_source)
                {
                    timestamp = last_timestamp + lidar->packet_interval;
                    ZeroPointDataOfStoragePacket(&storage_packet);
                    is_zero_packet = 1;
                }
            }
            if (!published_packet)
            {
                cloud.header.stamp = ros::Time::now();
            }
            uint32_t single_point_num = storage_packet.point_num * echo_num;

            if (kSourceLvxFile != data_source)
            {
                PointConvertHandler pf_point_convert =
                    GetConvertHandler(lidar->raw_data_type);
                if (pf_point_convert)
                {
                    point_base = pf_point_convert(point_base, raw_packet,
                                                  lidar->extrinsic_parameter, line_num);
                }
                else
                {
                    /** Skip the packet */
                    ROS_INFO("Lidar[%d] unkown packet type[%d]", handle,
                             raw_packet->data_type);
                    break;
                }
            }
            else
            {
                point_base = LivoxPointToPxyzrtl(point_base, raw_packet,
                                                 lidar->extrinsic_parameter, line_num);
            }

            if (!is_zero_packet)
            {
                QueuePopUpdate(queue);
            }
            else
            {
                is_zero_packet = 0;
            }
            cloud.width += single_point_num;
            ++published_packet;
            last_timestamp = timestamp;
        }
        cloud.row_step = cloud.width * cloud.point_step;
        cloud.is_bigendian = false;
        cloud.is_dense = true;
        cloud.data.resize(cloud.row_step); /** Adjust to the real size */


        auto publisher = Lddc::GetCurrentPublisher(handle);
        if (kOutputToRos == config_.output_type)
        {
            // UFR change- define ptr to pointcloud msg
            const sensor_msgs::PointCloud2Ptr msg_cloudPtr = boost::make_shared<sensor_msgs::PointCloud2>(cloud);
            publisher.publish(msg_cloudPtr); // UFR change
        }
        else
        {
            if (bag_ && config_.lidar_bag)
            {
                bag_->write(publisher.getTopic(), ros::Time::now(),
                            cloud);
            }
        }
        if (!lidar->data_is_published)
        {
            lidar->data_is_published = true;
        }
        return published_packet;
    }

    void Lddc::FillPointsToPclMsg(boost::shared_ptr<PointCloud> &pcl_msg,
                                  LivoxPointXyzrtl *src_point, uint32_t num)
    {
        LivoxPointXyzrtl *point_xyzrtl = (LivoxPointXyzrtl *)src_point;
        for (uint32_t i = 0; i < num; i++)
        {
            pcl::PointXYZI point;
            point.x = point_xyzrtl->x;
            point.y = point_xyzrtl->y;
            point.z = point_xyzrtl->z;
            point.intensity = point_xyzrtl->reflectivity;
            ++point_xyzrtl;
            pcl_msg->points.push_back(point);
        }
    }

    /* for pcl::pxyzi */
    uint32_t Lddc::PublishPointcloudData(LidarDataQueue *queue, uint32_t packet_num,
                                         uint8_t handle)
    {
        uint64_t timestamp = 0;
        uint64_t last_timestamp = 0;
        uint32_t published_packet = 0;

        StoragePacket storage_packet;
        LidarDevice *lidar = &lds_->lidars_[handle];
        if (GetPublishStartTime(lidar, queue, &last_timestamp, &storage_packet))
        {
            /* the remaning packets in queue maybe not enough after skip */
            return 0;
        }

        boost::shared_ptr<PointCloud> cloud(new PointCloud);
        cloud->header.frame_id.assign(config_.lidar_frame_id);
        // Converting ROS time (ns) to PCL time (us) -- Check pcl_conversions API for reference
        cloud->header.stamp = ros::Time::now().toNSec() / 1000ull;
        cloud->height = 1;
        cloud->width = 0;

        uint8_t point_buf[2048];
        uint32_t is_zero_packet = 0;
        uint8_t data_source = lidar->data_src;
        uint32_t line_num = GetLaserLineNumber(lidar->info.type);
        uint32_t echo_num = GetEchoNumPerPoint(lidar->raw_data_type);
        while ((published_packet < packet_num) && !QueueIsEmpty(queue))
        {
            QueuePrePop(queue, &storage_packet);
            LivoxEthPacket *raw_packet =
                reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
            timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
            int64_t packet_gap = timestamp - last_timestamp;
            if ((packet_gap > lidar->packet_interval_max) &&
                lidar->data_is_published)
            {
                // ROS_INFO("Lidar[%d] packet time interval is %ldns", handle, packet_gap);
                if (kSourceLvxFile != data_source)
                {
                    timestamp = last_timestamp + lidar->packet_interval;
                    ZeroPointDataOfStoragePacket(&storage_packet);
                    is_zero_packet = 1;
                }
            }
            if (!published_packet)
            {
                cloud->header.stamp = timestamp / 1000.0; // to pcl ros time stamp
            }
            uint32_t single_point_num = storage_packet.point_num * echo_num;

            if (kSourceLvxFile != data_source)
            {
                PointConvertHandler pf_point_convert =
                    GetConvertHandler(lidar->raw_data_type);
                if (pf_point_convert)
                {
                    pf_point_convert(point_buf, raw_packet, lidar->extrinsic_parameter,
                                     line_num);
                }
                else
                {
                    /* Skip the packet */
                    ROS_INFO("Lidar[%d] unkown packet type[%d]", handle,
                             raw_packet->data_type);
                    break;
                }
            }
            else
            {
                LivoxPointToPxyzrtl(point_buf, raw_packet, lidar->extrinsic_parameter,
                                    line_num);
            }
            LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
            FillPointsToPclMsg(cloud, dst_point, single_point_num);
            if (!is_zero_packet)
            {
                QueuePopUpdate(queue);
            }
            else
            {
                is_zero_packet = 0;
            }
            cloud->width += single_point_num;
            ++published_packet;
            last_timestamp = timestamp;
        }

        auto publisher = Lddc::GetCurrentPublisher(handle);
        if (kOutputToRos == config_.output_type)
        {
            publisher.publish(cloud);
        }
        else
        {
            if (bag_ && config_.lidar_bag)
            {
                bag_->write(publisher.getTopic(), ros::Time::now(),
                            cloud);
            }
        }
        if (!lidar->data_is_published)
        {
            lidar->data_is_published = true;
        }
        return published_packet;
    }

    void Lddc::FillPointsToCustomMsg(livox_ros_driver::CustomMsg &livox_msg,
                                     LivoxPointXyzrtl *src_point, uint32_t num, uint32_t offset_time,
                                     uint32_t point_interval, uint32_t echo_num)
    {
        LivoxPointXyzrtl *point_xyzrtl = (LivoxPointXyzrtl *)src_point;
        for (uint32_t i = 0; i < num; i++)
        {
            livox_ros_driver::CustomPoint point;
            if (echo_num > 1)
            { /** dual return mode */
                point.offset_time = offset_time + (i / echo_num) * point_interval;
            }
            else
            {
                point.offset_time = offset_time + i * point_interval;
            }
            point.x = point_xyzrtl->x;
            point.y = point_xyzrtl->y;
            point.z = point_xyzrtl->z;
            point.reflectivity = point_xyzrtl->reflectivity;
            point.tag = point_xyzrtl->tag;
            point.line = point_xyzrtl->line;
            ++point_xyzrtl;
            livox_msg.points.push_back(point);
        }
    }

    uint32_t Lddc::PublishCustomPointcloud(LidarDataQueue *queue,
                                           uint32_t packet_num, uint8_t handle)
    {
        static uint32_t msg_seq = 0;
        uint64_t timestamp = 0;
        uint64_t last_timestamp = 0;

        StoragePacket storage_packet;
        LidarDevice *lidar = &lds_->lidars_[handle];
        if (GetPublishStartTime(lidar, queue, &last_timestamp, &storage_packet))
        {
            /* the remaning packets in queue maybe not enough after skip */
            return 0;
        }

        livox_ros_driver::CustomMsg livox_msg;
        livox_msg.header.frame_id.assign(config_.lidar_frame_id);
        livox_msg.header.stamp = ros::Time::now();
        livox_msg.header.seq = msg_seq;
        ++msg_seq;
        livox_msg.timebase = 0;
        livox_msg.point_num = 0;
        livox_msg.lidar_id = handle;

        uint8_t point_buf[2048];
        uint8_t data_source = lds_->lidars_[handle].data_src;
        uint32_t line_num = GetLaserLineNumber(lidar->info.type);
        uint32_t echo_num = GetEchoNumPerPoint(lidar->raw_data_type);
        uint32_t point_interval = GetPointInterval(lidar->info.type);
        uint32_t published_packet = 0;
        uint32_t packet_offset_time = 0; /** uint:ns */
        uint32_t is_zero_packet = 0;
        while (published_packet < packet_num)
        {
            QueuePrePop(queue, &storage_packet);
            LivoxEthPacket *raw_packet =
                reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
            timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
            int64_t packet_gap = timestamp - last_timestamp;
            if ((packet_gap > lidar->packet_interval_max) &&
                lidar->data_is_published)
            {
                // ROS_INFO("Lidar[%d] packet time interval is %ldns", handle,
                // packet_gap);
                if (kSourceLvxFile != data_source)
                {
                    timestamp = last_timestamp + lidar->packet_interval;
                    ZeroPointDataOfStoragePacket(&storage_packet);
                    is_zero_packet = 1;
                }
            }
            /** first packet */
            const bool is_first_packet = (published_packet == 0);
            if (is_first_packet)
            {
                livox_msg.timebase = timestamp;
                packet_offset_time = 0;
                /** convert to ros time stamp */
                livox_msg.header.stamp = ros::Time::now();
            }
            else
            {
                packet_offset_time = (uint32_t)(timestamp - livox_msg.timebase);
            }
            uint32_t single_point_num = storage_packet.point_num * echo_num;

            if (kSourceLvxFile != data_source)
            {
                PointConvertHandler pf_point_convert =
                    GetConvertHandler(lidar->raw_data_type);
                if (pf_point_convert)
                {
                    pf_point_convert(
                        point_buf,
                        raw_packet,
                        lidar->extrinsic_parameter,
                        line_num);
                }
                else
                {
                    /* Skip the packet */
                    ROS_INFO("Lidar[%d] unkown packet type[%d]", handle,
                             lidar->raw_data_type);
                    break;
                }
            }
            else
            {
                LivoxPointToPxyzrtl(
                    point_buf,
                    raw_packet,
                    lidar->extrinsic_parameter,
                    line_num);
            }
            LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
            FillPointsToCustomMsg(
                livox_msg,
                dst_point,
                single_point_num,
                packet_offset_time,
                point_interval,
                echo_num);

            if (!is_zero_packet)
            {
                QueuePopUpdate(queue);
            }
            else
            {
                is_zero_packet = 0;
            }

            livox_msg.point_num += single_point_num;
            last_timestamp = timestamp;
            ++published_packet;
        }

        auto publisher = Lddc::GetCurrentPublisher(handle);
        if (kOutputToRos == config_.output_type)
        {
            publisher.publish(livox_msg);
        }
        else
        {
            if (bag_ && config_.lidar_bag)
            {
                bag_->write(
                    publisher.getTopic(),
                    ros::Time::now(),
                    livox_msg);
            }
        }

        if (!lidar->data_is_published)
        {
            lidar->data_is_published = true;
        }
        return published_packet;
    }

    uint32_t Lddc::PublishImuData(LidarDataQueue *queue, uint32_t packet_num,
                                  uint8_t handle)
    {
        uint32_t published_packet = 0;

        StoragePacket storage_packet;
        QueuePrePop(queue, &storage_packet);
        LivoxEthPacket *raw_packet =
            reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
        imu_data_.header.stamp = ros::Time::now(); // to ros time stamp

        uint8_t point_buf[2048];
        LivoxImuDataProcess(point_buf, raw_packet);

        LivoxImuPoint *imu = (LivoxImuPoint *)point_buf;
        imu_data_.angular_velocity.x = imu->gyro_x;
        imu_data_.angular_velocity.y = imu->gyro_y;
        imu_data_.angular_velocity.z = imu->gyro_z;
        imu_data_.linear_acceleration.x = imu->acc_x * kGravity_;
        imu_data_.linear_acceleration.y = imu->acc_y * kGravity_;
        imu_data_.linear_acceleration.z = imu->acc_z * kGravity_;

        QueuePopUpdate(queue);
        ++published_packet;

        auto publisher = Lddc::GetCurrentImuPublisher(handle);
        if (kOutputToRos == config_.output_type)
        {
            publisher.publish(imu_data_);
        }
        else
        {
            if (bag_ && config_.imu_bag)
            {
                bag_->write(
                    publisher.getTopic(),
                    ros::Time::now(),
                    imu_data_);
            }
        }
        return published_packet;
    }

    int Lddc::RegisterLds(Lds *lds)
    {
        if (lds_ == nullptr)
        {
            lds_ = lds;
            return 0;
        }
        else
        {
            return -1;
        }
    }

    void Lddc::PollingLidarPointCloudData(uint8_t handle, LidarDevice *lidar)
    {
        LidarDataQueue *p_queue = &lidar->data;
        if (p_queue->storage_packet == nullptr)
        {
            ROS_ERROR("Lidar storage package is null");
            return;
        }

        while (!QueueIsEmpty(p_queue))
        {
            uint32_t used_size = QueueUsedSize(p_queue);
            uint32_t onetime_publish_packets = lidar->onetime_publish_packets;
            if (used_size < onetime_publish_packets)
            {
                break;
            }

            switch (static_cast<TransferType>(config_.format))
            {
                case TransferType::kPointCloud2Msg:
                    PublishPointcloud2(p_queue, onetime_publish_packets, handle);
                    break;
                case TransferType::kLivoxCustomMsg:
                    PublishCustomPointcloud(p_queue, onetime_publish_packets, handle);
                    break;
                case TransferType::kPclPxyziMsg:
                    PublishPointcloudData(p_queue, onetime_publish_packets, handle);
                    break;
                default:
                    break;
            }
        }
    }

    void Lddc::PollingLidarImuData(uint8_t handle, LidarDevice *lidar)
    {
        LidarDataQueue *p_queue = &lidar->imu_data;
        if (p_queue->storage_packet == nullptr)
        {
            return;
        }

        while (!QueueIsEmpty(p_queue))
        {
            PublishImuData(p_queue, 1, handle);
        }
    }

    void Lddc::DistributeLidarData(void)
    {
        if (lds_ == nullptr)
        {
            ROS_ERROR("LDS is null");
            return;
        }
        lds_->semaphore_.Wait();
        for (uint32_t i = 0; i < lds_->lidar_count_; i++)
        {
            uint32_t lidar_id = i;
            LidarDevice *lidar = &lds_->lidars_[lidar_id];
            LidarDataQueue *p_queue = &lidar->data;
            if ((kConnectStateSampling != lidar->connect_state) ||
                (p_queue == nullptr))
            {
                continue;
            }
            PollingLidarPointCloudData(lidar_id, lidar);
            PollingLidarImuData(lidar_id, lidar);
        }

        if (lds_->IsRequestExit())
        {
            PrepareExit();
        }
    }


    void Lddc::ConfigureLidarPublishers()
    {
        if (config_.multi_topic)
        {
            const uint32_t queue_size = kMinEthPacketQueueSize * 2 ; // queue size is 64 for only one lidar
            ROS_INFO("Support multi topics.");
            for (size_t handle = 0; handle < kMaxSourceLidar; handle++)
            {
                const std::string topic_name = fmt::format("livox/imu_%s", lds_->lidars_[handle].info.broadcast_code);
                switch (static_cast<TransferType>(config_.format))
                {
                    case TransferType::kPointCloud2Msg:
                    {
                        private_pub_[handle] = cur_node_->advertise<sensor_msgs::PointCloud2>(topic_name, queue_size);
                        ROS_INFO(
                            "%s publish use PointCloud2 format, set ROS publisher queue size %d",
                            topic_name.c_str(), queue_size);
                        break;
                    }
                    case TransferType::kLivoxCustomMsg:
                    {
                        private_pub_[handle] = cur_node_->advertise<livox_ros_driver::CustomMsg>(topic_name,
                                                                                queue_size);
                        ROS_INFO(
                            "%s publish use livox custom format, set ROS publisher queue size %d",
                            topic_name.c_str(), queue_size);
                        break;
                    }
                    case TransferType::kPclPxyziMsg:
                    {
                        private_pub_[handle] = cur_node_->advertise<PointCloud>(topic_name, queue_size);
                        ROS_INFO(
                            "%s publish use pcl PointXYZI format, set ROS publisher queue "
                            "size %d",
                            topic_name.c_str(), queue_size);
                        break;
                    }
                    default:
                        std::runtime_error("Invalid transfer format for Livox Lidar");
                        break;
                }
            }
        }
        else
        {
            ROS_INFO("Support only one topic.");
            const std::string topic_name = "livox/lidar";
            const uint32_t queue_size = kMinEthPacketQueueSize * 8; // shared queue size is 256, for all lidars
            switch (static_cast<TransferType>(config_.format))
            {
                case TransferType::kPointCloud2Msg:
                {
                    global_pub_ = cur_node_->advertise<sensor_msgs::PointCloud2>(topic_name, queue_size);
                    ROS_INFO(
                        "%s publish use PointCloud2 format, set ROS publisher queue size %d",
                        topic_name.c_str(), queue_size);
                    break;
                }
                case TransferType::kLivoxCustomMsg:
                {
                    global_pub_ = cur_node_->advertise<livox_ros_driver::CustomMsg>(topic_name,
                                                                            queue_size);
                    ROS_INFO(
                        "%s publish use livox custom format, set ROS publisher queue size %d",
                        topic_name.c_str(), queue_size);
                    break;
                }
                case TransferType::kPclPxyziMsg:
                {
                    global_pub_ = cur_node_->advertise<PointCloud>(topic_name, queue_size);
                    ROS_INFO(
                        "%s publish use pcl PointXYZI format, set ROS publisher queue "
                        "size %d",
                        topic_name.c_str(), queue_size);
                    break;
                }
                default:
                    std::runtime_error("Invalid transfer format for Livox Lidar");
                    break;
            }
        }
    }

    void Lddc::ConfigureImuPublishers()
    {
        if (config_.multi_topic)
        {
            ROS_INFO("Support multi topics.");
            const uint32_t queue_size = kMinEthPacketQueueSize * 2 ; // queue size is 64 for only one lidar
            for (size_t handle = 0; handle < kMaxSourceLidar; handle++)
            {
                const std::string topic_name = fmt::format("/livox/imu_%s", lds_->lidars_[handle].info.broadcast_code);
                private_imu_pub_[handle] = cur_node_->advertise<sensor_msgs::Imu>(topic_name, queue_size);
            }
        }
        else
        {
            ROS_INFO("Support only one topic.");
            const std::string topic_name = "/livox/imu";
            const uint32_t queue_size = kMinEthPacketQueueSize * 8; // shared queue size is 256, for all lidars
            global_imu_pub_ = cur_node_->advertise<sensor_msgs::Imu>(topic_name, queue_size);
            ROS_INFO("%s publish imu data, set ROS publisher queue size %d", topic_name.c_str(),
                queue_size);
        }
    }

    ros::Publisher& Lddc::GetCurrentPublisher(uint8_t handle)
    {
        return config_.multi_topic ? private_pub_[handle] : global_pub_;
    }

    ros::Publisher& Lddc::GetCurrentImuPublisher(uint8_t handle)
    {
        return config_.multi_topic ? private_imu_pub_[handle] : global_imu_pub_;
    }

    void Lddc::CreateBagFile(const std::string &file_name)
    {
        if (!bag_)
        {
            bag_ = new rosbag::Bag;
            bag_->open(file_name, rosbag::bagmode::Write);
            ROS_INFO("Create bag file :%s!", file_name.c_str());
        }
    }

    void Lddc::PrepareExit(void)
    {
        if (bag_)
        {
            ROS_INFO("Waiting to save the bag file!");
            bag_->close();
            ROS_INFO("Save the bag file successfully!");
            bag_ = nullptr;
        }
        if (lds_)
        {
            lds_->PrepareExit();
            lds_ = nullptr;
        }
    }

} // namespace livox_ros
