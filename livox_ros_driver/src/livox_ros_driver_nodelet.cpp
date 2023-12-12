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

#include "livox_ros_driver_nodelet.h"

#include <csignal>
#include <algorithm>
#include <chrono>
#include <vector>

#include "lddc.h"
#include "lds_lidar.h"
#include "livox_sdk.h"
#include <ros/ros.h>

namespace livox_ros
{

    void LivoxRosDriverNodelet::onInit(void)
    {
        // std::this_thread::sleep_for(std::chrono::seconds(10));

        ros::NodeHandle nh = getNodeHandle();
        ros::NodeHandle pnh = getPrivateNodeHandle();

        ROS_INFO("Livox Ros Driver Version: %s", LIVOX_ROS_DRIVER_VERSION_STRING);

        /** Ros related */
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                           ros::console::levels::Debug))
        {
            ros::console::notifyLoggerLevelsChanged();
        }

        /** Check sdk version */
        LivoxSdkVersion _sdkversion;
        GetLivoxSdkVersion(&_sdkversion);
        if (_sdkversion.major < MIN_SUPPORTED_SDK_VERSION)
        {
            ROS_INFO("The SDK version[%d.%d.%d] is too low", _sdkversion.major,
                     _sdkversion.minor, _sdkversion.patch);
            return;
        }

        /** Init default system parameter */
        LdccConfig ldcc_config;

        ldcc_config.data_src = kSourceRawLidar;
        ldcc_config.format = kPointCloud2Msg;
        ldcc_config.frequency = 10.0; /* Hz */
        ldcc_config.imu_frame_id = "livox_frame";
        ldcc_config.lidar_frame_id = "livox_frame";
        ldcc_config.multi_topic = 0;

        pnh.getParam("data_src", ldcc_config.data_src);
        pnh.getParam("imu_frame_id", ldcc_config.imu_frame_id);
        pnh.getParam("lidar_frame_id", ldcc_config.lidar_frame_id);
        pnh.getParam("multi_topic", ldcc_config.multi_topic);
        pnh.getParam("publish_freq", ldcc_config.frequency);
        pnh.getParam("xfer_format", ldcc_config.format);

        // Clamp publish_freq between 1.0 and 100.0 Hz
        ldcc_config.frequency = std::clamp(ldcc_config.frequency, 1.0, 100.0);

        /** Lidar data distribute control and lidar data source set */
        lddc_ = std::make_unique<Lddc>(ldcc_config);
        lddc_->SetRosNodeHandlers(&nh, &pnh);
        lddc_->SetImuCovariances();

        bool lidar_data_source_initialized = false;
        switch (static_cast<LidarDataSourceType>(ldcc_config.data_src))
        {
        case LidarDataSourceType::kSourceRawLidar:
            lidar_data_source_initialized = InitializeRawLidar(pnh, ldcc_config.frequency);
            break;
        default:
            ROS_ERROR("Unsupported data source: %i", ldcc_config.data_src);
            throw std::runtime_error("Unsupported data source");
        }

        if (!lidar_data_source_initialized)
        {
            throw std::runtime_error("Failed to initialize data source");
        }

        const double poll_freq = ldcc_config.frequency * 4;
        poll_lidars_ = pnh.createTimer(ros::Duration(ros::Rate(poll_freq)), &LivoxRosDriverNodelet::proccessLidarLoop, this);
    }

    void LivoxRosDriverNodelet::proccessLidarLoop(const ros::TimerEvent &)
    {
        lddc_->DistributeLidarData();
    }

    bool LivoxRosDriverNodelet::InitializeRawLidar(ros::NodeHandle &pnh, const double publish_frequency)
    {
        ROS_INFO("Data Source is raw lidar.");

        // Get the Lidar data source.
        livox_ros::LdsLidar* p_read_lidar = LdsLidar::GetInstance(1000 / publish_frequency);

        // Register the Lidar data source with the lidar data distributor.
        constexpr int SUCCESS = 0;
        const int registered_lidar_data_source = lddc_->RegisterLds(static_cast<Lds *>(p_read_lidar));
        if (SUCCESS != registered_lidar_data_source)
        {
            ROS_ERROR("Register lidar data distributor failure.");
            return false;
        }

        // Get the lidar config from ROS.
        const auto lidar_config = GetLidarConfig(pnh);

        // Get the timesync config from ROS.
        const auto timesync_config = GetTimesyncConfig(pnh);

        // Initialize the lidar.
        const int lidar_initialization = p_read_lidar->InitLdsLidar(lidar_config, timesync_config);
        if (SUCCESS != lidar_initialization)
        {
            ROS_ERROR("Initializing Lidar Data source failure. Error code: %i", lidar_initialization);
            return false;
        }

        ROS_INFO("Init lds lidar success!");
        return true;
    }

    std::optional<UserRawConfig> LivoxRosDriverNodelet::GetLidarConfig(ros::NodeHandle& pnh)
    {
        // Check whether the lidar config is available.
        const bool lidar_config_available = pnh.hasParam("lidar_config");
        if (!lidar_config_available)
        {
            return std::nullopt;
        }

        // Check whether the broadcast code is specified.
        ros::NodeHandle config_nh(pnh, "lidar_config");

        // Pull the config from the ROS param server.
        UserRawConfig lidar_config;
        int return_mode = 0;
        int coordinate = 0;
        int imu_rate = 1;
        int extrinsic_parameter_source = 0;
        config_nh.getParam("enable_fan", lidar_config.enable_fan);
        config_nh.getParam("return_mode", return_mode);
        config_nh.getParam("coordinate", coordinate);
        config_nh.getParam("imu_rate", imu_rate);
        config_nh.getParam("extrinsic_parameter_source", extrinsic_parameter_source);
        config_nh.getParam("enable_high_sensitivity", lidar_config.enable_high_sensitivity);

        // Copy the ros parameters to the config.
        lidar_config.return_mode = return_mode;
        lidar_config.coordinate = coordinate;
        lidar_config.imu_rate = imu_rate;
        lidar_config.extrinsic_parameter_source = extrinsic_parameter_source;

        return lidar_config;
    }

    std::optional<TimeSyncRawConfig> LivoxRosDriverNodelet::GetTimesyncConfig(ros::NodeHandle& pnh)
    {
        // Check whether the timesync config is available.
        const bool timesync_config_available = pnh.hasParam("timesync_config");
        if (!timesync_config_available)
        {
            return std::nullopt;
        }

        // Pull the config from the ROS param server.
        TimeSyncRawConfig timesync_config;
        ros::NodeHandle config_nh(pnh, "timesync_config");
        config_nh.getParam("enable_timesync", timesync_config.enable_timesync);
        config_nh.getParam("device_name", timesync_config.device_name);
        config_nh.getParam("comm_device_type", timesync_config.comm_device_type);
        config_nh.getParam("baudrate_index", timesync_config.baudrate_index);
        config_nh.getParam("parity_index", timesync_config.parity_index);
        return timesync_config;
    }

} // namespace

// Pluginlib include.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(livox_ros::LivoxRosDriverNodelet, nodelet::Nodelet)
