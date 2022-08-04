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

#include "include/livox_ros_driver_nodelet.h"

#include <algorithm>
#include <chrono>
#include <vector>

#include "lddc.h"
#include "lds_hub.h"
#include "lds_lidar.h"
#include "lds_lvx.h"
#include "livox_sdk.h"
#include <ros/ros.h>

namespace livox_ros
{

    const int32_t kSdkVersionMajorLimit = 2;

    bool InitializeRawLidar(std::unique_ptr<Lddc> &lddc, ros::NodeHandle &pnh, const double publish_freq)
    {
        ROS_INFO("Data Source is raw lidar.");

        std::string user_config_path;
        pnh.getParam("user_config_path", user_config_path);
        ROS_INFO("Config file : %s", user_config_path.c_str());

        std::string cmdline_bd_code;
        pnh.getParam("cmdline_str", cmdline_bd_code);

        std::vector<std::string> bd_code_list;
        ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);

        // Get the Lidar data source.
        LdsLidar *read_lidar = LdsLidar::GetInstance(1000 / publish_freq);

        // Register the Lidar data source with the lidar data distributor.
        constexpr int SUCCESS = 0;
        const int registered_lidar_data_source = lddc->RegisterLds(static_cast<Lds *>(read_lidar));
        if (SUCCESS != registered_lidar_data_source)
        {
            ROS_ERROR("Register lidar data distributor failure.");
            return false;
        }

        // Initialize the lidar.
        const int lidar_initialization = read_lidar->InitLdsLidar(bd_code_list, user_config_path.c_str());
        if (SUCCESS != lidar_initialization)
        {
            ROS_ERROR("Initializing Lidar Data source failure. Error code: %i", lidar_initialization);
            return false;
        }

        ROS_INFO("Init lds lidar success!");
        return true;
    }

    bool InitializeRawHub(std::unique_ptr<Lddc> &lddc, ros::NodeHandle &pnh, const double publish_freq)
    {
        ROS_INFO("Data Source is hub.");

        std::string user_config_path;
        pnh.getParam("user_config_path", user_config_path);
        ROS_INFO("Config file : %s", user_config_path.c_str());

        std::string cmdline_bd_code;
        pnh.getParam("cmdline_str", cmdline_bd_code);

        std::vector<std::string> bd_code_list;
        ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);

        LdsHub *read_hub = LdsHub::GetInstance(1000 / publish_freq);
        lddc->RegisterLds(static_cast<Lds *>(read_hub));
        const int lidar_hub_initialization = read_hub->InitLdsHub(bd_code_list, user_config_path.c_str());
        constexpr int SUCCESS = 0;
        if (SUCCESS != lidar_hub_initialization)
        {
            ROS_ERROR("Init lds hub fail!");
            return false;
        }

        ROS_INFO("Init lds hub success!");
        return true;
    }

    bool InitializeLvxFile(std::unique_ptr<Lddc> &lddc, ros::NodeHandle &pnh, const double publish_freq)
    {
        ROS_INFO("Data Source is lvx file.");

        std::string cmdline_file_path;
        pnh.getParam("cmdline_file_path", cmdline_file_path);

        if (!IsFilePathValid(cmdline_file_path.c_str()))
        {
            ROS_ERROR("File path invalid : %s !", cmdline_file_path.c_str());
            return false;
        }

        std::string rosbag_file_path;
        int path_end_pos = cmdline_file_path.find_last_of('.');
        rosbag_file_path = cmdline_file_path.substr(0, path_end_pos);
        rosbag_file_path += ".bag";

        LdsLvx *read_lvx = LdsLvx::GetInstance(1000 / publish_freq);
        lddc->RegisterLds(static_cast<Lds *>(read_lvx));
        lddc->CreateBagFile(rosbag_file_path);
        const int livox_file_initialization = read_lvx->InitLdsLvx(cmdline_file_path.c_str());
        constexpr int SUCCESS = 0;
        if (SUCCESS != livox_file_initialization)
        {
            ROS_ERROR("Init lds lvx file fail!");
            return false;
        }

        ROS_INFO("Init lds lvx file success!");
        return true;
    }

    void LivoxRosDriverNodelet::proccessLidarLoop(const ros::TimerEvent &)
    {
        lddc_->DistributeLidarData();
    }

    void LivoxRosDriverNodelet::onInit(void)
    {
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
        if (_sdkversion.major < kSdkVersionMajorLimit)
        {
            ROS_INFO("The SDK version[%d.%d.%d] is too low", _sdkversion.major,
                     _sdkversion.minor, _sdkversion.patch);
            return;
        }

        /** Init default system parameter */
        Lddc_config ldcc_config;

        ldcc_config.format = kPointCloud2Msg;
        ldcc_config.multi_topic = 0;
        ldcc_config.data_src = kSourceRawLidar;
        ldcc_config.frequency = 10.0; /* Hz */
        ldcc_config.output_type = kOutputToRos;
        ldcc_config.lidar_frame_id = "livox_frame";
        ldcc_config.imu_frame_id = "livox_frame";
        ldcc_config.lidar_bag = true;
        ldcc_config.imu_bag = false;

        pnh.getParam("xfer_format", ldcc_config.format);
        pnh.getParam("multi_topic", ldcc_config.multi_topic);
        pnh.getParam("data_src", ldcc_config.data_src);
        pnh.getParam("publish_freq", ldcc_config.frequency);
        pnh.getParam("output_data_type", ldcc_config.output_type);
        pnh.getParam("lidar_frame_id", ldcc_config.lidar_frame_id);
        pnh.getParam("imu_frame_id", ldcc_config.imu_frame_id);
        pnh.getParam("enable_lidar_bag", ldcc_config.lidar_bag);
        pnh.getParam("enable_imu_bag", ldcc_config.imu_bag);

        // Clamp publish_freq between 1.0 and 100.0 Hz
        ldcc_config.frequency = std::clamp(ldcc_config.frequency, 1.0, 100.0);

        /** Lidar data distribute control and lidar data source set */
        std::unique_ptr<Lddc> lddc = std::make_unique<Lddc>(ldcc_config);
        lddc->SetRosNodeHandlers(&nh, &pnh);
        lddc->SetImuCovariances();

        bool lidar_data_source_initialized = false;
        switch (static_cast<LidarDataSourceType>(ldcc_config.data_src))
        {
        case LidarDataSourceType::kSourceRawLidar:
            lidar_data_source_initialized = InitializeRawLidar(lddc, pnh, ldcc_config.frequency);
            break;
        case LidarDataSourceType::kSourceRawHub:
            lidar_data_source_initialized = InitializeRawHub(lddc, pnh, ldcc_config.frequency);
            break;
        case LidarDataSourceType::kSourceLvxFile:
            lidar_data_source_initialized = InitializeLvxFile(lddc, pnh, ldcc_config.frequency);
            break;
        default:
            ROS_ERROR("Unsupported data source: %i", ldcc_config.data_src);
        }

        double poll_freq = ldcc_config.frequency * 4;
        if (ldcc_config.data_src == kSourceLvxFile)
        {
            poll_freq = 2000;
        }

        poll_lidars_ = pnh.createTimer(ros::Duration(ros::Rate(poll_freq)), &LivoxRosDriverNodelet::proccessLidarLoop, this);
    }


} // namespace

// Pluginlib include.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(livox_ros::LivoxRosDriverNodelet, nodelet::Nodelet)
