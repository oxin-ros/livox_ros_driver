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

#include "include/livox_ros_driver.h"

#include <chrono>
#include <vector>
#include <csignal>
#include <optional>

#include <ros/ros.h>
#include "lddc.h"
#include "lds_hub.h"
#include "lds_lidar.h"
#include "lds_lvx.h"
#include "livox_sdk.h"

using namespace livox_ros;

const int32_t kSdkVersionMajorLimit = 2;

inline void SignalHandler(int signum)
{
    printf("livox ros driver will exit\r\n");
    ros::shutdown();
    exit(signum);
}

std::optional<UserRawConfig> GetLidarConfig(ros::NodeHandle& pnh)
{
    const bool lidar_config_available = pnh.hasParam("lidar_config");
    if (!lidar_config_available)
    {
        return std::nullopt;
    }

    UserRawConfig lidar_config;
    std::string broadcast_code;
    int return_mode = 0;
    int coordinate = 0;
    int imu_rate = 1;
    int extrinsic_parameter_source = 0;
    pnh.getParam("lidar_config/broadcast_code", broadcast_code);
    pnh.getParam("lidar_config/enable_connect", lidar_config.enable_connect);
    pnh.getParam("lidar_config/enable_fan", lidar_config.enable_fan);
    pnh.getParam("lidar_config/return_mode", return_mode);
    pnh.getParam("lidar_config/coordinate", coordinate);
    pnh.getParam("lidar_config/imu_rate", imu_rate);
    pnh.getParam("lidar_config/extrinsic_parameter_source", extrinsic_parameter_source);
    pnh.getParam("lidar_config/enable_high_sensitivity", lidar_config.enable_high_sensitivity);

    strncpy(lidar_config.broadcast_code, broadcast_code.c_str(), 16);
    lidar_config.return_mode = return_mode;
    lidar_config.coordinate = coordinate;
    lidar_config.imu_rate = imu_rate;
    lidar_config.extrinsic_parameter_source = extrinsic_parameter_source;

    return lidar_config;
}

std::optional<TimeSyncRawConfig> GetTimesyncConfig(ros::NodeHandle& pnh)
{
    const bool timesync_config_available = pnh.hasParam("timesync_config");
    if (!timesync_config_available)
    {
        return std::nullopt;
    }

    TimeSyncRawConfig timesync_config;
    pnh.getParam("timesync_config/enable_timesync", timesync_config.enable_timesync);
    pnh.getParam("timesync_config/device_name", timesync_config.device_name);
    pnh.getParam("timesync_config/comm_device_type", timesync_config.comm_device_type);
    pnh.getParam("timesync_config/baudrate_index", timesync_config.baudrate_index);
    pnh.getParam("timesync_config/parity_index", timesync_config.parity_index);
    return timesync_config;
}

bool InitializeRawLidar(std::unique_ptr<Lddc> &lddc, ros::NodeHandle &pnh, const double publish_freq)
{
    ROS_INFO("Data Source is raw lidar.");

    // Get the Lidar data source.
    LdsLidar* p_read_lidar = LdsLidar::GetInstance(1000 / publish_freq);

    // Register the Lidar data source with the lidar data distributor.
    constexpr int SUCCESS = 0;
    const int registered_lidar_data_source = lddc->RegisterLds(static_cast<Lds *>(p_read_lidar));
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

int main(int argc, char **argv)
{
    /** Ros related */
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::init(argc, argv, "livox_lidar_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ROS_INFO("Livox Ros Driver Version: %s", LIVOX_ROS_DRIVER_VERSION_STRING);
    signal(SIGINT, SignalHandler);
    /** Check sdk version */
    LivoxSdkVersion _sdkversion;
    GetLivoxSdkVersion(&_sdkversion);
    if (_sdkversion.major < kSdkVersionMajorLimit)
    {
        ROS_INFO("The SDK version[%d.%d.%d] is too low", _sdkversion.major,
                 _sdkversion.minor, _sdkversion.patch);
        return 0;
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

    if (!lidar_data_source_initialized)
    {
        ros::shutdown();
        return -1;
    }

    ros::Time::init();
    while (ros::ok())
    {
        lddc->DistributeLidarData();
    }

    return 0;
}
