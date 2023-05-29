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

  void LivoxRosDriverNodelet::onInit(void)
  {
    ros::NodeHandle node_handle = getNodeHandle();
    ros::NodeHandle private_node_handle = getPrivateNodeHandle();

    NODELET_INFO("Livox Ros Driver Version: %s", LIVOX_ROS_DRIVER_VERSION_STRING);

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
      NODELET_INFO("The SDK version[%d.%d.%d] is too low", _sdkversion.major,
                   _sdkversion.minor, _sdkversion.patch);
      return;
    }

    /** Init defualt system parameter */
    int xfer_format = kPointCloud2Msg;
    int multi_topic = 0;
    int data_src = kSourceRawLidar;
    double publish_freq = 10.0; /* Hz */
    int output_type = kOutputToRos;
    std::string lidar_frame_id = "livox_frame";
    std::string imu_frame_id = "livox_frame";
    bool lidar_bag = true;
    bool imu_bag = false;

    private_node_handle.getParam("xfer_format", xfer_format);
    private_node_handle.getParam("multi_topic", multi_topic);
    private_node_handle.getParam("data_src", data_src);
    private_node_handle.getParam("publish_freq", publish_freq);
    private_node_handle.getParam("output_data_type", output_type);
    private_node_handle.getParam("lidar_frame_id", lidar_frame_id);
    private_node_handle.getParam("imu_frame_id", imu_frame_id);
    private_node_handle.getParam("enable_lidar_bag", lidar_bag);
    private_node_handle.getParam("enable_imu_bag", imu_bag);

    // Clamp publish_freq between 1.0 and 100.0 Hz
    publish_freq = std::clamp(publish_freq, 1.0, 100.0);

    /** Lidar data distribute control and lidar data source set */
    lddc_ = new Lddc(
        xfer_format,
        multi_topic,
        data_src,
        output_type,
        publish_freq,
        lidar_frame_id,
        imu_frame_id,
        lidar_bag,
        imu_bag);
    lddc_->SetRosNodeHandlers(node_handle, private_node_handle);
    lddc_->SetImuCovariances();

    int ret = 0;
    if (data_src == kSourceRawLidar)
    {
      NODELET_INFO("Data Source is raw lidar.");

      std::string user_config_path;
      private_node_handle.getParam("user_config_path", user_config_path);
      NODELET_INFO("Config file : %s", user_config_path.c_str());

      std::string cmdline_bd_code;
      private_node_handle.getParam("cmdline_str", cmdline_bd_code);

      std::vector<std::string> bd_code_list;
      ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);

      LdsLidar *read_lidar = LdsLidar::GetInstance(1000 / publish_freq);
      lddc_->RegisterLds(static_cast<Lds *>(read_lidar));
      ret = read_lidar->InitLdsLidar(bd_code_list, user_config_path.c_str());
      if (!ret)
      {
        NODELET_INFO("Init lds lidar success!");
      }
      else
      {
        NODELET_ERROR("Init lds lidar fail!");
      }
    }
    else if (data_src == kSourceRawHub)
    {
      NODELET_INFO("Data Source is hub.");

      std::string user_config_path;
      private_node_handle.getParam("user_config_path", user_config_path);
      NODELET_INFO("Config file : %s", user_config_path.c_str());

      std::string cmdline_bd_code;
      private_node_handle.getParam("cmdline_str", cmdline_bd_code);

      std::vector<std::string> bd_code_list;
      ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);

      LdsHub *read_hub = LdsHub::GetInstance(1000 / publish_freq);
      lddc_->RegisterLds(static_cast<Lds *>(read_hub));
      ret = read_hub->InitLdsHub(bd_code_list, user_config_path.c_str());
      if (!ret)
      {
        NODELET_INFO("Init lds hub success!");
      }
      else
      {
        NODELET_ERROR("Init lds hub fail!");
      }
    }
    else
    {
      NODELET_INFO("Data Source is lvx file.");

      std::string cmdline_file_path;
      private_node_handle.getParam("cmdline_file_path", cmdline_file_path);

      do
      {
        if (!IsFilePathValid(cmdline_file_path.c_str()))
        {
          NODELET_ERROR("File path invalid : %s !", cmdline_file_path.c_str());
          break;
        }

        std::string rosbag_file_path;
        int path_end_pos = cmdline_file_path.find_last_of('.');
        rosbag_file_path = cmdline_file_path.substr(0, path_end_pos);
        rosbag_file_path += ".bag";

        LdsLvx *read_lvx = LdsLvx::GetInstance(1000 / publish_freq);
        lddc_->RegisterLds(static_cast<Lds *>(read_lvx));
        lddc_->CreateBagFile(rosbag_file_path);
        int ret = read_lvx->InitLdsLvx(cmdline_file_path.c_str());
        if (!ret)
        {
          NODELET_INFO("Init lds lvx file success!");
        }
        else
        {
          NODELET_ERROR("Init lds lvx file fail!");
        }
      } while (0);
    }

    double poll_freq = publish_freq * 4;
    if (data_src == kSourceLvxFile)
    {
      poll_freq = 2000;
    }

    poll_lidars_ = private_node_handle.createTimer(ros::Duration(ros::Rate(poll_freq)), &LivoxRosDriverNodelet::proccessLidarLoop, this);
  }

  void LivoxRosDriverNodelet::proccessLidarLoop(const ros::TimerEvent &)
  {
    try
    {
      lddc_->DistributeLidarData();
    }
    catch (const std::exception &e)
    {
      NODELET_ERROR_STREAM("Exception occurred while polling lidar: " << e.what());
    }
  }

} // namespace

// Pluginlib include.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(livox_ros::LivoxRosDriverNodelet, nodelet::Nodelet)
