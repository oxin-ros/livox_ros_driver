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
#pragma once

#include <memory>
#include <optional>
#include <thread>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "ldcc_config.h"
#include "lddc.h"
#include "livox_ros_driver_version.h"

namespace livox_ros
{

class LivoxRosDriverNodelet : public nodelet::Nodelet
{
    public:
        LivoxRosDriverNodelet() = default;
        ~LivoxRosDriverNodelet() = default;

        void onInit(void);

    private:
        bool InitializeRawLidar(
            ros::NodeHandle &pnh, const double publish_frequency);
        bool InitializeRawHub(
            ros::NodeHandle &pnh, const double publish_frequency);
        bool InitializeLvxFile(
            ros::NodeHandle &pnh, const double publish_frequency);

        void proccessLidarLoop(const ros::TimerEvent&);
        std::optional<UserRawConfig> GetLidarConfig(ros::NodeHandle& nh);
        std::optional<TimeSyncRawConfig> GetTimesyncConfig(ros::NodeHandle& nh);

        std::unique_ptr<livox_ros::Lddc> lddc_;
        ros::Timer poll_lidars_;

        static constexpr int32_t MIN_SUPPORTED_SDK_VERSION = 2;
};

} // namespace livox_ros
