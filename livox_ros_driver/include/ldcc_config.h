#pragma once

#include <string>

namespace livox_ros
{
    struct LdccConfig
    {
        int format;
        int multi_topic;
        int data_src;
        double frequency;
        std::string lidar_frame_id;
        std::string imu_frame_id;
    };
}
