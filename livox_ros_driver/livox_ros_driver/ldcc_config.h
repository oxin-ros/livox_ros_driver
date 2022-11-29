#include <string>

namespace livox_ros
{
    struct Lddc_config
    {
        int format;
        int multi_topic;
        int data_src;
        int output_type;
        double frequency;
        std::string lidar_frame_id;
        std::string imu_frame_id;
        bool lidar_bag;
        bool imu_bag;
    };
}
