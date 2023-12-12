#pragma once

#include <cstdint>

namespace livox_ros
{
    enum class ReturnMode : uint32_t
    {
        SINGLE = 0,
        STRONGEST = 1,
        DUAL = 2,
        TRIPLE = 3,
    };

    enum class CoordinateSystem : uint32_t
    {
        CARTESIAN = 0,
        SPHERICAL = 1
    };

    enum class ImuRate : uint32_t
    {
        OFF = 0,
        ON = 1 // 200Hz.
    };

    enum class ExtrinsicParameterCompensation : uint32_t
    {
        OFF = 0,
        ON = 1
    };

    struct LidarConfig
    {
        bool enable_fan;
        ReturnMode return_mode;
        CoordinateSystem coordinate_system;
        ImuRate imu_rate;
        ExtrinsicParameterCompensation extrinsic_parameter_compensation;
        bool enable_high_sensitivity;
    };
} //namespace livox_ros
