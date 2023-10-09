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

#include "lds_lidar.h"

#include <stdio.h>
#include <string.h>
#include <memory>
#include <mutex>
#include <thread>

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/stringbuffer.h"

using namespace std;

namespace livox_ros {

/** Const varible ------------------------------------------------------------*/
/** For callback use only */
LdsLidar *g_lds_ldiar = nullptr;

/** Global function for common use -------------------------------------------*/

/** Lds lidar function -------------------------------------------------------*/
LdsLidar::LdsLidar(uint32_t interval_ms) : Lds(interval_ms, kSourceRawLidar) {
  is_initialized_ = false;
  ResetLdsLidar();
}

LdsLidar::~LdsLidar() {}

void LdsLidar::ResetLdsLidar(void) { ResetLds(kSourceRawLidar); }

int LdsLidar::InitLdsLidar(
  const std::optional<UserRawConfig>& lidar_config,
  const std::optional<TimeSyncRawConfig>& timesync_config)
{
  if (is_initialized_) {
    printf("LiDAR data source is already inited!\n");
    return -1;
  }

  if (!Init()) {
    Uninit();
    printf("Livox-SDK init fail!\n");
    return -1;
  }

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  printf("Livox SDK version %d.%d.%d\n",
    _sdkversion.major,
    _sdkversion.minor,
    _sdkversion.patch);

  SetBroadcastCallback(OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(OnDeviceChange);

  // Check if the device config is present.
  if (lidar_config.has_value())
  {
    SetRawConfig(*lidar_config);
  }
  else
  {
      // Use the default.
      printf("Could not find raw config, set config to default!\n");
      raw_config_.enable_fan = 1;
      raw_config_.return_mode = kFirstReturn;
      raw_config_.coordinate = kCoordinateCartesian;
      raw_config_.imu_rate = kImuFreq200Hz;
      raw_config_.extrinsic_parameter_source = kNoneExtrinsicParameter;
      raw_config_.enable_high_sensitivity = false;
  }

  // Check if the timesync config is present.
  if (timesync_config.has_value())
  {
    if (ParseTimesyncConfig(*timesync_config)) {
      printf("Parse timesync config fail\n");
      enable_timesync_ = false;
    }

    if (enable_timesync_) {
      timesync_ = TimeSync::GetInstance();
      if (timesync_->InitTimeSync(timesync_config_)) {
        printf("Timesync init fail\n");
        return -1;
      }

      if (timesync_->SetReceiveSyncTimeCb(ReceiveSyncTimeCallback, this)) {
        printf("Set Timesync callback fail\n");
        return -1;
      }

      timesync_->StartTimesync();
    }
  }

  /** Start livox sdk to receive lidar data */
  if (!Start()) {
    Uninit();
    printf("Livox-SDK init fail!\n");
    return -1;
  }

  /** Add here, only for callback use */
  if (g_lds_ldiar == nullptr) {
    g_lds_ldiar = this;
  }
  is_initialized_ = true;
  printf("Livox-SDK init success!\n");

  return 0;
}

int LdsLidar::DeInitLdsLidar(void) {
  if (!is_initialized_) {
    printf("LiDAR data source is not exit");
    return -1;
  }

  Uninit();
  printf("Livox SDK Deinit completely!\n");

  if (timesync_) {
    timesync_->DeInitTimeSync();
  }

  return 0;
}

void LdsLidar::PrepareExit(void) { DeInitLdsLidar(); }

/** Static function in LdsLidar for callback or event process ----------------*/

/** Receiving point cloud data from Livox LiDAR. */
void LdsLidar::OnLidarDataCb(uint8_t handle, LivoxEthPacket *data,
                             uint32_t data_num, void *client_data) {
  using namespace std;

  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  LivoxEthPacket *eth_packet = data;

  if (!data || !data_num || (handle >= kMaxLidarCount)) {
    return;
  }

  lds_lidar->StorageRawPacket(handle, eth_packet);
}

void LdsLidar::OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == nullptr) {
    return;
  }

  if (info->dev_type == kDeviceTypeHub) {
    printf("In lidar mode, couldn't connect a hub : %s\n",
           info->broadcast_code);
    return;
  }

  bool result = false;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess && handle < kMaxLidarCount) {
    SetDataCallback(handle, OnLidarDataCb, (void *)g_lds_ldiar);

    LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[handle]);
    p_lidar->handle = handle;
    p_lidar->connect_state = kConnectStateOff;

    UserRawConfig config = g_lds_ldiar->GetConfig();
    p_lidar->config.enable_fan = config.enable_fan;
    p_lidar->config.return_mode = config.return_mode;
    p_lidar->config.coordinate = config.coordinate;
    p_lidar->config.imu_rate = config.imu_rate;
    p_lidar->config.extrinsic_parameter_source =
        config.extrinsic_parameter_source;
    p_lidar->config.enable_high_sensitivity = config.enable_high_sensitivity;
  } else {
    printf("Add lidar to connect is failed : %d %d \n", result, handle);
  }
}

/** Callback function of changing of device state. */
void LdsLidar::OnDeviceChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == nullptr) {
    return;
  }

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[handle]);
  if (type == kEventConnect) {
    QueryDeviceInformation(handle, DeviceInformationCb, g_lds_ldiar);
    if (p_lidar->connect_state == kConnectStateOff) {
      p_lidar->connect_state = kConnectStateOn;
      p_lidar->info = *info;
    }
  } else if (type == kEventDisconnect) {
    printf("Lidar[%s] disconnect!\n", info->broadcast_code);
    ResetLidar(p_lidar, kSourceRawLidar);
  } else if (type == kEventStateChange) {
    p_lidar->info = *info;
  }

  if (p_lidar->connect_state == kConnectStateOn) {
    printf("Lidar[%s] status_code[%d] working state[%d] feature[%d]\n",
           p_lidar->info.broadcast_code,
           p_lidar->info.status.status_code.error_code, p_lidar->info.state,
           p_lidar->info.feature);
    SetErrorMessageCallback(handle, LidarErrorStatusCb);

    /** Config lidar parameter */
    if (p_lidar->info.state == kLidarStateNormal) {
      /** Ensure the thread safety for set_bits and connect_state */
      lock_guard<mutex> lock(g_lds_ldiar->config_mutex_);

      if (p_lidar->config.coordinate != 0) {
        SetSphericalCoordinate(handle, SetCoordinateCb, g_lds_ldiar);
      } else {
        SetCartesianCoordinate(handle, SetCoordinateCb, g_lds_ldiar);
      }
      p_lidar->config.set_bits |= kConfigCoordinate;

      if (kDeviceTypeLidarMid40 != info->type) {
        LidarSetPointCloudReturnMode(
            handle, (PointCloudReturnMode)(p_lidar->config.return_mode),
            SetPointCloudReturnModeCb, g_lds_ldiar);
        p_lidar->config.set_bits |= kConfigReturnMode;
      }

      if ((kDeviceTypeLidarMid70 != info->type) &&
          (kDeviceTypeLidarMid40 != info->type)) {
        LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),
                                 SetImuRatePushFrequencyCb, g_lds_ldiar);
        p_lidar->config.set_bits |= kConfigImuRate;
      }

      if (p_lidar->config.extrinsic_parameter_source ==
          kExtrinsicParameterFromLidar) {
        LidarGetExtrinsicParameter(handle, GetLidarExtrinsicParameterCb,
                                   g_lds_ldiar);
        p_lidar->config.set_bits |= kConfigGetExtrinsicParameter;
      }

      if (kDeviceTypeLidarTele == info->type) {
        if (p_lidar->config.enable_high_sensitivity) {
          LidarEnableHighSensitivity(handle, SetHighSensitivityCb, g_lds_ldiar);
          printf("Enable high sensitivity\n");
        } else {
          LidarDisableHighSensitivity(handle, SetHighSensitivityCb,
                                      g_lds_ldiar);
          printf("Disable high sensitivity\n");
        }
        p_lidar->config.set_bits |= kConfigSetHighSensitivity;
      }

      p_lidar->connect_state = kConnectStateConfig;
    }
  }
}

/** Query the firmware version of Livox LiDAR. */
void LdsLidar::DeviceInformationCb(livox_status status, uint8_t handle,
                                   DeviceInformationResponse *ack,
                                   void *clent_data) {
  if (status != kStatusSuccess) {
    printf("Device Query Informations Failed : %d\n", status);
  }
  if (ack) {
    printf("firmware version: %d.%d.%d.%d\n", ack->firmware_version[0],
           ack->firmware_version[1], ack->firmware_version[2],
           ack->firmware_version[3]);
  }
}

/** Callback function of Lidar error message. */
void LdsLidar::LidarErrorStatusCb(livox_status status, uint8_t handle,
                                  ErrorMessage *message) {
  static uint32_t error_message_count = 0;
  if (message != NULL) {
    ++error_message_count;
    if (0 == (error_message_count % 100)) {
      printf("handle: %u\n", handle);
      printf("temp_status : %u\n", message->lidar_error_code.temp_status);
      printf("volt_status : %u\n", message->lidar_error_code.volt_status);
      printf("motor_status : %u\n", message->lidar_error_code.motor_status);
      printf("dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
      printf("firmware_err : %u\n", message->lidar_error_code.firmware_err);
      printf("pps_status : %u\n", message->lidar_error_code.device_status);
      printf("fan_status : %u\n", message->lidar_error_code.fan_status);
      printf("self_heating : %u\n", message->lidar_error_code.self_heating);
      printf("ptp_status : %u\n", message->lidar_error_code.ptp_status);
      printf("time_sync_status : %u\n",
             message->lidar_error_code.time_sync_status);
      printf("system_status : %u\n", message->lidar_error_code.system_status);
    }
  }
}

void LdsLidar::ControlFanCb(livox_status status, uint8_t handle,
                            uint8_t response, void *clent_data) {}

void LdsLidar::SetPointCloudReturnModeCb(livox_status status, uint8_t handle,
                                         uint8_t response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    printf("Set return mode success!\n");

    lock_guard<mutex> lock(lds_lidar->config_mutex_);
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigReturnMode));
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    LidarSetPointCloudReturnMode(
        handle, (PointCloudReturnMode)(p_lidar->config.return_mode),
        SetPointCloudReturnModeCb, lds_lidar);
    printf("Set return mode fail, try again!\n");
  }
}

void LdsLidar::SetCoordinateCb(livox_status status, uint8_t handle,
                               uint8_t response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    printf("Set coordinate success!\n");

    lock_guard<mutex> lock(lds_lidar->config_mutex_);
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigCoordinate));
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    if (p_lidar->config.coordinate != 0) {
      SetSphericalCoordinate(handle, SetCoordinateCb, lds_lidar);
    } else {
      SetCartesianCoordinate(handle, SetCoordinateCb, lds_lidar);
    }

    printf("Set coordinate fail, try again!\n");
  }
}

void LdsLidar::SetImuRatePushFrequencyCb(livox_status status, uint8_t handle,
                                         uint8_t response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    printf("Set imu rate success!\n");

    lock_guard<mutex> lock(lds_lidar->config_mutex_);
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigImuRate));
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),
                             SetImuRatePushFrequencyCb, g_lds_ldiar);
    printf("Set imu rate fail, try again!\n");
  }
}

/** Callback function of get LiDARs' extrinsic parameter. */
void LdsLidar::GetLidarExtrinsicParameterCb(
    livox_status status, uint8_t handle,
    LidarGetExtrinsicParameterResponse *response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);
  if (handle >= kMaxLidarCount) {
    return;
  }

  if (status == kStatusSuccess) {
    if (response != nullptr) {
      printf("Lidar[%d] get ExtrinsicParameter status[%d] response[%d]\n",
             handle, status, response->ret_code);
      LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);
      ExtrinsicParameter *p_extrinsic = &p_lidar->extrinsic_parameter;
      p_extrinsic->euler[0] = static_cast<float>(response->roll * PI / 180.0);
      p_extrinsic->euler[1] = static_cast<float>(response->pitch * PI / 180.0);
      p_extrinsic->euler[2] = static_cast<float>(response->yaw * PI / 180.0);
      p_extrinsic->trans[0] = static_cast<float>(response->x / 1000.0);
      p_extrinsic->trans[1] = static_cast<float>(response->y / 1000.0);
      p_extrinsic->trans[2] = static_cast<float>(response->z / 1000.0);
      EulerAnglesToRotationMatrix(p_extrinsic->euler, p_extrinsic->rotation);
      if (p_lidar->config.extrinsic_parameter_source) {
        p_extrinsic->enable = true;
      }
      printf("Lidar[%d] get ExtrinsicParameter success!\n", handle);

      lock_guard<mutex> lock(lds_lidar->config_mutex_);
      p_lidar->config.set_bits &= ~((uint32_t)(kConfigGetExtrinsicParameter));
      if (!p_lidar->config.set_bits) {
        LidarStartSampling(handle, StartSampleCb, lds_lidar);
        p_lidar->connect_state = kConnectStateSampling;
      }
    } else {
      printf("Lidar[%d] get ExtrinsicParameter fail!\n", handle);
    }
  } else if (status == kStatusTimeout) {
    printf("Lidar[%d] get ExtrinsicParameter timeout!\n", handle);
  }
}

void LdsLidar::SetHighSensitivityCb(livox_status status, uint8_t handle,
                                    DeviceParameterResponse *response,
                                    void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigSetHighSensitivity));
    printf("Set high sensitivity success!\n");

    lock_guard<mutex> lock(lds_lidar->config_mutex_);
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    };
  } else {
    if (p_lidar->config.enable_high_sensitivity) {
      LidarEnableHighSensitivity(handle, SetHighSensitivityCb, g_lds_ldiar);
    } else {
      LidarDisableHighSensitivity(handle, SetHighSensitivityCb, g_lds_ldiar);
    }
    printf("Set high sensitivity fail, try again!\n");
  }
}

/** Callback function of starting sampling. */
void LdsLidar::StartSampleCb(livox_status status, uint8_t handle,
                             uint8_t response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);

  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);
  if (status == kStatusSuccess) {
    if (response != 0) {
      p_lidar->connect_state = kConnectStateOn;
      printf("Lidar start sample fail : state[%d] handle[%d] res[%d]\n", status,
             handle, response);
    } else {
      printf("Lidar start sample success\n");
    }
  } else if (status == kStatusTimeout) {
    p_lidar->connect_state = kConnectStateOn;
    printf("Lidar start sample timeout : state[%d] handle[%d] res[%d]\n",
           status, handle, response);
  }
}

/** Callback function of stopping sampling. */
void LdsLidar::StopSampleCb(livox_status status, uint8_t handle,
                            uint8_t response, void *clent_data) {}

void LdsLidar::SetRmcSyncTimeCb(livox_status status, uint8_t handle,
                                uint8_t response, void *client_data) {
  if (handle >= kMaxLidarCount) {
    return;
  }
  printf("Set lidar[%d] sync time status[%d] response[%d]\n", handle, status,
         response);
}

void LdsLidar::ReceiveSyncTimeCallback(const char *rmc, uint32_t rmc_length,
                                       void *client_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  // std::unique_lock<std::mutex> lock(mtx);
  LidarDevice *p_lidar = nullptr;
  for (uint8_t handle = 0; handle < kMaxLidarCount; handle++) {
    p_lidar = &(lds_lidar->lidars_[handle]);
    if (p_lidar->connect_state == kConnectStateSampling &&
        p_lidar->info.state == kLidarStateNormal) {
      livox_status status = LidarSetRmcSyncTime(handle, rmc, rmc_length,
                                                SetRmcSyncTimeCb, lds_lidar);
      if (status != kStatusSuccess) {
        printf("Set GPRMC synchronization time error code: %d.\n", status);
      }
    }
  }
}

int LdsLidar::ParseTimesyncConfig(const TimeSyncRawConfig& timesync_config)
{
  // Store the timesync enable flag.
  enable_timesync_ = timesync_config.enable_timesync;

  // Store the device name.
  std::strncpy(
    timesync_config_.dev_config.name,
    timesync_config.device_name.c_str(),
    sizeof(timesync_config_.dev_config.name));

  // Store the device type.
  timesync_config_.dev_config.type = timesync_config.comm_device_type;

  // If UART.
  if (timesync_config_.dev_config.type == kCommDevUart) {
    // Store baudrate.
    timesync_config_.dev_config.config.uart.baudrate = timesync_config.baudrate_index;
    // store parity.
    timesync_config_.dev_config.config.uart.parity = timesync_config.parity_index;
  }

  if (enable_timesync_) {
    printf("Enable timesync : \n");
    if (timesync_config_.dev_config.type == kCommDevUart) {
      printf("Uart[%s],baudrate index[%d],parity index[%d]\n",
              timesync_config_.dev_config.name,
              timesync_config_.dev_config.config.uart.baudrate,
              timesync_config_.dev_config.config.uart.parity);
    }
  } else {
    printf("Disable timesync\n");
  }
  return 0;
}

}  // namespace livox_ros
