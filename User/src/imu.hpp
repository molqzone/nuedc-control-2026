#pragma once

#include <cstdint>

#include "ICM42688.hpp"
#include "MadgwickAHRS.hpp"
#include "libxr.hpp"
#include "mspm0_gpio.hpp"
#include "mspm0_i2c.hpp"

namespace App
{

class Imu
{
 public:
  Imu();

  void Process();
  void Monitor();

  bool IsOnline() const;
  uint8_t GetWhoAmI() const;
  uint8_t GetLastFailedRegister() const;
  LibXR::ErrorCode GetLastError() const;
  uint32_t GetSampleCount() const;
  float GetTemperature() const;
  const Eigen::Matrix<float, 3, 1>& GetAccl() const;
  const Eigen::Matrix<float, 3, 1>& GetGyro() const;
  bool IsAttitudeValid() const;
  const LibXR::Quaternion<float>& GetQuaternion() const;
  const Eigen::Matrix<float, 3, 1>& GetEuler() const;
  float GetContinuousYawDegrees() const;

 private:
  static constexpr uint32_t GYRO_CALIBRATION_DURATION_MS = 2000;

  struct InterruptPin
  {
    InterruptPin();
    LibXR::MSPM0GPIO gpio;
  };

  uint8_t i2c_stage_buffer_[32]{};
  LibXR::MSPM0I2C i2c_;
  InterruptPin interrupt_;
  LibXR::ApplicationManager application_manager_;
  LibXR::HardwareContainer hardware_;
  ICM42688 imu_;
  MadgwickAHRS ahrs_;
  Eigen::Matrix<float, 3, 1> gyro_bias_ = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> gyro_calibration_sum_ = Eigen::Matrix<float, 3, 1>::Zero();
  uint32_t gyro_calibration_start_ms_ = 0;
  uint32_t gyro_calibration_sample_count_ = 0;
  bool gyro_calibration_started_ = false;
  bool gyro_calibrated_ = false;
};

}  // namespace App
