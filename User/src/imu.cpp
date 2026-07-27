#include "imu.hpp"

#include "ti_msp_dl_config.h"

namespace App
{

namespace
{

constexpr float RAD_TO_DEG = static_cast<float>(180.0 / LibXR::PI);

}  // namespace

Imu::InterruptPin::InterruptPin()
    : gpio(ICM42688_INT_PORT, ICM42688_INT_INT1_PIN, ICM42688_INT_INT1_IOMUX)
{
  ASSERT(gpio.SetConfig({LibXR::GPIO::Direction::FALL_INTERRUPT,
                         LibXR::GPIO::Pull::UP}) == LibXR::ErrorCode::OK);
  gpio.DisableInterrupt();
}

Imu::Imu()
    : i2c_(MSPM0_I2C_INIT(I2C_0, i2c_stage_buffer_, sizeof(i2c_stage_buffer_), 8),
           LibXR::I2C::Configuration{I2C_0_BUS_SPEED_HZ}),
      interrupt_(),
      application_manager_(),
      hardware_(LibXR::Entry<LibXR::I2C>{i2c_, {"i2c_icm42688"}},
                LibXR::Entry<LibXR::GPIO>{interrupt_.gpio, {"icm42688_int"}}),
      imu_(hardware_, application_manager_, ICM42688::DataRate::DATA_RATE_1KHZ,
           ICM42688::AcclRange::RANGE_16G, ICM42688::GyroRange::DPS_2000,
           LibXR::Quaternion<float>(1.0F, 0.0F, 0.0F, 0.0F), false, "icm42688_gyro",
           "icm42688_accl", 0x68)
{
}

void Imu::Process()
{
  if (!imu_.Process())
  {
    if (!imu_.IsOnline() && gyro_calibration_started_ && !gyro_calibrated_)
    {
      gyro_calibrated_ = true;
      gyro_calibration_started_ = false;
      gyro_calibration_sum_.setZero();
      gyro_calibration_sample_count_ = 0;
      ahrs_.Reset();
      XR_LOG_WARN("Gyro calibration interrupted; bias disabled.");
    }
    return;
  }

  const auto& raw_gyro = imu_.GetGyro();
  if (!gyro_calibrated_)
  {
    if (!raw_gyro.allFinite())
    {
      return;
    }

    const uint32_t now = LibXR::Timebase::GetMilliseconds();
    if (!gyro_calibration_started_)
    {
      gyro_calibration_started_ = true;
      gyro_calibration_start_ms_ = now;
      gyro_calibration_sum_.setZero();
      gyro_calibration_sample_count_ = 0;
      XR_LOG_INFO("Gyro calibration: keep still for 2 seconds.");
    }

    gyro_calibration_sum_ += raw_gyro;
    gyro_calibration_sample_count_++;

    if (now - gyro_calibration_start_ms_ < GYRO_CALIBRATION_DURATION_MS)
    {
      return;
    }

    gyro_bias_ =
        gyro_calibration_sum_ / static_cast<float>(gyro_calibration_sample_count_);
    gyro_calibrated_ = true;
    ahrs_.Reset();
    XR_LOG_PASS("Gyro bias(deg/s): x=%.4f y=%.4f z=%.4f", gyro_bias_.x() * RAD_TO_DEG,
                gyro_bias_.y() * RAD_TO_DEG, gyro_bias_.z() * RAD_TO_DEG);
    return;
  }

  const Eigen::Matrix<float, 3, 1> corrected_gyro = raw_gyro - gyro_bias_;
  if (!ahrs_.Update(corrected_gyro, imu_.GetAccl(), imu_.GetSampleIntervalSeconds()))
  {
    XR_LOG_WARN("AHRS update failed.");
  }
}

void Imu::Monitor() { application_manager_.MonitorAll(); }

bool Imu::IsOnline() const { return imu_.IsOnline(); }

uint8_t Imu::GetWhoAmI() const { return imu_.GetWhoAmI(); }

uint8_t Imu::GetLastFailedRegister() const { return imu_.GetLastFailedRegister(); }

LibXR::ErrorCode Imu::GetLastError() const { return imu_.GetLastError(); }

uint32_t Imu::GetSampleCount() const { return imu_.GetSampleCount(); }

float Imu::GetTemperature() const { return imu_.GetTemperature(); }

const Eigen::Matrix<float, 3, 1>& Imu::GetAccl() const { return imu_.GetAccl(); }

const Eigen::Matrix<float, 3, 1>& Imu::GetGyro() const { return imu_.GetGyro(); }

bool Imu::IsAttitudeValid() const
{
  return imu_.IsOnline() && gyro_calibrated_ && ahrs_.IsValid();
}

const LibXR::Quaternion<float>& Imu::GetQuaternion() const
{
  return ahrs_.GetQuaternion();
}

const Eigen::Matrix<float, 3, 1>& Imu::GetEuler() const { return ahrs_.GetEuler(); }

float Imu::GetContinuousYawDegrees() const { return ahrs_.GetContinuousYawDegrees(); }

}  // namespace App
