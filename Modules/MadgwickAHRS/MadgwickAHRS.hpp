#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Madgwick attitude estimator consuming gyro/accl topics.
constructor_args:
  - beta: 0.05
  - gyro_topic_name: "icm42688_gyro"
  - accl_topic_name: "icm42688_accl"
template_args: []
required_hardware: []
depends:
  - ICM42688
=== END MANIFEST === */
// clang-format on

#include "libxr.hpp"

class MadgwickAHRS : public LibXR::Application
{
 public:
  explicit MadgwickAHRS(float beta = 0.05F);
  MadgwickAHRS(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
               float beta, const char* gyro_topic_name,
               const char* accl_topic_name);

  bool Update(const Eigen::Matrix<float, 3, 1>& gyro_rad_s,
              const Eigen::Matrix<float, 3, 1>& accl_g, float dt_seconds);
  void Reset();
  void OnMonitor() override;

  bool IsValid() const;
  const LibXR::Quaternion<float>& GetQuaternion() const;
  const Eigen::Matrix<float, 3, 1>& GetEuler() const;
  float GetContinuousYawDegrees() const;
  void SetBeta(float beta);

 private:
  static constexpr float kDefaultDtSeconds = 0.001F;
  static constexpr float kEpsilon = 1e-6F;
  static constexpr uint32_t GYRO_CALIBRATION_DURATION_MS = 2000;

  static float InvSqrtf(float value);
  static float ResolveDt(float dt_seconds);
  void UpdateEuler();
  bool UpdateFromTopics();

  float beta_ = 0.05F;
  LibXR::Quaternion<float> quaternion_;
  Eigen::Matrix<float, 3, 1> euler_ = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> gyro_ = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> accl_ = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> gyro_bias_ = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> gyro_calibration_sum_ =
      Eigen::Matrix<float, 3, 1>::Zero();
  float previous_wrapped_yaw_deg_ = 0.0F;
  float continuous_yaw_deg_ = 0.0F;
  uint32_t gyro_calibration_start_ms_ = 0;
  uint32_t gyro_calibration_sample_count_ = 0;
  uint32_t last_update_ms_ = 0;
  uint32_t sample_count_ = 0;
  bool gyro_calibration_started_ = false;
  bool gyro_calibrated_ = false;
  bool yaw_initialized_ = false;
  bool valid_ = false;
  LibXR::Topic* gyro_topic_ = nullptr;
  LibXR::Topic* accl_topic_ = nullptr;
  LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>>* gyro_sub_ = nullptr;
  LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>>* accl_sub_ = nullptr;
};
