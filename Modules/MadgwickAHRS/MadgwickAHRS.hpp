#pragma once

#include "libxr.hpp"

class MadgwickAHRS
{
 public:
  explicit MadgwickAHRS(float beta = 0.05F);

  bool Update(const Eigen::Matrix<float, 3, 1>& gyro_rad_s,
              const Eigen::Matrix<float, 3, 1>& accl_g, float dt_seconds);
  void Reset();

  bool IsValid() const;
  const LibXR::Quaternion<float>& GetQuaternion() const;
  const Eigen::Matrix<float, 3, 1>& GetEuler() const;
  float GetContinuousYawDegrees() const;
  void SetBeta(float beta);

 private:
  static constexpr float kDefaultDtSeconds = 0.001F;
  static constexpr float kEpsilon = 1e-6F;

  static float InvSqrtf(float value);
  static float ResolveDt(float dt_seconds);
  void UpdateEuler();

  float beta_ = 0.05F;
  LibXR::Quaternion<float> quaternion_;
  Eigen::Matrix<float, 3, 1> euler_ = Eigen::Matrix<float, 3, 1>::Zero();
  float previous_wrapped_yaw_deg_ = 0.0F;
  float continuous_yaw_deg_ = 0.0F;
  bool yaw_initialized_ = false;
  bool valid_ = false;
};
