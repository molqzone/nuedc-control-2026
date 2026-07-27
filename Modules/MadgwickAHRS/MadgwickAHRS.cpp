#include "MadgwickAHRS.hpp"

#include <cmath>

MadgwickAHRS::MadgwickAHRS(float beta) : beta_(beta) { Reset(); }

float MadgwickAHRS::InvSqrtf(float value) { return 1.0F / std::sqrt(value); }

float MadgwickAHRS::ResolveDt(float dt_seconds)
{
  if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0F)
  {
    return kDefaultDtSeconds;
  }
  return dt_seconds;
}

bool MadgwickAHRS::Update(const Eigen::Matrix<float, 3, 1>& gyro_rad_s,
                          const Eigen::Matrix<float, 3, 1>& accl_g, float dt_seconds)
{
  if (!gyro_rad_s.allFinite() || !accl_g.allFinite())
  {
    return false;
  }

  const float dt = ResolveDt(dt_seconds);
  float ax = accl_g.x();
  float ay = accl_g.y();
  float az = accl_g.z();
  const float gx = gyro_rad_s.x();
  const float gy = gyro_rad_s.y();
  const float gz = gyro_rad_s.z();

  float q_dot1 =
      0.5F * (-quaternion_.x() * gx - quaternion_.y() * gy - quaternion_.z() * gz);
  float q_dot2 =
      0.5F * (quaternion_.w() * gx + quaternion_.y() * gz - quaternion_.z() * gy);
  float q_dot3 =
      0.5F * (quaternion_.w() * gy - quaternion_.x() * gz + quaternion_.z() * gx);
  float q_dot4 =
      0.5F * (quaternion_.w() * gz + quaternion_.x() * gy - quaternion_.y() * gx);

  if (!((ax == 0.0F) && (ay == 0.0F) && (az == 0.0F)))
  {
    float recip_norm = InvSqrtf(ax * ax + ay * ay + az * az);
    if (!std::isfinite(recip_norm))
    {
      return false;
    }
    ax *= recip_norm;
    ay *= recip_norm;
    az *= recip_norm;

    const float q_2q0 = 2.0F * quaternion_.w();
    const float q_2q1 = 2.0F * quaternion_.x();
    const float q_2q2 = 2.0F * quaternion_.y();
    const float q_2q3 = 2.0F * quaternion_.z();
    const float q_4q0 = 4.0F * quaternion_.w();
    const float q_4q1 = 4.0F * quaternion_.x();
    const float q_4q2 = 4.0F * quaternion_.y();
    const float q_8q1 = 8.0F * quaternion_.x();
    const float q_8q2 = 8.0F * quaternion_.y();
    const float q0q0 = quaternion_.w() * quaternion_.w();
    const float q1q1 = quaternion_.x() * quaternion_.x();
    const float q2q2 = quaternion_.y() * quaternion_.y();
    const float q3q3 = quaternion_.z() * quaternion_.z();

    float s0 = q_4q0 * q2q2 + q_2q2 * ax + q_4q0 * q1q1 - q_2q1 * ay;
    float s1 = q_4q1 * q3q3 - q_2q3 * ax + 4.0F * q0q0 * quaternion_.x() - q_2q0 * ay -
               q_4q1 + q_8q1 * q1q1 + q_8q1 * q2q2 + q_4q1 * az;
    float s2 = 4.0F * q0q0 * quaternion_.y() + q_2q0 * ax + q_4q2 * q3q3 - q_2q3 * ay -
               q_4q2 + q_8q2 * q1q1 + q_8q2 * q2q2 + q_4q2 * az;
    float s3 = 4.0F * q1q1 * quaternion_.z() - q_2q1 * ax +
               4.0F * q2q2 * quaternion_.z() - q_2q2 * ay;

    const float step_norm_sq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
    if (step_norm_sq > kEpsilon)
    {
      recip_norm = InvSqrtf(step_norm_sq);
      if (!std::isfinite(recip_norm))
      {
        return false;
      }
      s0 *= recip_norm;
      s1 *= recip_norm;
      s2 *= recip_norm;
      s3 *= recip_norm;

      q_dot1 -= beta_ * s0;
      q_dot2 -= beta_ * s1;
      q_dot3 -= beta_ * s2;
      q_dot4 -= beta_ * s3;
    }
  }

  quaternion_.w() += q_dot1 * dt;
  quaternion_.x() += q_dot2 * dt;
  quaternion_.y() += q_dot3 * dt;
  quaternion_.z() += q_dot4 * dt;

  const float quaternion_norm_sq =
      quaternion_.w() * quaternion_.w() + quaternion_.x() * quaternion_.x() +
      quaternion_.y() * quaternion_.y() + quaternion_.z() * quaternion_.z();
  if (!std::isfinite(quaternion_norm_sq) || quaternion_norm_sq <= kEpsilon)
  {
    Reset();
    return false;
  }

  const float recip_norm = InvSqrtf(quaternion_norm_sq);
  quaternion_.w() *= recip_norm;
  quaternion_.x() *= recip_norm;
  quaternion_.y() *= recip_norm;
  quaternion_.z() *= recip_norm;

  UpdateEuler();
  valid_ = euler_.allFinite();
  if (!valid_)
  {
    Reset();
  }
  return valid_;
}

void MadgwickAHRS::Reset()
{
  quaternion_ = LibXR::Quaternion<float>(1.0F, 0.0F, 0.0F, 0.0F);
  euler_.setZero();
  previous_wrapped_yaw_deg_ = 0.0F;
  continuous_yaw_deg_ = 0.0F;
  yaw_initialized_ = false;
  valid_ = false;
}

bool MadgwickAHRS::IsValid() const { return valid_; }

const LibXR::Quaternion<float>& MadgwickAHRS::GetQuaternion() const
{
  return quaternion_;
}

const Eigen::Matrix<float, 3, 1>& MadgwickAHRS::GetEuler() const { return euler_; }

float MadgwickAHRS::GetContinuousYawDegrees() const { return continuous_yaw_deg_; }

void MadgwickAHRS::SetBeta(float beta) { beta_ = beta; }

void MadgwickAHRS::UpdateEuler()
{
  euler_ = quaternion_.ToEulerAngleZYX();

  constexpr float RAD_TO_DEG = static_cast<float>(180.0 / LibXR::PI);
  const float wrapped_yaw_deg = euler_.z() * RAD_TO_DEG;
  if (!yaw_initialized_)
  {
    previous_wrapped_yaw_deg_ = wrapped_yaw_deg;
    continuous_yaw_deg_ = wrapped_yaw_deg;
    yaw_initialized_ = true;
    return;
  }

  float delta_deg = wrapped_yaw_deg - previous_wrapped_yaw_deg_;
  if (delta_deg > 180.0F)
  {
    delta_deg -= 360.0F;
  }
  else if (delta_deg < -180.0F)
  {
    delta_deg += 360.0F;
  }
  continuous_yaw_deg_ += delta_deg;
  previous_wrapped_yaw_deg_ = wrapped_yaw_deg;
}
