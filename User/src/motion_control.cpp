#include "motion_control.hpp"

#include <algorithm>
#include <cmath>

namespace App
{

MotionControl::MotionControl() : MotionControl(Configuration{}) {}

MotionControl::MotionControl(const Configuration& config)
    : config_(config),
      line_pid_(config.line_pid),
      angle_pid_(config.angle_pid),
      base_target_(config.default_base_target)
{
  ASSERT(config_.outer_loop_divider > 0U);
  ASSERT(config_.turn_stable_outer_cycles > 0U);
  ASSERT(config_.turn_timeout_outer_cycles > 0U);
}

void MotionControl::HandleCommand(uint8_t command)
{
  switch (command)
  {
    case 'g':
    case 'G':
      SetMode(DriveMode::LINE_FOLLOW);
      break;
    case 'w':
    case 'W':
      manual_left_ = 1;
      manual_right_ = 1;
      SetMode(DriveMode::MANUAL);
      break;
    case 's':
    case 'S':
      manual_left_ = -1;
      manual_right_ = -1;
      SetMode(DriveMode::MANUAL);
      break;
    case 'a':
    case 'A':
      manual_left_ = -1;
      manual_right_ = 1;
      SetMode(DriveMode::MANUAL);
      break;
    case 'd':
    case 'D':
      manual_left_ = 1;
      manual_right_ = -1;
      SetMode(DriveMode::MANUAL);
      break;
    case '+':
      base_target_ = std::min(base_target_ + 5, config_.max_base_target);
      outer_update_pending_ = true;
      break;
    case '-':
      base_target_ = std::max(base_target_ - 5, config_.min_base_target);
      outer_update_pending_ = true;
      break;
    case 'x':
    case 'X':
    case ' ':
      manual_left_ = 0;
      manual_right_ = 0;
      SetMode(DriveMode::STOPPED);
      break;
    default:
      break;
  }
}

bool MotionControl::StartRelativeTurn(float delta_deg)
{
  if (!latest_attitude_valid_ || !std::isfinite(delta_deg) ||
      std::fabs(delta_deg) > MAX_RELATIVE_TURN_DEG)
  {
    return false;
  }

  turn_target_yaw_deg_ = latest_continuous_yaw_deg_ + delta_deg;
  turn_stable_cycles_ = 0;
  turn_elapsed_cycles_ = 0;
  SetMode(DriveMode::ANGLE_TURN);
  ResetControllers();
  return true;
}

MotionControl::Outputs MotionControl::Step(const Inputs& inputs)
{
  latest_continuous_yaw_deg_ = inputs.continuous_yaw_deg;
  latest_attitude_valid_ =
      inputs.attitude_valid && std::isfinite(inputs.continuous_yaw_deg);

  if (mode_ == DriveMode::ANGLE_TURN && !latest_attitude_valid_)
  {
    SetMode(DriveMode::STOPPED);
  }

  if (mode_ == DriveMode::MANUAL)
  {
    left_target_ = base_target_ * manual_left_;
    right_target_ = base_target_ * manual_right_;
  }
  else if (mode_ == DriveMode::STOPPED)
  {
    left_target_ = 0;
    right_target_ = 0;
  }
  else
  {
    outer_loop_counter_++;
    if (outer_update_pending_ || outer_loop_counter_ >= config_.outer_loop_divider)
    {
      outer_loop_counter_ = 0;
      outer_update_pending_ = false;
      UpdateOuterLoop(inputs);
    }
  }

  return {left_target_, right_target_};
}

MotionControl::DriveMode MotionControl::GetMode() const { return mode_; }

float MotionControl::GetTurnTargetYawDeg() const { return turn_target_yaw_deg_; }

void MotionControl::SetMode(DriveMode mode)
{
  if (mode_ == mode)
  {
    outer_update_pending_ =
        mode == DriveMode::LINE_FOLLOW || mode == DriveMode::ANGLE_TURN;
    return;
  }

  mode_ = mode;
  left_target_ = 0;
  right_target_ = 0;
  outer_loop_counter_ = 0;
  outer_update_pending_ = mode == DriveMode::LINE_FOLLOW || mode == DriveMode::ANGLE_TURN;
  ResetControllers();
}

void MotionControl::ResetControllers()
{
  line_pid_.Reset();
  angle_pid_.Reset();
}

void MotionControl::UpdateOuterLoop(const Inputs& inputs)
{
  if (mode_ == DriveMode::LINE_FOLLOW)
  {
    UpdateLineTargets(inputs);
  }
  else if (mode_ == DriveMode::ANGLE_TURN)
  {
    UpdateAngleTargets(inputs);
  }
}

void MotionControl::UpdateLineTargets(const Inputs& inputs)
{
  if (!inputs.line_detected || !std::isfinite(inputs.line_position))
  {
    line_pid_.Reset();
    left_target_ = 0;
    right_target_ = 0;
    return;
  }

  const float line_error = inputs.line_position / GREY_SENSOR_POSITION_SCALE;
  const float correction = line_pid_.Calculate(0.0F, line_error, OUTER_DT_SECONDS);
  SetWheelTargets(static_cast<float>(base_target_) - correction,
                  static_cast<float>(base_target_) + correction, false);
}

void MotionControl::UpdateAngleTargets(const Inputs& inputs)
{
  if (!inputs.attitude_valid || !std::isfinite(inputs.continuous_yaw_deg))
  {
    SetMode(DriveMode::STOPPED);
    return;
  }

  turn_elapsed_cycles_++;
  const float error_deg = turn_target_yaw_deg_ - inputs.continuous_yaw_deg;
  if (std::fabs(error_deg) <= config_.turn_tolerance_deg)
  {
    turn_stable_cycles_++;
    if (turn_stable_cycles_ >= config_.turn_stable_outer_cycles)
    {
      SetMode(DriveMode::LINE_FOLLOW);
      return;
    }
  }
  else
  {
    turn_stable_cycles_ = 0;
  }

  if (turn_elapsed_cycles_ >= config_.turn_timeout_outer_cycles)
  {
    SetMode(DriveMode::STOPPED);
    return;
  }

  const float correction = angle_pid_.Calculate(
      turn_target_yaw_deg_, inputs.continuous_yaw_deg, OUTER_DT_SECONDS);
  SetWheelTargets(static_cast<float>(base_target_) - correction,
                  static_cast<float>(base_target_) + correction, true);
}

void MotionControl::SetWheelTargets(float left_target, float right_target,
                                    bool allow_reverse)
{
  const float minimum =
      allow_reverse ? -static_cast<float>(config_.max_wheel_target) : 0.0F;
  const float maximum = static_cast<float>(config_.max_wheel_target);
  left_target_ =
      static_cast<int32_t>(std::lround(std::clamp(left_target, minimum, maximum)));
  right_target_ =
      static_cast<int32_t>(std::lround(std::clamp(right_target, minimum, maximum)));
}

}  // namespace App
