#pragma once

#include <cstdint>

#include "pid.hpp"

class MotionControl
{
 public:
  using PidParam = LibXR::PID<float>::Param;

  enum class DriveMode : uint8_t
  {
    STOPPED,
    MANUAL,
    LINE_FOLLOW,
    ANGLE_TURN,
  };

  struct Configuration
  {
    PidParam line_pid = {1.0F, 3.0F, 0.0F, 0.0F, 0.0F, 30.0F, false};
    PidParam angle_pid = {1.0F, 0.35F, 0.0F, 0.0F, 0.0F, 45.0F, false};
    int32_t default_base_target = 30;
    int32_t min_base_target = 10;
    int32_t max_base_target = 55;
    int32_t max_wheel_target = 60;
    float turn_tolerance_deg = 2.0F;
    uint16_t turn_timeout_outer_cycles = 250;
    uint8_t turn_stable_outer_cycles = 5;
    uint8_t outer_loop_divider = 2;
  };

  struct Inputs
  {
    bool line_detected;
    float line_position;
    float continuous_yaw_deg;
    bool attitude_valid;
  };

  struct Outputs
  {
    int32_t left_target;
    int32_t right_target;
  };

  MotionControl();
  explicit MotionControl(const Configuration& config);

  void HandleCommand(uint8_t command);
  bool StartRelativeTurn(float delta_deg);
  Outputs Step(const Inputs& inputs);

  DriveMode GetMode() const;
  float GetTurnTargetYawDeg() const;

 private:
  void SetMode(DriveMode mode);
  void ResetControllers();
  void UpdateOuterLoop(const Inputs& inputs);
  void UpdateLineTargets(const Inputs& inputs);
  void UpdateAngleTargets(const Inputs& inputs);
  void SetWheelTargets(float left_target, float right_target, bool allow_reverse);

  static constexpr float OUTER_DT_SECONDS = 0.020F;
  static constexpr float GREY_SENSOR_POSITION_SCALE = 1000.0F;
  static constexpr float MAX_RELATIVE_TURN_DEG = 180.0F;

  Configuration config_;
  LibXR::PID<float> line_pid_;
  LibXR::PID<float> angle_pid_;

  DriveMode mode_ = DriveMode::STOPPED;
  int8_t manual_left_ = 0;
  int8_t manual_right_ = 0;
  int32_t base_target_ = 30;
  int32_t left_target_ = 0;
  int32_t right_target_ = 0;

  float latest_continuous_yaw_deg_ = 0.0F;
  float turn_target_yaw_deg_ = 0.0F;
  bool latest_attitude_valid_ = false;
  bool outer_update_pending_ = false;
  uint8_t outer_loop_counter_ = 0;
  uint8_t turn_stable_cycles_ = 0;
  uint16_t turn_elapsed_cycles_ = 0;
};
