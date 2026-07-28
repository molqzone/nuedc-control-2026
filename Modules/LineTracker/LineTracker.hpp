#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Converts grey sensor line samples into primitive differential chassis wheel commands while line following is active.
constructor_args:
  - line_sample_topic_name: line_sensors
  - chassis_command_topic_name: chassis_command
  - control_period_ms: 10
  - line_timeout_ms: 100
  - line_pid:
      k: 1.0
      p: 3.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 30.0
      cycle: false
  - max_wheel_target: 60
  - outer_loop_divider: 2
  - max_control_dt_ms: 100
template_args: []
required_hardware: []
depends: [DifferentialChassis, GreySensor]
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "DifferentialChassis.hpp"
#include "GreySensor.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "message.hpp"
#include "pid.hpp"

class LineTracker : public LibXR::Application
{
 public:
  using PidParam = LibXR::PID<float>::Param;
  using Command = DifferentialChassis::Command;
  using CommandMode = DifferentialChassis::CommandMode;

  LineTracker(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
              const char* line_sample_topic_name,
              const char* chassis_command_topic_name,
              uint32_t control_period_ms, uint32_t line_timeout_ms,
              PidParam line_pid, int32_t max_wheel_target,
              uint8_t outer_loop_divider,
              uint32_t max_control_dt_ms)
      : control_period_ms_(control_period_ms),
        line_timeout_ms_(line_timeout_ms),
        max_wheel_target_(max_wheel_target),
        outer_loop_divider_(outer_loop_divider),
        max_control_dt_ms_(max_control_dt_ms),
        line_pid_(line_pid),
        line_topic_(LibXR::Topic::CreateTopic<GreySensor::Sample>(
            line_sample_topic_name)),
        line_sub_(line_topic_),
        command_topic_(LibXR::Topic::CreateTopic<Command>(chassis_command_topic_name))
  {
    UNUSED(hw);
    ASSERT(line_sample_topic_name != nullptr && line_sample_topic_name[0] != '\0');
    ASSERT(chassis_command_topic_name != nullptr &&
           chassis_command_topic_name[0] != '\0');
    ASSERT(control_period_ms_ > 0U);
    ASSERT(max_control_dt_ms_ >= control_period_ms_);
    ASSERT(max_wheel_target_ >= 0);
    ASSERT(outer_loop_divider_ > 0U);
    line_sub_.StartWaiting();
    last_control_ms_ = LibXR::Timebase::GetMilliseconds();
    last_control_timestamp_ = LibXR::Timebase::GetMicroseconds();
    app.Register(*this);
  }

  void Start(int32_t base_target)
  {
    SetBaseTarget(base_target);
    if (!active_)
    {
      ResetControlState();
      active_ = true;
    }
    outer_update_pending_ = true;
  }

  void Stop()
  {
    active_ = false;
    ResetControlState();
  }

  void SetBaseTarget(int32_t base_target)
  {
    base_target_ = static_cast<int32_t>(
        std::clamp(base_target, static_cast<int32_t>(0), max_wheel_target_));
    outer_update_pending_ = true;
  }

  bool IsActive() const { return active_; }

  int32_t GetBaseTarget() const { return base_target_; }

  bool HasLineSample() const { return has_line_sample_; }

  const GreySensor::Sample& GetLastSample() const { return latest_line_sample_; }

  void OnMonitor() override
  {
    RefreshLineSample();
    if (!active_)
    {
      return;
    }

    const uint32_t now_ms = LibXR::Timebase::GetMilliseconds();
    if (has_published_ &&
        static_cast<uint32_t>(now_ms - last_control_ms_) < control_period_ms_)
    {
      return;
    }

    const float control_dt_seconds =
        MeasureControlDtSeconds(LibXR::Timebase::GetMicroseconds());
    has_published_ = true;
    last_control_ms_ = now_ms;

    const bool line_valid = IsLineValid(now_ms);
    if (!line_valid)
    {
      line_pid_.Reset();
      left_target_ = 0;
      right_target_ = 0;
      outer_dt_seconds_ = 0.0F;
      outer_loop_counter_ = 0;
      PublishCommand(CommandMode::STOP, 0.0F, 0.0F);
      return;
    }

    outer_dt_seconds_ += control_dt_seconds;
    outer_loop_counter_++;
    if (outer_update_pending_ || outer_loop_counter_ >= outer_loop_divider_)
    {
      const float outer_dt_seconds = outer_dt_seconds_;
      outer_dt_seconds_ = 0.0F;
      outer_loop_counter_ = 0;
      outer_update_pending_ = false;
      UpdateLineTargets(static_cast<float>(latest_line_sample_.position),
                        outer_dt_seconds);
    }

    PublishCommand(CommandMode::WHEEL_TARGET_DELTA, static_cast<float>(left_target_),
                   static_cast<float>(right_target_));
  }

 private:
  bool IsLineValid(uint32_t now_ms) const
  {
    if (!has_line_sample_ || latest_line_sample_.line_detected == 0U)
    {
      return false;
    }
    return line_timeout_ms_ == 0U ||
           static_cast<uint32_t>(now_ms - last_line_sample_ms_) <=
               line_timeout_ms_;
  }

  float MeasureControlDtSeconds(LibXR::MicrosecondTimestamp now)
  {
    float dt_seconds = NominalControlDtSeconds();
    if (has_published_)
    {
      dt_seconds = (now - last_control_timestamp_).ToSecondf();
    }
    last_control_timestamp_ = now;
    return SanitizeControlDtSeconds(dt_seconds);
  }

  float NominalControlDtSeconds() const
  {
    return static_cast<float>(control_period_ms_) * 0.001F;
  }

  float SanitizeControlDtSeconds(float dt_seconds) const
  {
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0F)
    {
      return NominalControlDtSeconds();
    }

    return std::min(dt_seconds, static_cast<float>(max_control_dt_ms_) * 0.001F);
  }

  void RefreshLineSample()
  {
    if (!line_sub_.Available())
    {
      return;
    }

    latest_line_sample_ = line_sub_.GetData();
    line_sub_.StartWaiting();
    has_line_sample_ = true;
    last_line_sample_ms_ = LibXR::Timebase::GetMilliseconds();
  }

  void UpdateLineTargets(float line_position, float dt_seconds)
  {
    if (!std::isfinite(line_position))
    {
      line_pid_.Reset();
      left_target_ = 0;
      right_target_ = 0;
      return;
    }

    const float line_error =
        line_position / static_cast<float>(GreySensor::POSITION_SCALE);
    const float correction = line_pid_.Calculate(0.0F, line_error, dt_seconds);
    SetWheelTargets(static_cast<float>(base_target_) - correction,
                    static_cast<float>(base_target_) + correction);
  }

  void SetWheelTargets(float left_target, float right_target)
  {
    left_target_ = static_cast<int32_t>(std::lround(std::clamp(
        left_target, 0.0F, static_cast<float>(max_wheel_target_))));
    right_target_ = static_cast<int32_t>(std::lround(std::clamp(
        right_target, 0.0F, static_cast<float>(max_wheel_target_))));
  }

  void ResetControlState()
  {
    line_pid_.Reset();
    left_target_ = 0;
    right_target_ = 0;
    outer_dt_seconds_ = 0.0F;
    outer_loop_counter_ = 0;
    outer_update_pending_ = false;
    has_published_ = false;
  }

  void PublishCommand(CommandMode mode, float left_target, float right_target)
  {
    command_.mode = mode;
    command_.left_target = left_target;
    command_.right_target = right_target;
    command_.sequence = command_sequence_++;
    command_topic_.Publish(command_);
  }

  uint32_t control_period_ms_;
  uint32_t line_timeout_ms_;
  int32_t max_wheel_target_;
  uint8_t outer_loop_divider_;
  uint32_t max_control_dt_ms_;
  LibXR::PID<float> line_pid_;
  LibXR::Topic line_topic_;
  LibXR::Topic::ASyncSubscriber<GreySensor::Sample> line_sub_;
  LibXR::Topic command_topic_;
  GreySensor::Sample latest_line_sample_{};
  Command command_{};
  int32_t base_target_ = 0;
  int32_t left_target_ = 0;
  int32_t right_target_ = 0;
  float outer_dt_seconds_ = 0.0F;
  bool active_ = false;
  bool outer_update_pending_ = false;
  bool has_line_sample_ = false;
  bool has_published_ = false;
  uint8_t outer_loop_counter_ = 0;
  uint32_t last_line_sample_ms_ = 0;
  uint32_t last_control_ms_ = 0;
  LibXR::MicrosecondTimestamp last_control_timestamp_{};
  uint32_t command_sequence_ = 0;
};
