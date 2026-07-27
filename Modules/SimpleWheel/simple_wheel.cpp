#include "simple_wheel.hpp"

#include <algorithm>
#include <cmath>

SimpleWheel::SimpleWheel(Hardware hardware)
    : SimpleWheel(hardware, Configuration{})
{
}

SimpleWheel::SimpleWheel(Hardware hardware, const Configuration& config)
    : hardware_(hardware), config_(config), speed_pid_(config.speed_pid)
{
}

LibXR::ErrorCode SimpleWheel::Initialize()
{
  constexpr LibXR::GPIO::Configuration OUTPUT_CONFIG = {
      LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::NONE};

  LibXR::ErrorCode result = hardware_.pin1.SetConfig(OUTPUT_CONFIG);
  if (result != LibXR::ErrorCode::OK)
  {
    return result;
  }
  result = hardware_.pin2.SetConfig(OUTPUT_CONFIG);
  if (result != LibXR::ErrorCode::OK)
  {
    return result;
  }
  hardware_.pin1.Write(false);
  hardware_.pin2.Write(false);

  result = hardware_.pwm.SetDutyCycle(0.0F);
  if (result != LibXR::ErrorCode::OK)
  {
    return result;
  }

  result = hardware_.encoder.Initialize();
  if (result != LibXR::ErrorCode::OK)
  {
    return result;
  }

  Disable();
  speed_pid_.Reset();
  feedback_ = {};
  current_direction_ = 0;
  return LibXR::ErrorCode::OK;
}

LibXR::ErrorCode SimpleWheel::Enable() { return hardware_.pwm.Enable(); }

void SimpleWheel::Disable()
{
  (void)hardware_.pwm.Disable();
  (void)hardware_.pwm.SetDutyCycle(0.0F);
  hardware_.pin1.Write(false);
  hardware_.pin2.Write(false);
  current_direction_ = 0;
}

void SimpleWheel::Relax()
{
  feedback_.delta = hardware_.encoder.TakeDelta();
  feedback_.total = hardware_.encoder.GetTotal();
  target_delta_ = 0.0F;
  open_loop_duty_ = 0.0F;
  closed_loop_ = true;
  speed_pid_.Reset();
  feedback_.target_delta = 0.0F;
  feedback_.duty = 0.0F;
  feedback_.closed_loop = true;
  (void)ApplyCommand(0.0F);
}

void SimpleWheel::SetTargetDelta(float target_delta)
{
  target_delta_ = target_delta;
  closed_loop_ = true;
  feedback_.target_delta = target_delta;
  feedback_.closed_loop = true;
}

void SimpleWheel::SetOpenLoopDuty(float duty)
{
  open_loop_duty_ = std::clamp(duty, -config_.max_duty, config_.max_duty);
  closed_loop_ = false;
  feedback_.target_delta = 0.0F;
  feedback_.closed_loop = false;
}

LibXR::ErrorCode SimpleWheel::Update(float dt_seconds)
{
  feedback_.delta = hardware_.encoder.TakeDelta();
  feedback_.total = hardware_.encoder.GetTotal();
  feedback_.target_delta = target_delta_;
  feedback_.closed_loop = closed_loop_;

  float duty = open_loop_duty_;
  if (closed_loop_)
  {
    if (target_delta_ == 0.0F)
    {
      speed_pid_.Reset();
      duty = 0.0F;
    }
    else
    {
      duty = speed_pid_.Calculate(target_delta_, static_cast<float>(feedback_.delta),
                                  dt_seconds);
    }
  }

  duty = std::clamp(duty, -config_.max_duty, config_.max_duty);
  duty = ApplyDeadband(duty);
  feedback_.duty = duty;

  const LibXR::ErrorCode result = ApplyCommand(duty);
  if (result != LibXR::ErrorCode::OK)
  {
    Disable();
    feedback_.duty = 0.0F;
  }
  return result;
}

const SimpleWheel::Feedback& SimpleWheel::GetFeedback() const { return feedback_; }

float SimpleWheel::GetTargetDelta() const { return target_delta_; }

bool SimpleWheel::HasActiveCommand() const
{
  return closed_loop_ ? target_delta_ != 0.0F : open_loop_duty_ != 0.0F;
}

int8_t SimpleWheel::DirectionOf(float command)
{
  return command > 0.0F ? 1 : (command < 0.0F ? -1 : 0);
}

float SimpleWheel::ApplyDeadband(float duty) const
{
  if (duty > 0.0F && duty < config_.duty_deadband)
  {
    return config_.duty_deadband;
  }
  if (duty < 0.0F && duty > -config_.duty_deadband)
  {
    return -config_.duty_deadband;
  }
  return duty;
}

LibXR::ErrorCode SimpleWheel::ApplyCommand(float command)
{
  command = std::clamp(command, -config_.max_duty, config_.max_duty);
  if (config_.driver == DriverType::DRV8701E)
  {
    return SetDRV8701E(command);
  }
  return SetTB6612(command);
}

LibXR::ErrorCode SimpleWheel::SetTB6612(float command)
{
  const int8_t requested_direction = DirectionOf(command);

  if (requested_direction == 0)
  {
    const LibXR::ErrorCode result = hardware_.pwm.SetDutyCycle(0.0F);
    hardware_.pin1.Write(false);
    hardware_.pin2.Write(false);
    current_direction_ = 0;
    return result;
  }

  if (requested_direction != current_direction_)
  {
    const LibXR::ErrorCode result = hardware_.pwm.SetDutyCycle(0.0F);
    if (result != LibXR::ErrorCode::OK)
    {
      hardware_.pin1.Write(false);
      hardware_.pin2.Write(false);
      current_direction_ = 0;
      return result;
    }

    bool forward = requested_direction > 0;
    if (config_.reversed)
    {
      forward = !forward;
    }
    hardware_.pin1.Write(!forward);
    hardware_.pin2.Write(forward);
    current_direction_ = requested_direction;
  }

  const LibXR::ErrorCode result = hardware_.pwm.SetDutyCycle(std::fabs(command));
  if (result != LibXR::ErrorCode::OK)
  {
    Disable();
  }
  return result;
}

LibXR::ErrorCode SimpleWheel::SetDRV8701E(float command)
{
  const int8_t requested_direction = DirectionOf(command);

  if (requested_direction == 0)
  {
    const LibXR::ErrorCode result = hardware_.pwm.SetDutyCycle(0.0F);
    current_direction_ = 0;
    return result;
  }

  if (requested_direction != current_direction_)
  {
    const LibXR::ErrorCode result = hardware_.pwm.SetDutyCycle(0.0F);
    if (result != LibXR::ErrorCode::OK)
    {
      current_direction_ = 0;
      return result;
    }

    bool forward = requested_direction > 0;
    if (config_.reversed)
    {
      forward = !forward;
    }
    hardware_.pin2.Write(forward);
    current_direction_ = requested_direction;
  }

  const LibXR::ErrorCode result = hardware_.pwm.SetDutyCycle(std::fabs(command));
  if (result != LibXR::ErrorCode::OK)
  {
    Disable();
  }
  return result;
}
