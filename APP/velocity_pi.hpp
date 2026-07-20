#pragma once

#include <cstdint>

namespace App
{

class VelocityPI
{
 public:
  struct Configuration
  {
    float kp;
    float ki;
    float max_duty;
  };

  explicit VelocityPI(Configuration config) : config_(config) {}

  float Update(int32_t target_count, int32_t measured_count)
  {
    if (target_count == 0)
    {
      Reset();
      return 0.0F;
    }

    const int8_t direction = target_count > 0 ? 1 : -1;
    if (direction != last_direction_)
    {
      integral_ = 0.0F;
      last_direction_ = direction;
    }

    const float target = static_cast<float>(Abs(target_count));
    const float measured = static_cast<float>(Abs(measured_count));
    const float error = target - measured;
    const float candidate_integral = integral_ + error;
    float output = config_.kp * error + config_.ki * candidate_integral;

    if (output > config_.max_duty)
    {
      output = config_.max_duty;
      if (error < 0.0F)
      {
        integral_ = candidate_integral;
      }
    }
    else if (output < 0.0F)
    {
      output = 0.0F;
      if (error > 0.0F)
      {
        integral_ = candidate_integral;
      }
    }
    else
    {
      integral_ = candidate_integral;
    }

    last_output_ = output * static_cast<float>(direction);
    return last_output_;
  }

  void Reset()
  {
    integral_ = 0.0F;
    last_output_ = 0.0F;
    last_direction_ = 0;
  }

  float GetLastOutput() const { return last_output_; }

 private:
  static uint32_t Abs(int32_t value)
  {
    return value < 0 ? static_cast<uint32_t>(-(value + 1)) + 1U
                     : static_cast<uint32_t>(value);
  }

  Configuration config_;
  float integral_ = 0.0F;
  float last_output_ = 0.0F;
  int8_t last_direction_ = 0;
};

}  // namespace App
