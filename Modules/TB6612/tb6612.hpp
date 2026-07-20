#pragma once

#include <algorithm>
#include <cstdint>

#include "gpio.hpp"
#include "libxr_def.hpp"
#include "pwm.hpp"

class TB6612Motor
{
 public:
  TB6612Motor(LibXR::PWM& pwm, LibXR::GPIO& in1, LibXR::GPIO& in2,
              bool reversed = false)
      : pwm_(pwm), in1_(in1), in2_(in2), reversed_(reversed)
  {
    ASSERT(Initialize() == LibXR::ErrorCode::OK);
  }

  LibXR::ErrorCode Set(float command)
  {
    command = std::clamp(command, -1.0F, 1.0F);
    const int8_t requested_direction =
        command > 0.0F ? 1 : (command < 0.0F ? -1 : 0);

    if (requested_direction == 0)
    {
      const LibXR::ErrorCode result = pwm_.SetDutyCycle(0.0F);
      in1_.Write(false);
      in2_.Write(false);
      current_direction_ = 0;
      return result;
    }

    if (requested_direction != current_direction_)
    {
      const LibXR::ErrorCode result = pwm_.SetDutyCycle(0.0F);
      if (result != LibXR::ErrorCode::OK)
      {
        in1_.Write(false);
        in2_.Write(false);
        current_direction_ = 0;
        return result;
      }

      bool forward = requested_direction > 0;
      if (reversed_)
      {
        forward = !forward;
      }
      in1_.Write(!forward);
      in2_.Write(forward);
      current_direction_ = requested_direction;
    }

    const float duty_cycle = command > 0.0F ? command : -command;
    return pwm_.SetDutyCycle(duty_cycle);
  }

  LibXR::ErrorCode Stop() { return Set(0.0F); }

 private:
  LibXR::ErrorCode Initialize()
  {
    constexpr LibXR::GPIO::Configuration OUTPUT_CONFIG = {
        LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::NONE};

    LibXR::ErrorCode result = in1_.SetConfig(OUTPUT_CONFIG);
    if (result != LibXR::ErrorCode::OK)
    {
      return result;
    }

    result = in2_.SetConfig(OUTPUT_CONFIG);
    if (result != LibXR::ErrorCode::OK)
    {
      return result;
    }

    in1_.Write(false);
    in2_.Write(false);

    result = pwm_.SetDutyCycle(0.0F);
    if (result != LibXR::ErrorCode::OK)
    {
      return result;
    }

    result = pwm_.Enable();
    if (result != LibXR::ErrorCode::OK)
    {
      return result;
    }

    return LibXR::ErrorCode::OK;
  }

  LibXR::PWM& pwm_;
  LibXR::GPIO& in1_;
  LibXR::GPIO& in2_;
  bool reversed_;
  int8_t current_direction_ = 0;
};
