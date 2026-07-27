#pragma once

#include <cstdint>

#include "gpio.hpp"
#include "pid.hpp"
#include "pwm.hpp"
#include "quadrature_encoder.hpp"

class SimpleWheel
{
 public:
  enum class DriverType : uint8_t
  {
    TB6612,
    DRV8701E,
  };

  struct Hardware
  {
    LibXR::PWM& pwm;
    LibXR::GPIO& pin1;
    LibXR::GPIO& pin2;
    QuadratureEncoder& encoder;
  };

  struct Configuration
  {
    DriverType driver = DriverType::DRV8701E;
    bool reversed = false;
    float duty_deadband = 0.08F;
    float max_duty = 0.90F;
    LibXR::PID<float>::Param speed_pid = {1.0F, 0.0125F, 0.0F, 0.0F,
                                          0.0F, 0.90F,   false};
  };

  struct Feedback
  {
    int32_t delta = 0;
    int32_t total = 0;
    float target_delta = 0.0F;
    float duty = 0.0F;
    bool closed_loop = true;
  };

  explicit SimpleWheel(Hardware hardware);
  SimpleWheel(Hardware hardware, const Configuration& config);

  LibXR::ErrorCode Initialize();
  LibXR::ErrorCode Enable();
  void Disable();
  void Relax();

  void SetTargetDelta(float target_delta);
  void SetOpenLoopDuty(float duty);
  LibXR::ErrorCode Update(float dt_seconds);

  const Feedback& GetFeedback() const;
  float GetTargetDelta() const;
  bool HasActiveCommand() const;

 private:
  static int8_t DirectionOf(float command);

  float ApplyDeadband(float duty) const;
  LibXR::ErrorCode ApplyCommand(float command);
  LibXR::ErrorCode SetTB6612(float command);
  LibXR::ErrorCode SetDRV8701E(float command);

  Hardware hardware_;
  Configuration config_;
  LibXR::PID<float> speed_pid_;
  Feedback feedback_{};
  float target_delta_ = 0.0F;
  float open_loop_duty_ = 0.0F;
  bool closed_loop_ = true;
  int8_t current_direction_ = 0;
};
