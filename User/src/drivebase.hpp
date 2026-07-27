#pragma once

#include <cstdint>

#include "libxr_def.hpp"
#include "mspm0_gpio.hpp"
#include "mspm0_pwm.hpp"
#include "quadrature_encoder.hpp"
#include "simple_chassis.hpp"
#include "simple_wheel.hpp"

namespace App
{

class Drivebase
{
 public:
  enum class DriverType : uint8_t
  {
    TB6612,
    DRV8701E,
  };

  struct Configuration
  {
    DriverType driver = DriverType::DRV8701E;
    bool left_reversed = false;
    bool right_reversed = false;
    bool left_encoder_reversed = false;
    bool right_encoder_reversed = true;
    float duty_deadband = 0.08F;
    float max_duty = 0.90F;
    LibXR::PID<float>::Param left_speed_pid = {1.0F, 0.0125F, 0.0F, 0.0F,
                                               0.0F, 0.90F,   false};
    LibXR::PID<float>::Param right_speed_pid = {1.0F, 0.0125F, 0.0F, 0.0F,
                                                0.0F, 0.90F,   false};
  };

  Drivebase();
  explicit Drivebase(const Configuration& config);

  void Stop();
  LibXR::ErrorCode SetWheelTargets(float left_target_delta, float right_target_delta,
                                   float dt_seconds);
  LibXR::ErrorCode SetOpenLoopDuty(float left_duty, float right_duty, float dt_seconds);
  const SimpleChassis::Feedback& GetFeedback() const;

 private:
  static SimpleWheel::DriverType ToWheelDriver(DriverType driver);
  static SimpleWheel::Configuration MakeWheelConfig(const Configuration& config,
                                                    bool left);
  LibXR::ErrorCode Initialize();
  LibXR::ErrorCode EnablePwmOutputs();
  void DisablePwmOutputs();

  Configuration config_;
  LibXR::MSPM0GPIO motor_ain1_;
  LibXR::MSPM0GPIO motor_ain2_;
  LibXR::MSPM0GPIO motor_bin1_;
  LibXR::MSPM0GPIO motor_bin2_;
  LibXR::MSPM0PWM motor_a_pwm_;
  LibXR::MSPM0PWM motor_b_pwm_;
  LibXR::MSPM0GPIO encoder_1a_;
  LibXR::MSPM0GPIO encoder_1b_;
  LibXR::MSPM0GPIO encoder_2a_;
  LibXR::MSPM0GPIO encoder_2b_;
  QuadratureEncoder encoder_left_;
  QuadratureEncoder encoder_right_;
  SimpleWheel left_wheel_;
  SimpleWheel right_wheel_;
  SimpleChassis chassis_;
  bool pwm_enabled_ = false;
};

}  // namespace App
