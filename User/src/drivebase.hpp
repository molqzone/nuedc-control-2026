#pragma once

#include <cstdint>

#include "app_framework.hpp"
#include "gpio.hpp"
#include "libxr_def.hpp"
#include "pwm.hpp"
#include "quadrature_encoder.hpp"
#include "ramfs.hpp"
#include "simple_chassis.hpp"
#include "simple_wheel.hpp"

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

  explicit Drivebase(LibXR::HardwareContainer& hw);
  Drivebase(LibXR::HardwareContainer& hw, const Configuration& config);

  void Stop();
  void RegisterCommands(LibXR::RamFS& ramfs);
  LibXR::ErrorCode SetWheelTargets(float left_target_delta, float right_target_delta,
                                   float dt_seconds);
  LibXR::ErrorCode SetOpenLoopDuty(float left_duty, float right_duty, float dt_seconds);
  const SimpleChassis::Feedback& GetFeedback() const;

 private:
  static SimpleWheel::DriverType ToWheelDriver(DriverType driver);
  static SimpleWheel::Configuration MakeWheelConfig(const Configuration& config,
                                                    bool left);
  static LibXR::GPIO& FindGpio(LibXR::HardwareContainer& hw, const char* alias);
  static LibXR::PWM& FindPwm(LibXR::HardwareContainer& hw, const char* alias);
  LibXR::ErrorCode Initialize();
  LibXR::ErrorCode EnablePwmOutputs();
  void DisablePwmOutputs();

  Configuration config_;
  LibXR::GPIO& motor_ain1_;
  LibXR::GPIO& motor_ain2_;
  LibXR::GPIO& motor_bin1_;
  LibXR::GPIO& motor_bin2_;
  LibXR::PWM& motor_a_pwm_;
  LibXR::PWM& motor_b_pwm_;
  LibXR::GPIO& encoder_1a_;
  LibXR::GPIO& encoder_1b_;
  LibXR::GPIO& encoder_2a_;
  LibXR::GPIO& encoder_2b_;
  QuadratureEncoder encoder_left_;
  QuadratureEncoder encoder_right_;
  SimpleWheel left_wheel_;
  SimpleWheel right_wheel_;
  SimpleChassis chassis_;
  bool pwm_enabled_ = false;
};
