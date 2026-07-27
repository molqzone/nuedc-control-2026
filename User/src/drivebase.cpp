#include "drivebase.hpp"

#include "ti_msp_dl_config.h"

Drivebase::Drivebase(LibXR::HardwareContainer& hw) : Drivebase(hw, Configuration{}) {}

Drivebase::Drivebase(LibXR::HardwareContainer& hw, const Configuration& config)
    : config_(config),
      motor_ain1_(FindGpio(hw, "motor_ain1")),
      motor_ain2_(FindGpio(hw, "motor_ain2")),
      motor_bin1_(FindGpio(hw, "motor_bin1")),
      motor_bin2_(FindGpio(hw, "motor_bin2")),
      motor_a_pwm_(FindPwm(hw, "motor_a_pwm")),
      motor_b_pwm_(FindPwm(hw, "motor_b_pwm")),
      encoder_1a_(FindGpio(hw, "encoder_1a")),
      encoder_1b_(FindGpio(hw, "encoder_1b")),
      encoder_2a_(FindGpio(hw, "encoder_2a")),
      encoder_2b_(FindGpio(hw, "encoder_2b")),
      encoder_left_(encoder_1a_, encoder_1b_, config.left_encoder_reversed),
      encoder_right_(encoder_2a_, encoder_2b_, config.right_encoder_reversed),
      left_wheel_({motor_a_pwm_, motor_ain1_, motor_ain2_, encoder_left_},
                  MakeWheelConfig(config, true)),
      right_wheel_({motor_b_pwm_, motor_bin1_, motor_bin2_, encoder_right_},
                   MakeWheelConfig(config, false)),
      chassis_(left_wheel_, right_wheel_)
{
  ASSERT(Initialize() == LibXR::ErrorCode::OK);
}

void Drivebase::Stop()
{
  chassis_.Relax();
  DisablePwmOutputs();
}

LibXR::ErrorCode Drivebase::SetWheelTargets(float left_target_delta,
                                            float right_target_delta, float dt_seconds)
{
  chassis_.SetWheelTargets(left_target_delta, right_target_delta);
  const LibXR::ErrorCode result = chassis_.Update(dt_seconds);
  if (result != LibXR::ErrorCode::OK)
  {
    Stop();
    return result;
  }

  if (!chassis_.HasActiveCommand())
  {
    DisablePwmOutputs();
    return LibXR::ErrorCode::OK;
  }
  return EnablePwmOutputs();
}

LibXR::ErrorCode Drivebase::SetOpenLoopDuty(float left_duty, float right_duty,
                                            float dt_seconds)
{
  chassis_.SetOpenLoopDuty(left_duty, right_duty);
  const LibXR::ErrorCode result = chassis_.Update(dt_seconds);
  if (result != LibXR::ErrorCode::OK)
  {
    Stop();
    return result;
  }

  if (!chassis_.HasActiveCommand())
  {
    DisablePwmOutputs();
    return LibXR::ErrorCode::OK;
  }
  return EnablePwmOutputs();
}

const SimpleChassis::Feedback& Drivebase::GetFeedback() const
{
  return chassis_.GetFeedback();
}

void Drivebase::RegisterCommands(LibXR::RamFS& ramfs)
{
  chassis_.RegisterCommands(
      ramfs,
      SimpleChassis::CommandInterface{
          this,
          [](void* context, float left, float right, float dt_seconds)
          {
            return static_cast<Drivebase*>(context)->SetWheelTargets(left, right,
                                                                     dt_seconds);
          },
          [](void* context, float left, float right, float dt_seconds)
          {
            return static_cast<Drivebase*>(context)->SetOpenLoopDuty(left, right,
                                                                     dt_seconds);
          },
          [](void* context) { static_cast<Drivebase*>(context)->Stop(); }});
}

SimpleWheel::DriverType Drivebase::ToWheelDriver(DriverType driver)
{
  switch (driver)
  {
    case DriverType::TB6612:
      return SimpleWheel::DriverType::TB6612;
    case DriverType::DRV8701E:
    default:
      return SimpleWheel::DriverType::DRV8701E;
  }
}

SimpleWheel::Configuration Drivebase::MakeWheelConfig(const Configuration& config,
                                                      bool left)
{
  SimpleWheel::Configuration wheel_config;
  wheel_config.driver = ToWheelDriver(config.driver);
  wheel_config.reversed = left ? config.left_reversed : config.right_reversed;
  wheel_config.duty_deadband = config.duty_deadband;
  wheel_config.max_duty = config.max_duty;
  wheel_config.speed_pid = left ? config.left_speed_pid : config.right_speed_pid;
  return wheel_config;
}

LibXR::GPIO& Drivebase::FindGpio(LibXR::HardwareContainer& hw, const char* alias)
{
  return *hw.FindOrExit<LibXR::GPIO>({alias});
}

LibXR::PWM& Drivebase::FindPwm(LibXR::HardwareContainer& hw, const char* alias)
{
  return *hw.FindOrExit<LibXR::PWM>({alias});
}

LibXR::ErrorCode Drivebase::Initialize()
{
  const LibXR::ErrorCode result = chassis_.Initialize();
  if (result != LibXR::ErrorCode::OK)
  {
    return result;
  }

  DL_TimerA_setCCPOutputDisabledAdv(
      MOTOR_PWM_INST,
      DL_TIMERA_CCP2_DIS_OUT_ADV_FORCE_LOW | DL_TIMERA_CCP3_DIS_OUT_ADV_FORCE_LOW);
  DisablePwmOutputs();
  return LibXR::ErrorCode::OK;
}

LibXR::ErrorCode Drivebase::EnablePwmOutputs()
{
  if (pwm_enabled_)
  {
    return LibXR::ErrorCode::OK;
  }

  DL_TimerA_setCCPOutputDisabledAdv(
      MOTOR_PWM_INST,
      DL_TIMERA_CCP2_DIS_OUT_ADV_SET_BY_OCTL | DL_TIMERA_CCP3_DIS_OUT_ADV_SET_BY_OCTL);
  const LibXR::ErrorCode result = chassis_.Enable();
  if (result != LibXR::ErrorCode::OK)
  {
    DisablePwmOutputs();
    return result;
  }
  pwm_enabled_ = true;
  return LibXR::ErrorCode::OK;
}

void Drivebase::DisablePwmOutputs()
{
  chassis_.Disable();
  DL_TimerA_setCCPOutputDisabledAdv(
      MOTOR_PWM_INST,
      DL_TIMERA_CCP2_DIS_OUT_ADV_FORCE_LOW | DL_TIMERA_CCP3_DIS_OUT_ADV_FORCE_LOW);
  pwm_enabled_ = false;
}
