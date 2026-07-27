#pragma once

#include "libxr_def.hpp"
#include "simple_wheel.hpp"

namespace App
{

class SimpleChassis
{
 public:
  struct Feedback
  {
    SimpleWheel::Feedback left;
    SimpleWheel::Feedback right;
  };

  SimpleChassis(SimpleWheel& left, SimpleWheel& right);

  LibXR::ErrorCode Initialize();
  LibXR::ErrorCode Enable();
  void Disable();
  void Relax();

  void SetWheelTargets(float left_target_delta, float right_target_delta);
  void SetOpenLoopDuty(float left_duty, float right_duty);
  LibXR::ErrorCode Update(float dt_seconds);

  const Feedback& GetFeedback() const;
  bool HasActiveCommand() const;

 private:
  SimpleWheel& left_;
  SimpleWheel& right_;
  Feedback feedback_{};
};

}  // namespace App
