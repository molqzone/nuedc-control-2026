#include "simple_chassis.hpp"

namespace App
{

SimpleChassis::SimpleChassis(SimpleWheel& left, SimpleWheel& right)
    : left_(left), right_(right)
{
}

LibXR::ErrorCode SimpleChassis::Initialize()
{
  const LibXR::ErrorCode left_result = left_.Initialize();
  if (left_result != LibXR::ErrorCode::OK)
  {
    return left_result;
  }
  return right_.Initialize();
}

LibXR::ErrorCode SimpleChassis::Enable()
{
  const LibXR::ErrorCode left_result = left_.Enable();
  const LibXR::ErrorCode right_result = right_.Enable();
  return left_result != LibXR::ErrorCode::OK ? left_result : right_result;
}

void SimpleChassis::Disable()
{
  left_.Disable();
  right_.Disable();
}

void SimpleChassis::Relax()
{
  left_.Relax();
  right_.Relax();
  feedback_.left = left_.GetFeedback();
  feedback_.right = right_.GetFeedback();
}

void SimpleChassis::SetWheelTargets(float left_target_delta,
                                    float right_target_delta)
{
  left_.SetTargetDelta(left_target_delta);
  right_.SetTargetDelta(right_target_delta);
}

void SimpleChassis::SetOpenLoopDuty(float left_duty, float right_duty)
{
  left_.SetOpenLoopDuty(left_duty);
  right_.SetOpenLoopDuty(right_duty);
}

LibXR::ErrorCode SimpleChassis::Update(float dt_seconds)
{
  const LibXR::ErrorCode left_result = left_.Update(dt_seconds);
  const LibXR::ErrorCode right_result = right_.Update(dt_seconds);
  feedback_.left = left_.GetFeedback();
  feedback_.right = right_.GetFeedback();
  return left_result != LibXR::ErrorCode::OK ? left_result : right_result;
}

const SimpleChassis::Feedback& SimpleChassis::GetFeedback() const
{
  return feedback_;
}

bool SimpleChassis::HasActiveCommand() const
{
  return left_.HasActiveCommand() || right_.HasActiveCommand();
}

}  // namespace App
