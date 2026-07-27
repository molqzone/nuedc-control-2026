#include "simple_chassis.hpp"

#include <cmath>
#include <cstring>

#include "libxr.hpp"

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

void SimpleChassis::RegisterCommands(LibXR::RamFS& ramfs)
{
  RegisterCommands(ramfs, CommandInterface{});
}

void SimpleChassis::RegisterCommands(LibXR::RamFS& ramfs,
                                     CommandInterface command_interface)
{
  command_interface_ = command_interface;
  if (command_file_ != nullptr)
  {
    return;
  }

  command_file_ = new LibXR::RamFS::File(
      LibXR::RamFS::CreateCommand<SimpleChassis*>("chassis", CommandThunk, this));
  ramfs.Add(*command_file_);
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

int SimpleChassis::CommandThunk(SimpleChassis* self, int argc, char** argv)
{
  if (self == nullptr)
  {
    return -1;
  }
  return self->HandleCommand(argc, argv);
}

bool SimpleChassis::ParseFloat(const char* text, float& value)
{
  if (text == nullptr || *text == '\0')
  {
    return false;
  }

  uint32_t index = 0;
  float sign = 1.0F;
  if (text[index] == '+' || text[index] == '-')
  {
    sign = text[index] == '-' ? -1.0F : 1.0F;
    index++;
  }

  bool has_digit = false;
  bool has_decimal_point = false;
  float result = 0.0F;
  float decimal_scale = 0.1F;
  for (; text[index] != '\0'; index++)
  {
    const char character = text[index];
    if (character >= '0' && character <= '9')
    {
      has_digit = true;
      const float digit = static_cast<float>(character - '0');
      if (has_decimal_point)
      {
        result += digit * decimal_scale;
        decimal_scale *= 0.1F;
      }
      else
      {
        result = result * 10.0F + digit;
      }
      continue;
    }
    if (character == '.' && !has_decimal_point)
    {
      has_decimal_point = true;
      continue;
    }
    return false;
  }

  value = result * sign;
  return has_digit && std::isfinite(value);
}

void SimpleChassis::PrintUsage()
{
  LibXR::STDIO::Printf<
      "Usage: chassis feedback|relax|stop|enable|disable|target <left> <right> "
      "[dt_ms]|duty <left> <right> [dt_ms]\r\n">();
}

int SimpleChassis::HandleCommand(int argc, char** argv)
{
  if (argc < 2 || argv == nullptr)
  {
    PrintUsage();
    return 0;
  }

  if (std::strcmp(argv[1], "feedback") == 0 || std::strcmp(argv[1], "fb") == 0)
  {
    PrintFeedback();
    return 0;
  }
  if (std::strcmp(argv[1], "relax") == 0)
  {
    Relax();
    LibXR::STDIO::Printf<"chassis relaxed\r\n">();
    return 0;
  }
  if (std::strcmp(argv[1], "stop") == 0)
  {
    CommandStop();
    LibXR::STDIO::Printf<"chassis stopped\r\n">();
    return 0;
  }
  if (std::strcmp(argv[1], "enable") == 0)
  {
    const LibXR::ErrorCode result = Enable();
    LibXR::STDIO::Printf<"chassis enable: %d\r\n">(static_cast<int>(result));
    return result == LibXR::ErrorCode::OK ? 0 : -1;
  }
  if (std::strcmp(argv[1], "disable") == 0)
  {
    Disable();
    LibXR::STDIO::Printf<"chassis disabled\r\n">();
    return 0;
  }

  const bool is_target = std::strcmp(argv[1], "target") == 0;
  const bool is_duty = std::strcmp(argv[1], "duty") == 0;
  if (!is_target && !is_duty)
  {
    PrintUsage();
    return -1;
  }
  if (argc < 4 || argc > 5)
  {
    PrintUsage();
    return -1;
  }

  float left = 0.0F;
  float right = 0.0F;
  float dt_seconds = COMMAND_DEFAULT_DT_SECONDS;
  if (!ParseFloat(argv[2], left) || !ParseFloat(argv[3], right))
  {
    LibXR::STDIO::Printf<"chassis: invalid numeric argument\r\n">();
    return -1;
  }
  if (argc == 5)
  {
    float dt_ms = 0.0F;
    if (!ParseFloat(argv[4], dt_ms) || dt_ms <= 0.0F)
    {
      LibXR::STDIO::Printf<"chassis: invalid dt_ms\r\n">();
      return -1;
    }
    dt_seconds = dt_ms * 0.001F;
  }

  const LibXR::ErrorCode result =
      is_target ? CommandSetTargets(left, right, dt_seconds)
                : CommandSetDuty(left, right, dt_seconds);
  LibXR::STDIO::Printf<"chassis %s: %d\r\n">(argv[1], static_cast<int>(result));
  return result == LibXR::ErrorCode::OK ? 0 : -1;
}

LibXR::ErrorCode SimpleChassis::CommandSetTargets(float left, float right,
                                                  float dt_seconds)
{
  if (command_interface_.set_targets != nullptr)
  {
    return command_interface_.set_targets(command_interface_.context, left, right,
                                          dt_seconds);
  }

  SetWheelTargets(left, right);
  return Update(dt_seconds);
}

LibXR::ErrorCode SimpleChassis::CommandSetDuty(float left, float right,
                                               float dt_seconds)
{
  if (command_interface_.set_duty != nullptr)
  {
    return command_interface_.set_duty(command_interface_.context, left, right,
                                       dt_seconds);
  }

  SetOpenLoopDuty(left, right);
  return Update(dt_seconds);
}

void SimpleChassis::CommandStop()
{
  if (command_interface_.stop != nullptr)
  {
    command_interface_.stop(command_interface_.context);
    return;
  }
  Relax();
}

void SimpleChassis::PrintFeedback() const
{
  LibXR::STDIO::Printf<
      "left: delta=%d total=%d target=%.2f duty=%.3f mode=%s\r\n"
      "right: delta=%d total=%d target=%.2f duty=%.3f mode=%s\r\n">(
      static_cast<int>(feedback_.left.delta), static_cast<int>(feedback_.left.total),
      static_cast<double>(feedback_.left.target_delta),
      static_cast<double>(feedback_.left.duty),
      feedback_.left.closed_loop ? "closed" : "open",
      static_cast<int>(feedback_.right.delta), static_cast<int>(feedback_.right.total),
      static_cast<double>(feedback_.right.target_delta),
      static_cast<double>(feedback_.right.duty),
      feedback_.right.closed_loop ? "closed" : "open");
}
