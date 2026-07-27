#pragma once

#include "libxr_def.hpp"
#include "ramfs.hpp"
#include "simple_wheel.hpp"

class SimpleChassis
{
 public:
  struct Feedback
  {
    SimpleWheel::Feedback left;
    SimpleWheel::Feedback right;
  };

  struct CommandInterface
  {
    void* context = nullptr;
    LibXR::ErrorCode (*set_targets)(void* context, float left, float right,
                                    float dt_seconds) = nullptr;
    LibXR::ErrorCode (*set_duty)(void* context, float left, float right,
                                 float dt_seconds) = nullptr;
    void (*stop)(void* context) = nullptr;
  };

  SimpleChassis(SimpleWheel& left, SimpleWheel& right);

  LibXR::ErrorCode Initialize();
  LibXR::ErrorCode Enable();
  void Disable();
  void Relax();
  void RegisterCommands(LibXR::RamFS& ramfs);
  void RegisterCommands(LibXR::RamFS& ramfs, CommandInterface command_interface);

  void SetWheelTargets(float left_target_delta, float right_target_delta);
  void SetOpenLoopDuty(float left_duty, float right_duty);
  LibXR::ErrorCode Update(float dt_seconds);

  const Feedback& GetFeedback() const;
  bool HasActiveCommand() const;

 private:
  static constexpr float COMMAND_DEFAULT_DT_SECONDS = 0.010F;

  static int CommandThunk(SimpleChassis* self, int argc, char** argv);
  static bool ParseFloat(const char* text, float& value);
  static void PrintUsage();

  int HandleCommand(int argc, char** argv);
  LibXR::ErrorCode CommandSetTargets(float left, float right, float dt_seconds);
  LibXR::ErrorCode CommandSetDuty(float left, float right, float dt_seconds);
  void CommandStop();
  void PrintFeedback() const;

  SimpleWheel& left_;
  SimpleWheel& right_;
  Feedback feedback_{};
  CommandInterface command_interface_{};
  LibXR::RamFS::File* command_file_ = nullptr;
};
