#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Hardcoded high-level decision state machine for the 100 cm square counter-clockwise line-following course.
constructor_args:
  - line_tracker: "@line_tracker"
  - chassis: "@chassis"
  - auto_start: true
  - line_base_target: 30
  - target_laps: 1
  - outer_square_side_mm: 1000.0
  - line_width_mm: 18.0
  - line_width_tolerance_mm: 2.0
  - wheel_diameter_mm: 65.0
  - encoder_counts_per_revolution: 1560.0
template_args: []
required_hardware: ramfs
depends: [DifferentialChassis, LineTracker]
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <cstdint>
#include <cstring>

#include "DifferentialChassis.hpp"
#include "LineTracker.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "ramfs.hpp"

class SimpleDicision : public LibXR::Application
{
 public:
  enum class State : uint8_t
  {
    STOPPED,
    LINE_FOLLOWING,
    FINISHED,
  };

  SimpleDicision(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                 LineTracker& line_tracker, DifferentialChassis& chassis,
                 bool auto_start, int32_t line_base_target,
                 uint8_t target_laps, float outer_square_side_mm,
                 float line_width_mm, float line_width_tolerance_mm,
                 float wheel_diameter_mm, float encoder_counts_per_revolution)
      : ramfs_(*hw.FindOrExit<LibXR::RamFS>({"ramfs"})),
        line_tracker_(line_tracker),
        chassis_(chassis),
        auto_start_(false),
        line_base_target_(line_base_target),
        target_laps_(target_laps),
        outer_square_side_mm_(outer_square_side_mm),
        line_width_mm_(line_width_mm),
        line_width_tolerance_mm_(line_width_tolerance_mm),
        wheel_diameter_mm_(wheel_diameter_mm),
        encoder_counts_per_revolution_(encoder_counts_per_revolution),
        wheel_circumference_mm_(wheel_diameter_mm * PI_FLOAT),
        side_target_mm_(outer_square_side_mm - line_width_mm),
        drive_command_(
            LibXR::RamFS::CreateCommand<SimpleDicision*>("drive", DriveCommand, this))
  {
    UNUSED(auto_start);
    ASSERT(line_base_target_ >= 0);
    ASSERT(std::isfinite(outer_square_side_mm_) && outer_square_side_mm_ > 0.0F);
    ASSERT(std::isfinite(line_width_mm_) && line_width_mm_ > 0.0F);
    ASSERT(std::isfinite(line_width_tolerance_mm_) &&
           line_width_tolerance_mm_ >= 0.0F);
    ASSERT(std::isfinite(wheel_diameter_mm_) && wheel_diameter_mm_ > 0.0F);
    ASSERT(std::isfinite(encoder_counts_per_revolution_) &&
           encoder_counts_per_revolution_ > 0.0F);
    ASSERT(side_target_mm_ > 0.0F);

    ramfs_.Add(drive_command_);
    app.Register(*this);
  }

  void StartSquareLine()
  {
    completed_laps_ = 0;
    side_index_ = 0;
    side_distance_mm_ = 0.0F;
    state_ = State::LINE_FOLLOWING;
    auto_started_ = true;
    CaptureSideStart();
    line_tracker_.Start(line_base_target_);
  }

  void Stop()
  {
    line_tracker_.Stop();
    chassis_.Stop();
    state_ = State::STOPPED;
    side_distance_mm_ = 0.0F;
    CaptureSideStart();
  }

  State GetState() const { return state_; }

  const char* GetStateName() const { return StateName(state_); }

  const char* GetCurrentSegmentName() const { return SegmentName(side_index_); }

  uint8_t GetCompletedLaps() const { return completed_laps_; }

  float GetSideDistanceMm() const { return side_distance_mm_; }

  float GetSideTargetMm() const { return side_target_mm_; }

  void OnMonitor() override
  {
    if (auto_start_ && !auto_started_ && state_ == State::STOPPED)
    {
      StartSquareLine();
    }

    if (state_ != State::LINE_FOLLOWING)
    {
      return;
    }

    UpdateRouteProgress();
  }

 private:
  static constexpr float PI_FLOAT = 3.14159265358979323846F;
  static constexpr uint8_t SIDE_COUNT = 4;

  static int DriveCommand(SimpleDicision* self, int argc, char** argv)
  {
    if (self == nullptr)
    {
      return -1;
    }
    return self->HandleDriveCommand(argc, argv);
  }

  static const char* StateName(State state)
  {
    switch (state)
    {
      case State::STOPPED:
        return "stopped";
      case State::LINE_FOLLOWING:
        return "line";
      case State::FINISHED:
        return "finished";
      default:
        return "unknown";
    }
  }

  static const char* SegmentName(uint8_t side_index)
  {
    switch (side_index % SIDE_COUNT)
    {
      case 0:
        return "A->B";
      case 1:
        return "B->C";
      case 2:
        return "C->D";
      case 3:
        return "D->A";
      default:
        return "?";
    }
  }

  static int32_t AbsI32(int32_t value)
  {
    return value < 0 ? -value : value;
  }

  int HandleDriveCommand(int argc, char** argv)
  {
    if (argc < 2 || argv == nullptr)
    {
      PrintDriveUsage();
      return 0;
    }

    if (std::strcmp(argv[1], "start") == 0 || std::strcmp(argv[1], "line") == 0 ||
        std::strcmp(argv[1], "run") == 0)
    {
      StartSquareLine();
      LibXR::STDIO::Printf<"drive square line start: %s lap_target=%u\r\n">(
          SegmentName(side_index_), static_cast<unsigned>(target_laps_));
      return 0;
    }

    if (std::strcmp(argv[1], "stop") == 0)
    {
      Stop();
      LibXR::STDIO::Printf<"drive stopped\r\n">();
      return 0;
    }

    if (std::strcmp(argv[1], "status") == 0)
    {
      PrintStatus();
      return 0;
    }

    LibXR::STDIO::Printf<"unknown drive command: %s\r\n">(argv[1]);
    PrintDriveUsage();
    return -1;
  }

  void PrintDriveUsage() const
  {
    LibXR::STDIO::Printf<"Usage: drive start|line|run|stop|status\r\n">();
  }

  void PrintStatus() const
  {
    const auto& feedback = chassis_.GetFeedback();
    LibXR::STDIO::Printf<
        "drive state=%s segment=%s lap=%u/%u dist=%.1f/%.1fmm wheel=%.1fmm "
        "cpr=%.1f line=%.1f+/-%.1fmm left=%d right=%d\r\n">(
        StateName(state_), SegmentName(side_index_), static_cast<unsigned>(completed_laps_),
        static_cast<unsigned>(target_laps_), static_cast<double>(side_distance_mm_),
        static_cast<double>(side_target_mm_), static_cast<double>(wheel_diameter_mm_),
        static_cast<double>(encoder_counts_per_revolution_),
        static_cast<double>(line_width_mm_),
        static_cast<double>(line_width_tolerance_mm_),
        static_cast<int>(feedback.left.delta), static_cast<int>(feedback.right.delta));
  }

  void UpdateRouteProgress()
  {
    side_distance_mm_ = MeasureSideDistanceMm();
    if (side_distance_mm_ < side_target_mm_)
    {
      return;
    }

    AdvanceSide();
  }

  float MeasureSideDistanceMm() const
  {
    const auto& feedback = chassis_.GetFeedback();
    const int32_t left_counts =
        AbsI32(feedback.left.total - side_start_left_total_);
    const int32_t right_counts =
        AbsI32(feedback.right.total - side_start_right_total_);
    const float average_counts =
        (static_cast<float>(left_counts) + static_cast<float>(right_counts)) * 0.5F;
    return average_counts * wheel_circumference_mm_ /
           encoder_counts_per_revolution_;
  }

  void AdvanceSide()
  {
    side_index_++;
    if (side_index_ >= SIDE_COUNT)
    {
      side_index_ = 0;
      completed_laps_++;
      if (target_laps_ > 0U && completed_laps_ >= target_laps_)
      {
        Finish();
        return;
      }
    }

    side_distance_mm_ = 0.0F;
    CaptureSideStart();
  }

  void Finish()
  {
    line_tracker_.Stop();
    chassis_.Stop();
    state_ = State::FINISHED;
    side_distance_mm_ = side_target_mm_;
  }

  void CaptureSideStart()
  {
    const auto& feedback = chassis_.GetFeedback();
    side_start_left_total_ = feedback.left.total;
    side_start_right_total_ = feedback.right.total;
  }

  LibXR::RamFS& ramfs_;
  LineTracker& line_tracker_;
  DifferentialChassis& chassis_;
  bool auto_start_;
  int32_t line_base_target_;
  uint8_t target_laps_;
  float outer_square_side_mm_;
  float line_width_mm_;
  float line_width_tolerance_mm_;
  float wheel_diameter_mm_;
  float encoder_counts_per_revolution_;
  float wheel_circumference_mm_;
  float side_target_mm_;
  LibXR::RamFS::File drive_command_;
  State state_ = State::STOPPED;
  bool auto_started_ = false;
  uint8_t side_index_ = 0;
  uint8_t completed_laps_ = 0;
  int32_t side_start_left_total_ = 0;
  int32_t side_start_right_total_ = 0;
  float side_distance_mm_ = 0.0F;
};
