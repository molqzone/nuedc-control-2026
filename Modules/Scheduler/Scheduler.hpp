#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Application scheduler owning diagnostics, display rendering, button actions, and sensor orchestration.
constructor_args:
  - imu: "@icm42688"
  - ahrs: "@ahrs"
  - chassis: "@chassis"
  - buttons: "@buttons"
  - control_period_ms: 10
  - diagnostic_period_ms: 1000
  - display_period_ms: 100
  - display_frame_topic_name: display_frame
  - display_width: 128
  - display_height: 64
  - line_sample_topic_name: line_sensors
template_args: []
required_hardware: ramfs console led1 led2
depends: [DisplaySurface, ICM42688, MadgwickAHRS, BitsButtonXR, GreySensor, DifferentialChassis]
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "BitsButtonXR.hpp"
#include "DifferentialChassis.hpp"
#include "DisplayTypes.hpp"
#include "GreySensor.hpp"
#include "ICM42688.hpp"
#include "MadgwickAHRS.hpp"
#include "MonoCanvas.hpp"
#include "SEGGER_RTT.h"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "ramfs.hpp"
#include "ti_msp_dl_config.h"
#include "uart.hpp"

class BitsButtonXR;
class ICM42688;
class MadgwickAHRS;

class Scheduler : public LibXR::Application
{
 public:
  Scheduler(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
            ICM42688& imu, MadgwickAHRS& ahrs,
            DifferentialChassis& chassis, BitsButtonXR& buttons,
            uint32_t control_period_ms, uint32_t diagnostic_period_ms,
            uint32_t display_period_ms, const char* display_frame_topic_name,
            uint16_t display_width, uint16_t display_height,
            const char* line_sample_topic_name)
      : ramfs_(*hw.FindOrExit<LibXR::RamFS>({"ramfs"})),
        console_uart_(*hw.FindOrExit<LibXR::UART>({"console"})),
        terminal_(ramfs_, nullptr, console_uart_.read_port_, console_uart_.write_port_),
        ahrs_(ahrs),
        imu_(imu),
        chassis_(chassis),
        buttons_(buttons),
        led1_(*hw.FindOrExit<LibXR::GPIO>({"led1"})),
        led2_(*hw.FindOrExit<LibXR::GPIO>({"led2"})),
        display_width_(CheckedDisplayWidth(display_width)),
        display_height_(CheckedDisplayHeight(display_height)),
        display_pages_(CalculateDisplayPages(display_height_)),
        display_framebuffer_size_(
            CalculateDisplayFramebufferSize(display_width_, display_pages_)),
        display_frame_topic_(LibXR::Topic::CreateTopic<DisplayFrame>(
            CheckedTopicName(display_frame_topic_name))),
        display_canvas_(display_frame_.data.data(), display_framebuffer_size_,
                        display_width_, display_height_, display_width_),
        line_sample_topic_(
            LibXR::Topic::CreateTopic<GreySensor::Sample>(
                CheckedTopicName(line_sample_topic_name))),
        line_sample_sub_(line_sample_topic_),
        diag_command_(LibXR::RamFS::CreateCommand<Scheduler*>("diag", DiagCommand, this))
  {
    ASSERT(control_period_ms > 0U);
    ASSERT(diagnostic_period_ms > 0U);
    ASSERT(display_period_ms > 0U);

    const LibXR::GPIO::Configuration led_config{LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
                                                LibXR::GPIO::Pull::NONE};
    ASSERT(led1_.SetConfig(led_config) == LibXR::ErrorCode::OK);
    ASSERT(led2_.SetConfig(led_config) == LibXR::ErrorCode::OK);
    SetLed(1, false);
    SetLed(2, false);

    chassis.RegisterCommands(ramfs_);
    ramfs_.Add(diag_command_);
    line_sample_sub_.StartWaiting();

    WriteRttDiagnostic(BuildDiagnosticSnapshot(
        latest_line_sample_, LibXR::Timebase::GetMilliseconds()));

    // Periodic work is driven by LibXR's soft timer, which the bare-metal backend
    // advances from Thread::Sleep in the main loop. No RTOS thread is required.
    auto control_task =
        LibXR::Timer::CreateTask(&Scheduler::ControlTick, this, control_period_ms);
    auto diagnostic_task =
        LibXR::Timer::CreateTask(&Scheduler::DiagnosticTick, this, diagnostic_period_ms);
    auto display_task =
        LibXR::Timer::CreateTask(&Scheduler::DisplayTick, this, display_period_ms);
    LibXR::Timer::Add(control_task);
    LibXR::Timer::Add(diagnostic_task);
    LibXR::Timer::Add(display_task);
    LibXR::Timer::Start(control_task);
    LibXR::Timer::Start(diagnostic_task);
    LibXR::Timer::Start(display_task);

    RenderDisplay();

    LibXR::STDIO::Printf<
        "Scheduler ready. Commands: drive, chassis, diag, ls, cd.\r\n">();
    PrintPrompt();
    app.Register(*this);
  }

  // Per-iteration I/O draining. These consume queues as fast as the loop spins and
  // are intentionally not periodic, so they stay out of the soft timer.
  void OnMonitor() override
  {
    RefreshLineSample();
    ProcessButtonEvents();
    PollTerminal();
  }

 private:
  struct DiagnosticSnapshot
  {
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint32_t gpioa_din31_0;
    uint32_t gpiob_din31_0;
    uint32_t imu_online;
    uint32_t imu_who_am_i;
    int32_t imu_last_error;
    uint32_t imu_last_failed_register;
    uint32_t imu_sample_count;
    int32_t imu_temperature_mdeg_c;
    int32_t imu_accl_mg[3];
    int32_t imu_gyro_millirad_s[3];
    uint32_t attitude_valid;
    int32_t yaw_mdeg;
    uint32_t line_raw_mask;
    uint32_t line_active_mask;
    uint32_t line_active_count;
    int32_t line_position;
    uint32_t line_detected;
    uint32_t line_lost;
    uint32_t line_lost_side;
    uint32_t line_sequence;
  };

  using Terminal = LibXR::Terminal<64, 128, 8, 4>;

  static constexpr float RAD_TO_DEG = static_cast<float>(180.0 / LibXR::PI);
  static constexpr size_t TERMINAL_RX_BUFFER_SIZE = 64;

  static const char* CheckedTopicName(const char* name)
  {
    ASSERT(name != nullptr && name[0] != '\0');
    return name;
  }

  static uint16_t CheckedDisplayWidth(uint16_t width)
  {
    ASSERT(width != 0U && width <= DisplayFrame::MAX_WIDTH);
    return width;
  }

  static uint16_t CheckedDisplayHeight(uint16_t height)
  {
    ASSERT(height != 0U && height <= DisplayFrame::MAX_HEIGHT);
    return height;
  }

  static uint16_t CalculateDisplayPages(uint16_t height)
  {
    return static_cast<uint16_t>((height + 7U) >> 3U);
  }

  static size_t CalculateDisplayFramebufferSize(uint16_t width, uint16_t pages)
  {
    return static_cast<size_t>(width) * pages;
  }

  static int32_t ScaleFinite(float value, float scale)
  {
    if (!std::isfinite(value))
    {
      return 0;
    }
    return static_cast<int32_t>(value * scale);
  }

  static void AppendChar(char*& out, size_t& remaining, char value)
  {
    if (remaining <= 1U)
    {
      return;
    }
    *out = value;
    out++;
    remaining--;
  }

  static void AppendText(char*& out, size_t& remaining, const char* text)
  {
    if (text == nullptr)
    {
      return;
    }
    while (*text != '\0')
    {
      AppendChar(out, remaining, *text);
      text++;
    }
  }

  static void AppendUnsigned(char*& out, size_t& remaining, uint32_t value)
  {
    char digits[10]{};
    size_t count = 0U;
    do
    {
      digits[count] = static_cast<char>('0' + (value % 10U));
      value /= 10U;
      count++;
    } while (value != 0U && count < sizeof(digits));

    while (count > 0U)
    {
      count--;
      AppendChar(out, remaining, digits[count]);
    }
  }

  static void AppendSigned(char*& out, size_t& remaining, int32_t value)
  {
    if (value < 0)
    {
      AppendChar(out, remaining, '-');
      const uint32_t magnitude = static_cast<uint32_t>(-(static_cast<int64_t>(value)));
      AppendUnsigned(out, remaining, magnitude);
      return;
    }
    AppendUnsigned(out, remaining, static_cast<uint32_t>(value));
  }

  static void AppendHexFixed(char*& out, size_t& remaining, uint32_t value,
                             uint8_t digits)
  {
    static constexpr char kHex[] = "0123456789ABCDEF";
    AppendText(out, remaining, "0x");
    for (uint8_t index = digits; index > 0U; index--)
    {
      const uint8_t shift = static_cast<uint8_t>((index - 1U) * 4U);
      AppendChar(out, remaining, kHex[(value >> shift) & 0xFU]);
    }
  }

  static void Terminate(char*& out, size_t& remaining)
  {
    if (remaining > 0U)
    {
      *out = '\0';
    }
  }

  static int DiagCommand(Scheduler* self, int argc, char** argv)
  {
    if (self == nullptr)
    {
      return -1;
    }
    return self->HandleDiagCommand(argc, argv);
  }

  static void DrawSignedFixed2(MonoCanvas& canvas, int16_t x, int16_t y, float value)
  {
    if (!std::isfinite(value))
    {
      canvas.DrawText(x, y, "--");
      return;
    }

    int32_t scaled =
        static_cast<int32_t>(value * 100.0F + (value >= 0.0F ? 0.5F : -0.5F));
    const bool negative = scaled < 0;
    uint32_t magnitude = static_cast<uint32_t>(negative ? -scaled : scaled);

    char text[16]{};
    size_t index = sizeof(text) - 1U;
    text[index] = '\0';
    text[--index] = static_cast<char>('0' + (magnitude % 10U));
    magnitude /= 10U;
    text[--index] = static_cast<char>('0' + (magnitude % 10U));
    magnitude /= 10U;
    text[--index] = '.';
    do
    {
      text[--index] = static_cast<char>('0' + (magnitude % 10U));
      magnitude /= 10U;
    } while (magnitude > 0U);
    if (negative)
    {
      text[--index] = '-';
    }

    canvas.DrawText(x, y, &text[index]);
  }

  int HandleDiagCommand(int argc, char** argv)
  {
    UNUSED(argc);
    UNUSED(argv);
    const DiagnosticSnapshot diag =
        BuildDiagnosticSnapshot(latest_line_sample_, LibXR::Timebase::GetMilliseconds());
    LibXR::STDIO::Printf<
        "diag seq=%lu t=%lu imu=%lu who=0x%02lX attitude=%lu yaw_mdeg=%ld "
        "line=0x%02lX pos=%ld lost=%lu\r\n">(
        static_cast<uint32_t>(diag.sequence),
        static_cast<uint32_t>(diag.timestamp_ms),
        static_cast<uint32_t>(diag.imu_online),
        static_cast<uint32_t>(diag.imu_who_am_i),
        static_cast<uint32_t>(diag.attitude_valid), static_cast<int32_t>(diag.yaw_mdeg),
        static_cast<uint32_t>(diag.line_active_mask),
        static_cast<int32_t>(diag.line_position),
        static_cast<uint32_t>(diag.line_lost));
    return 0;
  }

  void SetLed(uint8_t index, bool on)
  {
    if (index == 1U)
    {
      led1_on_ = on;
      led1_.Write(on);
    }
    else if (index == 2U)
    {
      led2_on_ = on;
      led2_.Write(on);
    }
  }

  void ToggleLed(uint8_t index)
  {
    if (index == 1U)
    {
      SetLed(1, !led1_on_);
    }
    else if (index == 2U)
    {
      SetLed(2, !led2_on_);
    }
  }

  void ProcessButtonEvents()
  {
    BitsButtonXR::ButtonEventResult event{};
    while (buttons_.GetEventResult(event))
    {
      if (event.event_type != BitsButtonXR::ButtonEvent::PRESSED ||
          event.key_alias == nullptr)
      {
        continue;
      }

      if (std::strcmp(event.key_alias, "key1") == 0)
      {
        ToggleLed(1);
      }
      else if (std::strcmp(event.key_alias, "key2") == 0)
      {
        ToggleLed(2);
      }
      else if (std::strcmp(event.key_alias, "key3") == 0)
      {
        SetLed(1, true);
        SetLed(2, true);
      }
      else if (std::strcmp(event.key_alias, "key4") == 0)
      {
        SetLed(1, false);
        SetLed(2, false);
      }
    }
  }

  void RefreshLineSample()
  {
    if (!line_sample_sub_.Available())
    {
      return;
    }

    latest_line_sample_ = line_sample_sub_.GetData();
    has_line_sample_ = true;
    line_sample_sub_.StartWaiting();
  }

  void PollTerminal()
  {
    while (console_uart_.read_port_->Size() > 0U)
    {
      const size_t read_size =
          LibXR::min(console_uart_.read_port_->Size(), TERMINAL_RX_BUFFER_SIZE);
      LibXR::ReadOperation read_operation;
      LibXR::RawData read_buffer(terminal_rx_buffer_, read_size);
      if ((*console_uart_.read_port_)(read_buffer, read_operation) !=
          LibXR::ErrorCode::OK)
      {
        break;
      }

      terminal_.write_mutex_->Lock();
      terminal_.Parse(read_buffer);
      terminal_.write_stream_.Commit();
      terminal_.write_mutex_->Unlock();
    }
  }

  void PrintPrompt()
  {
    terminal_.write_mutex_->Lock();
    terminal_.ShowHeader();
    terminal_.write_stream_.Commit();
    terminal_.write_mutex_->Unlock();
  }

  static void ControlTick(Scheduler* self) { self->RunControlSlot(); }
  static void DiagnosticTick(Scheduler* self) { self->RunDiagnosticSlot(); }
  static void DisplayTick(Scheduler* self) { self->RenderDisplay(); }

  void RunControlSlot() { (void)chassis_.Update(); }

  void RunDiagnosticSlot()
  {
    RefreshLineSample();
    WriteRttDiagnostic(
        BuildDiagnosticSnapshot(latest_line_sample_, LibXR::Timebase::GetMilliseconds()));
  }

  DiagnosticSnapshot BuildDiagnosticSnapshot(const GreySensor::Sample& line_sample,
                                             uint32_t now)
  {
    DiagnosticSnapshot diag{};
    diag.sequence = ++diag_sequence_;
    diag.timestamp_ms = now;
    diag.gpioa_din31_0 = GPIOA->DIN31_0;
    diag.gpiob_din31_0 = GPIOB->DIN31_0;
    const auto& accl = imu_.GetAccl();
    const auto& gyro = imu_.GetGyro();
    diag.imu_online = imu_.IsOnline() ? 1U : 0U;
    diag.imu_who_am_i = imu_.GetWhoAmI();
    diag.imu_last_error = static_cast<int32_t>(imu_.GetLastError());
    diag.imu_last_failed_register = imu_.GetLastFailedRegister();
    diag.imu_sample_count = imu_.GetSampleCount();
    diag.imu_temperature_mdeg_c = ScaleFinite(imu_.GetTemperature(), 1000.0F);
    diag.imu_accl_mg[0] = ScaleFinite(accl.x(), 1000.0F);
    diag.imu_accl_mg[1] = ScaleFinite(accl.y(), 1000.0F);
    diag.imu_accl_mg[2] = ScaleFinite(accl.z(), 1000.0F);
    diag.imu_gyro_millirad_s[0] = ScaleFinite(gyro.x(), 1000.0F);
    diag.imu_gyro_millirad_s[1] = ScaleFinite(gyro.y(), 1000.0F);
    diag.imu_gyro_millirad_s[2] = ScaleFinite(gyro.z(), 1000.0F);
    diag.attitude_valid = ahrs_.IsValid() ? 1U : 0U;
    diag.yaw_mdeg = ScaleFinite(ahrs_.GetContinuousYawDegrees(), 1000.0F);
    diag.line_raw_mask = line_sample.raw_mask;
    diag.line_active_mask = line_sample.active_mask;
    diag.line_active_count = line_sample.active_count;
    diag.line_position = line_sample.position;
    diag.line_detected = line_sample.line_detected;
    diag.line_lost = line_sample.line_lost;
    diag.line_lost_side = line_sample.lost_side;
    diag.line_sequence = line_sample.sequence;
    return diag;
  }

  void AppendDiagLine(char* buffer, size_t size, const DiagnosticSnapshot& diag)
  {
    char* out = buffer;
    size_t remaining = size;
    AppendText(out, remaining, "diag seq=");
    AppendUnsigned(out, remaining, diag.sequence);
    AppendText(out, remaining, " t=");
    AppendUnsigned(out, remaining, diag.timestamp_ms);
    AppendText(out, remaining, " imu=");
    AppendUnsigned(out, remaining, diag.imu_online);
    AppendText(out, remaining, " who=");
    AppendHexFixed(out, remaining, diag.imu_who_am_i, 2U);
    AppendText(out, remaining, " attitude=");
    AppendUnsigned(out, remaining, diag.attitude_valid);
    AppendText(out, remaining, " yaw_mdeg=");
    AppendSigned(out, remaining, diag.yaw_mdeg);
    AppendText(out, remaining, " line=");
    AppendHexFixed(out, remaining, diag.line_active_mask, 2U);
    AppendText(out, remaining, " pos=");
    AppendSigned(out, remaining, diag.line_position);
    AppendText(out, remaining, " lost=");
    AppendUnsigned(out, remaining, diag.line_lost);
    AppendText(out, remaining, "\r\n");
    Terminate(out, remaining);
  }

  void WriteRttDiagnostic(const DiagnosticSnapshot& diag)
  {
    char line[160]{};
    AppendDiagLine(line, sizeof(line), diag);
    SEGGER_RTT_WriteString(0, line);
  }

  void RenderDisplay()
  {
    MonoCanvas& canvas = display_canvas_;
    canvas.Clear(false);
    canvas.DrawText(0, 0, "ROLL :");
    canvas.DrawText(0, 8, "PITCH:");
    canvas.DrawText(0, 16, "YAW  :");
    canvas.DrawText(0, 32, "AHRS :");

    const auto& euler_rad = ahrs_.GetEuler();
    if (ahrs_.IsValid() && euler_rad.allFinite())
    {
      DrawSignedFixed2(canvas, 42, 0, euler_rad.x() * RAD_TO_DEG);
      DrawSignedFixed2(canvas, 42, 8, euler_rad.y() * RAD_TO_DEG);
      DrawSignedFixed2(canvas, 42, 16, euler_rad.z() * RAD_TO_DEG);
      canvas.DrawText(42, 32, "OK");
    }
    else
    {
      canvas.DrawText(42, 32, "CAL");
    }

    PublishDisplayFrame();
  }

  void PublishDisplayFrame()
  {
    MonoCanvas::Rect dirty{};
    bool full_update = false;
    if (!display_canvas_.GetDirtyRect(dirty, full_update))
    {
      return;
    }

    display_frame_.size = static_cast<uint32_t>(display_framebuffer_size_);
    display_frame_.width = display_width_;
    display_frame_.height = display_height_;
    display_frame_.pitch = display_width_;
    display_frame_.x = dirty.x;
    display_frame_.y = dirty.y;
    display_frame_.dirty_width = dirty.width;
    display_frame_.dirty_height = dirty.height;
    display_frame_.sequence = display_sequence_++;
    display_frame_.pixel_format = DisplayPixelFormat::MONO_VTILED_LSB;
    display_frame_.full_update = full_update;
    display_frame_topic_.Publish(display_frame_);
    display_canvas_.ClearDirty();
  }

  LibXR::RamFS& ramfs_;
  LibXR::UART& console_uart_;
  Terminal terminal_;
  MadgwickAHRS& ahrs_;
  ICM42688& imu_;
  DifferentialChassis& chassis_;
  BitsButtonXR& buttons_;
  LibXR::GPIO& led1_;
  LibXR::GPIO& led2_;
  uint16_t display_width_;
  uint16_t display_height_;
  uint16_t display_pages_;
  size_t display_framebuffer_size_;
  DisplayFrame display_frame_{};
  LibXR::Topic display_frame_topic_;
  MonoCanvas display_canvas_;
  LibXR::Topic line_sample_topic_;
  LibXR::Topic::ASyncSubscriber<GreySensor::Sample> line_sample_sub_;
  LibXR::RamFS::File diag_command_;
  uint8_t terminal_rx_buffer_[TERMINAL_RX_BUFFER_SIZE]{};
  GreySensor::Sample latest_line_sample_{};
  uint32_t diag_sequence_ = 0;
  uint32_t display_sequence_ = 0;
  bool has_line_sample_ = false;
  bool led1_on_ = false;
  bool led2_on_ = false;
};
