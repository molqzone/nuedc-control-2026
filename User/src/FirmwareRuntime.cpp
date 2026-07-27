#include "FirmwareRuntime.hpp"

#include <cmath>
#include <cstring>

#include "ti_msp_dl_config.h"

namespace
{
constexpr std::uint32_t kDiagMagic = 0x44494147U;  // "DIAG"
constexpr std::uint32_t kDiagVersion = 1U;

struct AppDiagSnapshot
{
  std::uint32_t magic;
  std::uint32_t version;
  std::uint32_t sequence;
  std::uint32_t timestamp_ms;
  std::uint32_t gpioa_din31_0;
  std::uint32_t gpiob_din31_0;
  std::uint32_t imu_online;
  std::uint32_t imu_who_am_i;
  std::int32_t imu_last_error;
  std::uint32_t imu_last_failed_register;
  std::uint32_t imu_sample_count;
  std::int32_t imu_temperature_mdeg_c;
  std::int32_t imu_accl_mg[3];
  std::int32_t imu_gyro_millirad_s[3];
  std::uint32_t attitude_valid;
  std::int32_t yaw_mdeg;
  std::uint32_t display_online;
  std::uint32_t line_raw_mask;
  std::uint32_t line_active_mask;
  std::uint32_t line_active_count;
  std::int32_t line_position;
  std::uint32_t line_detected;
  std::uint32_t line_lost;
  std::uint32_t line_lost_side;
  std::uint32_t line_sequence;
};

std::int32_t ScaleFinite(float value, float scale)
{
  if (!std::isfinite(value))
  {
    return 0;
  }
  return static_cast<std::int32_t>(value * scale);
}

}  // namespace

extern "C" {
volatile AppDiagSnapshot g_app_diag = {
    kDiagMagic,
    kDiagVersion,
};
}

FirmwareRuntime::FirmwareRuntime(LibXR::HardwareContainer& hw,
                                 LibXR::ApplicationManager& app,
                                 MadgwickAHRS& ahrs, ICM42688& imu,
                                 DisplaySurface& surface, SSD1306& display,
                                 Config config)
    : ramfs_(*hw.FindOrExit<LibXR::RamFS>({config.ramfs_alias})),
      console_uart_(*hw.FindOrExit<LibXR::UART>({config.console_alias})),
      terminal_(ramfs_, nullptr, console_uart_.read_port_, console_uart_.write_port_),
      ahrs_(ahrs),
      imu_(imu),
      surface_(surface),
      display_(display),
      drivebase_(hw),
      key_led_(hw),
      line_sensors_(hw),
      drive_command_(LibXR::RamFS::CreateCommand<FirmwareRuntime*>(
          "drive", DriveCommand, this)),
      diag_command_(
          LibXR::RamFS::CreateCommand<FirmwareRuntime*>("diag", DiagCommand, this)),
      control_period_ms_(config.control_period_ms == 0U ? 1U
                                                        : config.control_period_ms),
      diagnostic_period_ms_(config.diagnostic_period_ms == 0U
                                ? 1000U
                                : config.diagnostic_period_ms),
      display_period_ms_(config.display_period_ms == 0U ? 100U
                                                        : config.display_period_ms)
{
  ASSERT(line_sensors_.Initialize());
  drivebase_.RegisterCommands(ramfs_);
  ramfs_.Add(drive_command_);
  ramfs_.Add(diag_command_);

  const std::uint32_t now = LibXR::Timebase::GetMilliseconds();
  last_control_ms_ = now;
  last_diagnostic_ms_ = now;
  last_display_ms_ = now;
  surface_.PublishFullFrame();
  UpdateDiagnosticSnapshot(line_sensors_.Read(), now);

  app.Register(*this);
  LibXR::STDIO::Printf<
      "XRobot runtime ready. Commands: drive, chassis, diag, ls, cd.\r\n">();
  PrintPrompt();
}

void FirmwareRuntime::OnMonitor()
{
  ProcessKeyLedTest();
  PollTerminal();

  const std::uint32_t now = LibXR::Timebase::GetMilliseconds();
  RunControlSlot(now);
  RunDiagnosticSlot(now);
  RenderDisplay(now);
}

int FirmwareRuntime::DriveCommand(FirmwareRuntime* self, int argc, char** argv)
{
  if (self == nullptr)
  {
    return -1;
  }
  return self->HandleDriveCommand(argc, argv);
}

int FirmwareRuntime::DiagCommand(FirmwareRuntime* self, int argc, char** argv)
{
  if (self == nullptr)
  {
    return -1;
  }
  return self->HandleDiagCommand(argc, argv);
}

bool FirmwareRuntime::ParseFloat(const char* text, float& value)
{
  if (text == nullptr || *text == '\0')
  {
    return false;
  }

  std::uint32_t index = 0;
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

void FirmwareRuntime::DrawSignedFixed2(MonoCanvas& canvas, std::int16_t x,
                                       std::int16_t y, float value)
{
  if (!std::isfinite(value))
  {
    canvas.DrawText(x, y, "--");
    return;
  }

  std::int32_t scaled =
      static_cast<std::int32_t>(value * 100.0F + (value >= 0.0F ? 0.5F : -0.5F));
  const bool negative = scaled < 0;
  std::uint32_t magnitude = static_cast<std::uint32_t>(negative ? -scaled : scaled);

  char text[16]{};
  std::size_t index = sizeof(text) - 1U;
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

int FirmwareRuntime::HandleDriveCommand(int argc, char** argv)
{
  if (argc < 2)
  {
    LibXR::STDIO::Printf<
        "Usage: drive line|stop|forward|back|left|right|faster|slower|turn "
        "<deg>|status\r\n">();
    return 0;
  }

  const char* command = argv[1];
  if (std::strcmp(command, "line") == 0)
  {
    motion_control_.HandleCommand('g');
  }
  else if (std::strcmp(command, "stop") == 0)
  {
    motion_control_.HandleCommand('x');
    drivebase_.Stop();
  }
  else if (std::strcmp(command, "forward") == 0 ||
           std::strcmp(command, "fwd") == 0)
  {
    motion_control_.HandleCommand('w');
  }
  else if (std::strcmp(command, "back") == 0)
  {
    motion_control_.HandleCommand('s');
  }
  else if (std::strcmp(command, "left") == 0)
  {
    motion_control_.HandleCommand('a');
  }
  else if (std::strcmp(command, "right") == 0)
  {
    motion_control_.HandleCommand('d');
  }
  else if (std::strcmp(command, "faster") == 0)
  {
    motion_control_.HandleCommand('+');
  }
  else if (std::strcmp(command, "slower") == 0)
  {
    motion_control_.HandleCommand('-');
  }
  else if (std::strcmp(command, "turn") == 0)
  {
    if (argc != 3)
    {
      LibXR::STDIO::Printf<"Usage: drive turn <deg>\r\n">();
      return -1;
    }
    float delta_deg = 0.0F;
    if (!ParseFloat(argv[2], delta_deg) ||
        !motion_control_.StartRelativeTurn(delta_deg))
    {
      LibXR::STDIO::Printf<"turn rejected\r\n">();
      return -1;
    }
  }
  else if (std::strcmp(command, "status") == 0)
  {
    const auto& feedback = drivebase_.GetFeedback();
    LibXR::STDIO::Printf<
        "mode=%d attitude=%d yaw=%.2f line=0x%02X left=%d right=%d\r\n">(
        static_cast<int>(motion_control_.GetMode()), ahrs_.IsValid() ? 1 : 0,
        static_cast<double>(ahrs_.GetContinuousYawDegrees()),
        static_cast<unsigned>(g_app_diag.line_active_mask),
        static_cast<int>(feedback.left.delta), static_cast<int>(feedback.right.delta));
    return 0;
  }
  else
  {
    LibXR::STDIO::Printf<"unknown drive command: %s\r\n">(command);
    return -1;
  }

  LibXR::STDIO::Printf<"drive %s ok\r\n">(command);
  return 0;
}

int FirmwareRuntime::HandleDiagCommand(int argc, char** argv)
{
  UNUSED(argc);
  UNUSED(argv);
  LibXR::STDIO::Printf<
      "diag seq=%lu t=%lu imu=%lu who=0x%02lX attitude=%lu yaw_mdeg=%ld "
      "display=%lu line=0x%02lX pos=%ld lost=%lu\r\n">(
      static_cast<unsigned long>(g_app_diag.sequence),
      static_cast<unsigned long>(g_app_diag.timestamp_ms),
      static_cast<unsigned long>(g_app_diag.imu_online),
      static_cast<unsigned long>(g_app_diag.imu_who_am_i),
      static_cast<unsigned long>(g_app_diag.attitude_valid),
      static_cast<long>(g_app_diag.yaw_mdeg),
      static_cast<unsigned long>(g_app_diag.display_online),
      static_cast<unsigned long>(g_app_diag.line_active_mask),
      static_cast<long>(g_app_diag.line_position),
      static_cast<unsigned long>(g_app_diag.line_lost));
  return 0;
}

void FirmwareRuntime::ProcessKeyLedTest()
{
  using Key = KeyLed::Key;
  using Led = KeyLed::Led;

  key_led_.Process();
  if (key_led_.TakePressEvent(Key::KEY1))
  {
    key_led_.ToggleLed(Led::LED1);
  }
  if (key_led_.TakePressEvent(Key::KEY2))
  {
    key_led_.ToggleLed(Led::LED2);
  }
  if (key_led_.TakePressEvent(Key::KEY3))
  {
    key_led_.SetLed(Led::LED1, true);
    key_led_.SetLed(Led::LED2, true);
  }
  if (key_led_.TakePressEvent(Key::KEY4))
  {
    key_led_.SetLed(Led::LED1, false);
    key_led_.SetLed(Led::LED2, false);
  }
}

void FirmwareRuntime::PollTerminal()
{
  while (console_uart_.read_port_->Size() > 0U)
  {
    const std::size_t read_size =
        LibXR::min(console_uart_.read_port_->Size(), TERMINAL_RX_BUFFER_SIZE);
    LibXR::ReadOperation read_operation;
    if (console_uart_.Read({terminal_rx_buffer_, read_size}, read_operation) !=
        LibXR::ErrorCode::OK)
    {
      break;
    }

    LibXR::RawData raw_data(terminal_rx_buffer_, read_size);
    terminal_.write_mutex_->Lock();
    terminal_.Parse(raw_data);
    terminal_.write_stream_.Commit();
    terminal_.write_mutex_->Unlock();
  }
}

void FirmwareRuntime::PrintPrompt()
{
  terminal_.write_mutex_->Lock();
  terminal_.ShowHeader();
  terminal_.write_stream_.Commit();
  terminal_.write_mutex_->Unlock();
}

void FirmwareRuntime::RunControlSlot(std::uint32_t now)
{
  if (now - last_control_ms_ < control_period_ms_)
  {
    return;
  }

  const float control_period_seconds =
      static_cast<float>(now - last_control_ms_) * 0.001F;
  last_control_ms_ = now;

  const GreySensor::Sample line_sample = line_sensors_.Read();
  const MotionControl::Outputs outputs = motion_control_.Step(
      {line_sample.line_detected != 0U, static_cast<float>(line_sample.position),
       ahrs_.GetContinuousYawDegrees(), ahrs_.IsValid()});
  (void)drivebase_.SetWheelTargets(static_cast<float>(outputs.left_target),
                                   static_cast<float>(outputs.right_target),
                                   control_period_seconds);
}

void FirmwareRuntime::RunDiagnosticSlot(std::uint32_t now)
{
  if (now - last_diagnostic_ms_ < diagnostic_period_ms_)
  {
    return;
  }

  last_diagnostic_ms_ = now;
  UpdateDiagnosticSnapshot(line_sensors_.Read(), now);
}

void FirmwareRuntime::UpdateDiagnosticSnapshot(const GreySensor::Sample& line_sample,
                                               std::uint32_t now)
{
  g_app_diag.magic = kDiagMagic;
  g_app_diag.version = kDiagVersion;
  g_app_diag.sequence = g_app_diag.sequence + 1U;
  g_app_diag.timestamp_ms = now;
  g_app_diag.gpioa_din31_0 = GPIOA->DIN31_0;
  g_app_diag.gpiob_din31_0 = GPIOB->DIN31_0;
  const auto& accl = imu_.GetAccl();
  const auto& gyro = imu_.GetGyro();
  g_app_diag.imu_online = imu_.IsOnline() ? 1U : 0U;
  g_app_diag.imu_who_am_i = imu_.GetWhoAmI();
  g_app_diag.imu_last_error = static_cast<std::int32_t>(imu_.GetLastError());
  g_app_diag.imu_last_failed_register = imu_.GetLastFailedRegister();
  g_app_diag.imu_sample_count = imu_.GetSampleCount();
  g_app_diag.imu_temperature_mdeg_c = ScaleFinite(imu_.GetTemperature(), 1000.0F);
  g_app_diag.imu_accl_mg[0] = ScaleFinite(accl.x(), 1000.0F);
  g_app_diag.imu_accl_mg[1] = ScaleFinite(accl.y(), 1000.0F);
  g_app_diag.imu_accl_mg[2] = ScaleFinite(accl.z(), 1000.0F);
  g_app_diag.imu_gyro_millirad_s[0] = ScaleFinite(gyro.x(), 1000.0F);
  g_app_diag.imu_gyro_millirad_s[1] = ScaleFinite(gyro.y(), 1000.0F);
  g_app_diag.imu_gyro_millirad_s[2] = ScaleFinite(gyro.z(), 1000.0F);
  g_app_diag.attitude_valid = ahrs_.IsValid() ? 1U : 0U;
  g_app_diag.yaw_mdeg = ScaleFinite(ahrs_.GetContinuousYawDegrees(), 1000.0F);
  g_app_diag.display_online = display_.IsInitialized() ? 1U : 0U;
  g_app_diag.line_raw_mask = line_sample.raw_mask;
  g_app_diag.line_active_mask = line_sample.active_mask;
  g_app_diag.line_active_count = line_sample.active_count;
  g_app_diag.line_position = line_sample.position;
  g_app_diag.line_detected = line_sample.line_detected;
  g_app_diag.line_lost = line_sample.line_lost;
  g_app_diag.line_lost_side = line_sample.lost_side;
  g_app_diag.line_sequence = line_sample.sequence;
}

void FirmwareRuntime::RenderDisplay(std::uint32_t now)
{
  if (!display_.IsInitialized() || now - last_display_ms_ < display_period_ms_)
  {
    return;
  }

  last_display_ms_ = now;
  MonoCanvas& canvas = surface_.GetCanvas();
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
  surface_.PublishFullFrame();
}
