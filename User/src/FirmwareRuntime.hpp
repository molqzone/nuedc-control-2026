#pragma once

#include <cstdint>

#include "DisplaySurface.hpp"
#include "ICM42688.hpp"
#include "MadgwickAHRS.hpp"
#include "SSD1306.hpp"
#include "drivebase.hpp"
#include "key_led.hpp"
#include "libxr.hpp"
#include "line_sensors.hpp"
#include "motion_control.hpp"
#include "uart.hpp"

class FirmwareRuntime : public LibXR::Application
{
 public:
  struct Config
  {
    const char* ramfs_alias;
    const char* console_alias;
    std::uint32_t control_period_ms;
    std::uint32_t diagnostic_period_ms;
    std::uint32_t display_period_ms;
  };

  FirmwareRuntime(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                  MadgwickAHRS& ahrs, ICM42688& imu, DisplaySurface& surface,
                  SSD1306& display, Config config);

  void OnMonitor() override;

 private:
  using Terminal = LibXR::Terminal<64, 128, 8, 4>;

  static constexpr float RAD_TO_DEG = static_cast<float>(180.0 / LibXR::PI);
  static constexpr std::size_t TERMINAL_RX_BUFFER_SIZE = 64;

  static int DriveCommand(FirmwareRuntime* self, int argc, char** argv);
  static int DiagCommand(FirmwareRuntime* self, int argc, char** argv);
  static bool ParseFloat(const char* text, float& value);
  static void DrawSignedFixed2(MonoCanvas& canvas, std::int16_t x, std::int16_t y,
                               float value);

  int HandleDriveCommand(int argc, char** argv);
  int HandleDiagCommand(int argc, char** argv);
  void ProcessKeyLedTest();
  void PollTerminal();
  void PrintPrompt();
  void RunControlSlot(std::uint32_t now);
  void RunDiagnosticSlot(std::uint32_t now);
  void UpdateDiagnosticSnapshot(const GreySensor::Sample& line_sample,
                                std::uint32_t now);
  void RenderDisplay(std::uint32_t now);

  LibXR::RamFS& ramfs_;
  LibXR::UART& console_uart_;
  Terminal terminal_;
  MadgwickAHRS& ahrs_;
  ICM42688& imu_;
  DisplaySurface& surface_;
  SSD1306& display_;
  MotionControl motion_control_;
  Drivebase drivebase_;
  KeyLed key_led_;
  LineSensors line_sensors_;
  LibXR::RamFS::File drive_command_;
  LibXR::RamFS::File diag_command_;
  std::uint8_t terminal_rx_buffer_[TERMINAL_RX_BUFFER_SIZE]{};
  std::uint32_t control_period_ms_ = 10;
  std::uint32_t diagnostic_period_ms_ = 1000;
  std::uint32_t display_period_ms_ = 100;
  std::uint32_t last_control_ms_ = 0;
  std::uint32_t last_diagnostic_ms_ = 0;
  std::uint32_t last_display_ms_ = 0;
};
