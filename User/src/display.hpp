#pragma once

#include <cstdint>

#include "DisplaySurface.hpp"
#include "SSD1306.hpp"
#include "libxr.hpp"
#include "mspm0_i2c.hpp"

namespace App
{

class Display
{
 public:
  Display();

  void Process(bool imu_online, bool attitude_valid,
               const Eigen::Matrix<float, 3, 1>& euler_rad);
  bool IsOnline() const;

 private:
  static constexpr uint16_t OLED_I2C_ADDRESS = 0x3C;
  static constexpr uint32_t DISPLAY_PERIOD_MS = 100;
  static constexpr float RAD_TO_DEG = static_cast<float>(180.0 / LibXR::PI);
  static constexpr const char* OLED_I2C_ALIAS = "i2c_oled";
  static constexpr const char* DISPLAY_FRAME_TOPIC = DisplaySurface::DEFAULT_FRAME_TOPIC;

  void Render(bool imu_online, bool attitude_valid,
              const Eigen::Matrix<float, 3, 1>& euler_rad);

  uint8_t i2c_stage_buffer_[8]{};
  LibXR::MSPM0I2C i2c_;
  LibXR::ApplicationManager application_manager_;
  LibXR::HardwareContainer hardware_;
  DisplaySurface surface_;
  SSD1306 display_;
  uint32_t last_render_ms_ = 0;
};

}  // namespace App
