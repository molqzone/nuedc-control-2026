#include "display.hpp"

#include <cmath>
#include <cstddef>

#include "ti_msp_dl_config.h"

namespace App
{

namespace
{

void DrawSignedFixed2(MonoCanvas& canvas, int16_t x, int16_t y, float value)
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

}  // namespace

Display::Display()
    : i2c_(MSPM0_I2C_INIT(I2C_1, i2c_stage_buffer_, sizeof(i2c_stage_buffer_),
                          0xFFFFFFFFU),
           LibXR::I2C::Configuration{I2C_1_BUS_SPEED_HZ}),
      application_manager_(),
      hardware_(LibXR::Entry<LibXR::I2C>{i2c_, {OLED_I2C_ALIAS}}),
      surface_(hardware_, application_manager_, DISPLAY_FRAME_TOPIC, DISPLAY_PERIOD_MS),
      display_(hardware_, application_manager_, OLED_I2C_ALIAS, OLED_I2C_ADDRESS,
               DISPLAY_FRAME_TOPIC)
{
  last_render_ms_ = LibXR::Timebase::GetMilliseconds();
  surface_.PublishFullFrame();
  application_manager_.MonitorAll();
}

void Display::Process(bool imu_online, bool attitude_valid,
                      const Eigen::Matrix<float, 3, 1>& euler_rad)
{
  application_manager_.MonitorAll();

  if (!display_.IsInitialized())
  {
    return;
  }

  const uint32_t now = LibXR::Timebase::GetMilliseconds();
  if (now - last_render_ms_ < DISPLAY_PERIOD_MS)
  {
    return;
  }
  last_render_ms_ = now;
  Render(imu_online, attitude_valid, euler_rad);
  surface_.PublishFullFrame();
  application_manager_.MonitorAll();
}

bool Display::IsOnline() const { return display_.IsInitialized(); }

void Display::Render(bool imu_online, bool attitude_valid,
                     const Eigen::Matrix<float, 3, 1>& euler_rad)
{
  MonoCanvas& canvas = surface_.GetCanvas();
  canvas.Clear(false);
  canvas.DrawText(0, 0, "R:");
  canvas.DrawText(0, 8, "P:");
  canvas.DrawText(0, 16, "Y:");

  if (attitude_valid && euler_rad.allFinite())
  {
    DrawSignedFixed2(canvas, 18, 0, euler_rad.x() * RAD_TO_DEG);
    DrawSignedFixed2(canvas, 18, 8, euler_rad.y() * RAD_TO_DEG);
    DrawSignedFixed2(canvas, 18, 16, euler_rad.z() * RAD_TO_DEG);
    canvas.DrawText(80, 24, "OK");
  }
  else if (imu_online)
  {
    canvas.DrawText(80, 24, "CAL");
  }
  else
  {
    canvas.DrawText(80, 24, "OFF");
  }
}

}  // namespace App
