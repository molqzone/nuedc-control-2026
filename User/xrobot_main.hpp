#include "app_framework.hpp"
#include "libxr.hpp"

// Module headers
#include "DisplaySurface.hpp"
#include "SSD1306.hpp"
#include "ICM42688.hpp"
#include "MadgwickAHRS.hpp"
#include "FirmwareRuntime.hpp"

static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager appmgr;

  // Auto-generated module instantiations
  static DisplaySurface display_surface(
      hw,
      appmgr,
      DisplaySurface::Config{128, 64, "display_frame", 33}
  );
  static SSD1306 ssd1306(
      hw,
      appmgr,
      SSD1306::Config{"i2c_oled", 0x3C, 128, 64, "display_frame", 64}
  );
  static ICM42688 icm42688(
      hw,
      appmgr,
      ICM42688::DataRate::DATA_RATE_1KHZ,
      ICM42688::AcclRange::RANGE_16G,
      ICM42688::GyroRange::DPS_2000,
      {1.0, 0.0, 0.0, 0.0},
      false,
      "icm42688_gyro",
      "icm42688_accl",
      0x68
  );
  static MadgwickAHRS ahrs(hw, appmgr, 0.05, "icm42688_gyro", "icm42688_accl");
  static FirmwareRuntime firmware_runtime(
      hw,
      appmgr,
      ahrs,
      icm42688,
      display_surface,
      ssd1306,
      FirmwareRuntime::Config{"ramfs", "console", 10, 1000, 100}
  );

  while (true) {
    appmgr.MonitorAll();
    Timer::RefreshTimerInIdle();
    Thread::Sleep(1);
  }
}
