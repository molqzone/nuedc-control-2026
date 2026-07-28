#include "app_framework.hpp"
#include "libxr.hpp"

// Module headers
#include "DisplaySurface.hpp"
#include "SSD1306.hpp"
#include "ICM42688.hpp"
#include "MadgwickAHRS.hpp"
#include "BitsButtonXR.hpp"
#include "GreySensor.hpp"
#include "DRV8870Motor.hpp"
#include "DifferentialChassis.hpp"
#include "LineTracker.hpp"
#include "SimpleDicision.hpp"
#include "Scheduler.hpp"

static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager appmgr;

  // Auto-generated module instantiations
  static SSD1306 ssd1306(hw, appmgr, "i2c_oled", 60, 128, 64, 64);
  static DisplaySurface display_surface(hw, appmgr, ssd1306, "display_frame", 33);
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
      104
  );
  static MadgwickAHRS ahrs(hw, appmgr, 0.05, "icm42688_gyro", "icm42688_accl");
  static BitsButtonXR buttons(
      hw,
      appmgr,
      {{"key1", false, {50, 1000, 500, 300}}, {"key2", false, {50, 1000, 500, 300}}, {"key3", false, {50, 1000, 500, 300}}, {"key4", false, {50, 1000, 500, 300}}},
      {}
  );
  static GreySensor line_sensors(
      hw,
      appmgr,
      {"line_ad1", "line_ad2", "line_ad3", "line_ad4", "line_ad5", "line_ad6", "line_ad7", "line_ad8"},
      false,
      "line_sensors",
      10,
      LibXR::GPIO::Pull::UP
  );
  static DRV8870Motor left_motor(
      hw,
      appmgr,
      "motor_a_pwm",
      "motor_ain1",
      "motor_ain2",
      "encoder_1a",
      "encoder_1b",
      false,
      false,
      0.08,
      0.9,
      {1.0, 0.0125, 0.0, 0.0, 0.0, 0.9, false}
  );
  static DRV8870Motor right_motor(
      hw,
      appmgr,
      "motor_b_pwm",
      "motor_bin1",
      "motor_bin2",
      "encoder_2a",
      "encoder_2b",
      false,
      false,
      0.08,
      0.9,
      {1.0, 0.0125, 0.0, 0.0, 0.0, 0.9, false}
  );
  static DifferentialChassis chassis(
      hw,
      appmgr,
      &left_motor,
      &right_motor,
      "chassis_command",
      100,
      0.01,
      0.1
  );
  static LineTracker line_tracker(
      hw,
      appmgr,
      "line_sensors",
      "chassis_command",
      10,
      100,
      {1.0, 3.0, 0.0, 0.0, 0.0, 30.0, false},
      60,
      2,
      100
  );
  static SimpleDicision simple_dicision(
      hw,
      appmgr,
      line_tracker,
      chassis,
      true,
      30,
      1,
      1000.0,
      18.0,
      2.0,
      65.0,
      1560.0
  );
  static Scheduler scheduler(
      hw,
      appmgr,
      icm42688,
      ahrs,
      chassis,
      buttons,
      10,
      1000,
      100,
      "display_frame",
      128,
      64,
      "line_sensors"
  );

  while (true) {
    appmgr.MonitorAll();
    Thread::Sleep(1);
  }
}
