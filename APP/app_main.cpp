#include "app_main.h"

#include <algorithm>
#include <array>
#include <cstdint>

#include "ICM42688.hpp"
#include "libxr.hpp"
#include "line_sensor_array.hpp"
#include "mspm0_gpio.hpp"
#include "mspm0_i2c.hpp"
#include "mspm0_pwm.hpp"
#include "mspm0_timebase.hpp"
#include "mspm0_uart.hpp"
#include "quadrature_encoder.hpp"
#include "tb6612.hpp"
#include "ti_msp_dl_config.h"
#include "velocity_pi.hpp"

namespace
{

constexpr uint32_t CONTROL_PERIOD_MS = 10;
constexpr uint32_t TELEMETRY_PERIOD_MS = 200;
constexpr uint32_t MONITOR_PERIOD_MS = 1000;
constexpr int32_t DEFAULT_BASE_TARGET = 30;
constexpr int32_t MIN_BASE_TARGET = 10;
constexpr int32_t MAX_BASE_TARGET = 55;
constexpr int32_t MAX_LINE_TARGET = 60;
constexpr int32_t LINE_CORRECTION_GAIN = 3;

enum class DriveMode : uint8_t
{
  STOPPED,
  MANUAL,
  LINE_FOLLOW,
};

const char* ModeName(DriveMode mode)
{
  switch (mode)
  {
    case DriveMode::MANUAL:
      return "manual";
    case DriveMode::LINE_FOLLOW:
      return "line";
    case DriveMode::STOPPED:
    default:
      return "stop";
  }
}

bool ComputeLineTargets(uint8_t black_mask, int32_t base_target, int32_t& left_target,
                        int32_t& right_target)
{
  static constexpr std::array<int8_t, App::LineSensorArray::CHANNEL_COUNT> WEIGHTS = {
      -7, -5, -3, -1, 1, 3, 5, 7};

  if (black_mask == 0U || black_mask == 0xFFU)
  {
    left_target = 0;
    right_target = 0;
    return false;
  }

  int32_t weighted_sum = 0;
  int32_t active_count = 0;
  for (size_t index = 0; index < WEIGHTS.size(); index++)
  {
    if ((black_mask & static_cast<uint8_t>(1U << index)) != 0U)
    {
      weighted_sum += WEIGHTS[index];
      active_count++;
    }
  }

  const int32_t correction = weighted_sum * LINE_CORRECTION_GAIN / active_count;
  left_target = std::clamp(base_target + correction, int32_t{0}, MAX_LINE_TARGET);
  right_target = std::clamp(base_target - correction, int32_t{0}, MAX_LINE_TARGET);
  return true;
}

void HandleCommand(uint8_t command, DriveMode& mode, int8_t& manual_left,
                   int8_t& manual_right, int32_t& base_target)
{
  switch (command)
  {
    case 'g':
    case 'G':
      mode = DriveMode::LINE_FOLLOW;
      break;
    case 'w':
    case 'W':
      mode = DriveMode::MANUAL;
      manual_left = 1;
      manual_right = 1;
      break;
    case 's':
    case 'S':
      mode = DriveMode::MANUAL;
      manual_left = -1;
      manual_right = -1;
      break;
    case 'a':
    case 'A':
      mode = DriveMode::MANUAL;
      manual_left = -1;
      manual_right = 1;
      break;
    case 'd':
    case 'D':
      mode = DriveMode::MANUAL;
      manual_left = 1;
      manual_right = -1;
      break;
    case '+':
      base_target = std::min(base_target + 5, MAX_BASE_TARGET);
      break;
    case '-':
      base_target = std::max(base_target - 5, MIN_BASE_TARGET);
      break;
    case 'x':
    case 'X':
    case ' ':
      mode = DriveMode::STOPPED;
      manual_left = 0;
      manual_right = 0;
      break;
    default:
      break;
  }
}

}  // namespace

extern "C" void app_main()
{
  LibXR::MSPM0Timebase timebase;
  LibXR::PlatformInit();

  NVIC_SetPriority(GPIOA_INT_IRQn, 1U);
  NVIC_SetPriority(GPIOB_INT_IRQn, 1U);
  NVIC_SetPriority(UART_0_INST_INT_IRQN, 2U);
  NVIC_SetPriority(SysTick_IRQn, 3U);

  static uint8_t uart_rx_stage_buffer[256];
  LibXR::MSPM0UART uart(MSPM0_UART_INIT(UART_0, uart_rx_stage_buffer,
                                        sizeof(uart_rx_stage_buffer), 16, 512));
  LibXR::STDIO::read_ = uart.read_port_;
  LibXR::STDIO::write_ = uart.write_port_;

  LibXR::MSPM0GPIO motor_ain1(MOTOR_AIN1_PORT, MOTOR_AIN1_AIN1_PIN,
                              MOTOR_AIN1_AIN1_IOMUX);
  LibXR::MSPM0GPIO motor_ain2(MOTOR_AIN2_PORT, MOTOR_AIN2_AIN2_PIN,
                              MOTOR_AIN2_AIN2_IOMUX);
  LibXR::MSPM0GPIO motor_bin1(MOTOR_BIN1_PORT, MOTOR_BIN1_BIN1_PIN,
                              MOTOR_BIN1_BIN1_IOMUX);
  LibXR::MSPM0GPIO motor_bin2(MOTOR_BIN2_PORT, MOTOR_BIN2_BIN2_PIN,
                              MOTOR_BIN2_BIN2_IOMUX);
  LibXR::MSPM0PWM motor_a_pwm(
      {MOTOR_PWM_INST, GPIO_MOTOR_PWM_C3_IDX, MOTOR_PWM_INST_CLK_FREQ});
  LibXR::MSPM0PWM motor_b_pwm(
      {MOTOR_PWM_INST, GPIO_MOTOR_PWM_C2_IDX, MOTOR_PWM_INST_CLK_FREQ});
  TB6612Motor motor_left(motor_a_pwm, motor_ain1, motor_ain2);
  TB6612Motor motor_right(motor_b_pwm, motor_bin1, motor_bin2);
  (void)motor_left.Stop();
  (void)motor_right.Stop();

  LibXR::MSPM0GPIO encoder_1a(ENCODERS_PORT, ENCODERS_E1A_PIN, ENCODERS_E1A_IOMUX);
  LibXR::MSPM0GPIO encoder_1b(ENCODERS_PORT, ENCODERS_E1B_PIN, ENCODERS_E1B_IOMUX);
  LibXR::MSPM0GPIO encoder_2a(ENCODERS_PORT, ENCODERS_E2A_PIN, ENCODERS_E2A_IOMUX);
  LibXR::MSPM0GPIO encoder_2b(ENCODERS_PORT, ENCODERS_E2B_PIN, ENCODERS_E2B_IOMUX);
  App::QuadratureEncoder encoder_left(encoder_1a, encoder_1b);
  App::QuadratureEncoder encoder_right(encoder_2a, encoder_2b, true);
  ASSERT(encoder_left.Initialize() == LibXR::ErrorCode::OK);
  ASSERT(encoder_right.Initialize() == LibXR::ErrorCode::OK);

  LibXR::MSPM0GPIO line_ad1(LINE_A_PORT, LINE_A_AD1_PIN, LINE_A_AD1_IOMUX);
  LibXR::MSPM0GPIO line_ad2(LINE_B_PORT, LINE_B_AD2_PIN, LINE_B_AD2_IOMUX);
  LibXR::MSPM0GPIO line_ad3(LINE_B_PORT, LINE_B_AD3_PIN, LINE_B_AD3_IOMUX);
  LibXR::MSPM0GPIO line_ad4(LINE_B_PORT, LINE_B_AD4_PIN, LINE_B_AD4_IOMUX);
  LibXR::MSPM0GPIO line_ad5(LINE_B_PORT, LINE_B_AD5_PIN, LINE_B_AD5_IOMUX);
  LibXR::MSPM0GPIO line_ad6(LINE_A_PORT, LINE_A_AD6_PIN, LINE_A_AD6_IOMUX);
  LibXR::MSPM0GPIO line_ad7(LINE_A_PORT, LINE_A_AD7_PIN, LINE_A_AD7_IOMUX);
  LibXR::MSPM0GPIO line_ad8(LINE_A_PORT, LINE_A_AD8_PIN, LINE_A_AD8_IOMUX);
  App::LineSensorArray line_sensors({{&line_ad1, &line_ad2, &line_ad3, &line_ad4,
                                      &line_ad5, &line_ad6, &line_ad7, &line_ad8}});
  ASSERT(line_sensors.Initialize() == LibXR::ErrorCode::OK);

  static uint8_t i2c_stage_buffer[32];
  LibXR::MSPM0I2C i2c(
      MSPM0_I2C_INIT(I2C_0, i2c_stage_buffer, sizeof(i2c_stage_buffer), 8),
      LibXR::I2C::Configuration{I2C_0_BUS_SPEED_HZ});
  LibXR::MSPM0GPIO icm42688_int(ICM42688_INT_PORT, ICM42688_INT_INT1_PIN,
                                ICM42688_INT_INT1_IOMUX);
  ASSERT(icm42688_int.SetConfig({LibXR::GPIO::Direction::FALL_INTERRUPT,
                                 LibXR::GPIO::Pull::UP}) == LibXR::ErrorCode::OK);
  icm42688_int.DisableInterrupt();

  LibXR::ApplicationManager application_manager;
  LibXR::HardwareContainer hardware(
      LibXR::Entry<LibXR::I2C>{i2c, {"i2c_icm42688"}},
      LibXR::Entry<LibXR::GPIO>{icm42688_int, {"icm42688_int"}});
  ICM42688 imu(hardware, application_manager, ICM42688::DataRate::DATA_RATE_1KHZ,
               ICM42688::AcclRange::RANGE_16G, ICM42688::GyroRange::DPS_2000,
               LibXR::Quaternion<float>(1.0F, 0.0F, 0.0F, 0.0F), false, "icm42688_gyro",
               "icm42688_accl", 0x68);

  App::VelocityPI left_pi({0.0125F, 0.0015F, 0.90F});
  App::VelocityPI right_pi({0.0125F, 0.0015F, 0.90F});

  DriveMode mode = DriveMode::STOPPED;
  int8_t manual_left = 0;
  int8_t manual_right = 0;
  int32_t base_target = DEFAULT_BASE_TARGET;
  int32_t left_target = 0;
  int32_t right_target = 0;
  int32_t left_delta = 0;
  int32_t right_delta = 0;
  float left_duty = 0.0F;
  float right_duty = 0.0F;
  uint8_t line_mask = line_sensors.ReadBlackMask();

  uint32_t last_control_ms = LibXR::Timebase::GetMilliseconds();
  uint32_t last_telemetry_ms = last_control_ms;
  uint32_t last_monitor_ms = last_control_ms;

  LibXR::STDIO::Printf<
      "Bare-metal ready: g=line, x/space=stop, w/s/a/d=manual, +/-=speed.\r\n">();

  while (true)
  {
    (void)imu.Process();

    while (uart.read_port_->Size() > 0U)
    {
      uint8_t command = 0;
      LibXR::ReadOperation read_operation;
      if (uart.Read({&command, 1}, read_operation) != LibXR::ErrorCode::OK)
      {
        break;
      }
      HandleCommand(command, mode, manual_left, manual_right, base_target);
    }

    const uint32_t now = LibXR::Timebase::GetMilliseconds();
    if (now - last_control_ms >= CONTROL_PERIOD_MS)
    {
      last_control_ms = now;
      line_mask = line_sensors.ReadBlackMask();
      left_delta = encoder_left.TakeDelta();
      right_delta = encoder_right.TakeDelta();

      switch (mode)
      {
        case DriveMode::MANUAL:
          left_target = base_target * manual_left;
          right_target = base_target * manual_right;
          break;
        case DriveMode::LINE_FOLLOW:
          (void)ComputeLineTargets(line_mask, base_target, left_target, right_target);
          break;
        case DriveMode::STOPPED:
        default:
          left_target = 0;
          right_target = 0;
          break;
      }

      left_duty = left_pi.Update(left_target, left_delta);
      right_duty = right_pi.Update(right_target, right_delta);
      (void)motor_left.Set(left_duty);
      (void)motor_right.Set(right_duty);
    }

    if (now - last_telemetry_ms >= TELEMETRY_PERIOD_MS)
    {
      last_telemetry_ms = now;
      const auto& accl = imu.GetAccl();
      const auto& gyro = imu.GetGyro();
      LibXR::STDIO::Printf<
          "mode=%s line=0x%02X enc=%d,%d target=%d,%d duty=%f,%f base=%d "
          "imu=%s who=0x%02X fail_reg=0x%02X err=%d\r\n">(
          ModeName(mode), static_cast<unsigned int>(line_mask), left_delta, right_delta,
          left_target, right_target, left_duty, right_duty, base_target,
          imu.IsOnline() ? "ok" : "offline", static_cast<unsigned int>(imu.GetWhoAmI()),
          static_cast<unsigned int>(imu.GetLastFailedRegister()),
          static_cast<int>(imu.GetLastError()));
      LibXR::STDIO::Printf<"imu_samples=%u temp_c=%f\r\n">(
          static_cast<unsigned int>(imu.GetSampleCount()), imu.GetTemperature());
      LibXR::STDIO::Printf<"accl_g=%f,%f,%f\r\n">(accl.x(), accl.y(), accl.z());
      LibXR::STDIO::Printf<"gyro_rad_s=%f,%f,%f\r\n">(gyro.x(), gyro.y(), gyro.z());
    }

    if (now - last_monitor_ms >= MONITOR_PERIOD_MS)
    {
      last_monitor_ms = now;
      application_manager.MonitorAll();
    }
  }
}
