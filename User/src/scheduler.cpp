#include <cstdint>

#include "console.hpp"
#include "display.hpp"
#include "drivebase.hpp"
#include "imu.hpp"
#include "key_led.hpp"
#include "libxr.hpp"
#include "line_sensors.hpp"
#include "motion_control.hpp"
#include "uart.hpp"

namespace
{
constexpr uint32_t kControlPeriodMs = 10;
constexpr float kControlPeriodSeconds = static_cast<float>(kControlPeriodMs) / 1000.0F;
constexpr uint32_t kMonitorPeriodMs = 1000;

struct Runtime
{
  explicit Runtime(LibXR::UART& console_uart)
      : console_uart(console_uart), console(motion_control)
  {
  }

  LibXR::UART& console_uart;
  App::MotionControl motion_control;
  App::Drivebase drivebase;
  App::KeyLed key_led;
  App::LineSensors line_sensors;
  App::Display display;
  App::Imu imu;
  App::Console console;
  uint32_t last_control_ms = 0;
  uint32_t last_monitor_ms = 0;
  bool initialized = false;
};

Runtime* runtime = nullptr;

bool IsStopCommand(uint8_t command)
{
  return command == 'x' || command == 'X' || command == ' ' || command == 0x1BU;
}

void ProcessKeyLedTest(App::KeyLed& key_led)
{
  using Key = App::KeyLed::Key;
  using Led = App::KeyLed::Led;

  key_led.Process();
  if (key_led.TakePressEvent(Key::KEY1))
  {
    key_led.ToggleLed(Led::LED1);
  }
  if (key_led.TakePressEvent(Key::KEY2))
  {
    key_led.ToggleLed(Led::LED2);
  }
  if (key_led.TakePressEvent(Key::KEY3))
  {
    key_led.SetLed(Led::LED1, true);
    key_led.SetLed(Led::LED2, true);
  }
  if (key_led.TakePressEvent(Key::KEY4))
  {
    key_led.SetLed(Led::LED1, false);
    key_led.SetLed(Led::LED2, false);
  }
}

void DrainConsole(Runtime& rt)
{
  while (rt.console_uart.read_port_->Size() > 0U)
  {
    uint8_t command = 0;
    LibXR::ReadOperation read_operation;
    if (rt.console_uart.Read({&command, 1}, read_operation) != LibXR::ErrorCode::OK)
    {
      break;
    }

    rt.console.HandleByte(command);
    if (IsStopCommand(command))
    {
      rt.drivebase.Stop();
    }
  }
}

void RunControlSlot(Runtime& rt, uint32_t now)
{
  if (now - rt.last_control_ms < kControlPeriodMs)
  {
    return;
  }

  rt.last_control_ms = now;
  const GreySensor::Sample line_sample = rt.line_sensors.Read();
  const App::MotionControl::Outputs outputs = rt.motion_control.Step(
      {line_sample.line_detected != 0U, static_cast<float>(line_sample.position),
       rt.imu.GetContinuousYawDegrees(), rt.imu.IsAttitudeValid()});
  (void)rt.drivebase.SetWheelTargets(static_cast<float>(outputs.left_target),
                                     static_cast<float>(outputs.right_target),
                                     kControlPeriodSeconds);
}

void RunMonitorSlot(Runtime& rt, uint32_t now)
{
  if (now - rt.last_monitor_ms < kMonitorPeriodMs)
  {
    return;
  }

  rt.last_monitor_ms = now;
  rt.imu.Monitor();
}

void InitializeRuntime(Runtime& rt)
{
  if (rt.initialized)
  {
    return;
  }

  ASSERT(rt.line_sensors.Initialize());
  const uint32_t now = LibXR::Timebase::GetMilliseconds();
  rt.last_control_ms = now;
  rt.last_monitor_ms = now;
  rt.console.PrintHelp();
  rt.initialized = true;
}

void RunRuntime(Runtime& rt)
{
  ProcessKeyLedTest(rt.key_led);
  rt.imu.Process();
  rt.display.Process(rt.imu.IsOnline(), rt.imu.IsAttitudeValid(), rt.imu.GetEuler());
  DrainConsole(rt);

  const uint32_t now = LibXR::Timebase::GetMilliseconds();
  RunControlSlot(rt, now);
  RunMonitorSlot(rt, now);
}

}  // namespace

namespace Scheduler
{

void Init(LibXR::UART& uart)
{
  if (runtime != nullptr)
  {
    return;
  }

  static Runtime firmware_runtime(uart);
  runtime = &firmware_runtime;
  InitializeRuntime(*runtime);
}

void Update()
{
  if (runtime == nullptr)
  {
    return;
  }

  RunRuntime(*runtime);
}

}  // namespace Scheduler
