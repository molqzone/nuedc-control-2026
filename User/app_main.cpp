#include "app_main.h"

#include <array>
#include <cstdint>

#include "libxr.hpp"
#include "mspm0_timebase.hpp"
#include "mspm0_uart.hpp"
#include "ti_msp_dl_config.h"

namespace Scheduler
{
void Init(LibXR::UART& uart);
void Update();
}  // namespace Scheduler

extern "C" void app_main()
{
  static LibXR::MSPM0Timebase timebase;
  LibXR::PlatformInit();

  NVIC_SetPriority(GPIOA_INT_IRQn, 1U);
  NVIC_SetPriority(GPIOB_INT_IRQn, 1U);
  NVIC_SetPriority(UART_0_INST_INT_IRQN, 2U);
  NVIC_SetPriority(SysTick_IRQn, 3U);

  static std::array<uint8_t, 256> uart_rx_stage_buffer{};
  LibXR::MSPM0UART uart(MSPM0_UART_INIT(UART_0, uart_rx_stage_buffer.data(),
                                        uart_rx_stage_buffer.size(), 16, 512));
  LibXR::STDIO::read_ = uart.read_port_;
  LibXR::STDIO::write_ = uart.write_port_;

  (void)timebase;
  Scheduler::Init(uart);

  while (true)
  {
    Scheduler::Update();
  }
}
