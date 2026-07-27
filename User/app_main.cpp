#include "app_main.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "database.hpp"
#include "libxr.hpp"
#include "mspm0_gpio.hpp"
#include "mspm0_i2c.hpp"
#include "mspm0_pwm.hpp"
#include "mspm0_timebase.hpp"
#include "mspm0_uart.hpp"
#include "ramfs.hpp"
#include "ti_msp_dl_config.h"
#include "xrobot_main.hpp"

class UserRamDatabase : public LibXR::Database
{
 private:
  static constexpr std::size_t MAX_ENTRIES = 8;
  static constexpr std::size_t MAX_NAME_LEN = 48;
  static constexpr std::size_t MAX_DATA_SIZE = 64;

  struct Entry
  {
    bool used = false;
    std::array<char, MAX_NAME_LEN> name{};
    std::array<std::uint8_t, MAX_DATA_SIZE> data{};
    std::size_t size = 0;
  };

  Entry* FindEntry(const char* name)
  {
    for (auto& entry : entries_)
    {
      if (entry.used && std::strcmp(entry.name.data(), name) == 0)
      {
        return &entry;
      }
    }
    return nullptr;
  }

  LibXR::ErrorCode Get(KeyBase& key) override
  {
    auto* entry = FindEntry(key.name_);
    if (entry == nullptr)
    {
      return LibXR::ErrorCode::NOT_FOUND;
    }
    if (key.raw_data_.size_ < entry->size)
    {
      return LibXR::ErrorCode::FAILED;
    }

    std::memcpy(key.raw_data_.addr_, entry->data.data(), entry->size);
    return LibXR::ErrorCode::OK;
  }

  LibXR::ErrorCode Set(KeyBase& key, LibXR::RawData data) override
  {
    if (data.size_ > MAX_DATA_SIZE || key.raw_data_.size_ < data.size_)
    {
      return LibXR::ErrorCode::FULL;
    }

    auto* entry = FindEntry(key.name_);
    if (entry == nullptr)
    {
      const LibXR::ErrorCode add_result = Add(key);
      if (add_result != LibXR::ErrorCode::OK)
      {
        return add_result;
      }
      entry = FindEntry(key.name_);
    }

    std::memcpy(key.raw_data_.addr_, data.addr_, data.size_);
    std::memcpy(entry->data.data(), data.addr_, data.size_);
    entry->size = data.size_;
    return LibXR::ErrorCode::OK;
  }

  LibXR::ErrorCode Add(KeyBase& key) override
  {
    if (key.raw_data_.size_ > MAX_DATA_SIZE ||
        std::strlen(key.name_) >= MAX_NAME_LEN)
    {
      return LibXR::ErrorCode::FULL;
    }

    auto* entry = FindEntry(key.name_);
    if (entry != nullptr)
    {
      return Set(key, key.raw_data_);
    }

    for (auto& candidate : entries_)
    {
      if (!candidate.used)
      {
        candidate.used = true;
        std::strncpy(candidate.name.data(), key.name_, MAX_NAME_LEN - 1);
        candidate.name[MAX_NAME_LEN - 1] = '\0';
        std::memcpy(candidate.data.data(), key.raw_data_.addr_, key.raw_data_.size_);
        candidate.size = key.raw_data_.size_;
        return LibXR::ErrorCode::OK;
      }
    }

    return LibXR::ErrorCode::FULL;
  }

  std::array<Entry, MAX_ENTRIES> entries_{};
};

extern "C" void app_main()
{
  static LibXR::MSPM0Timebase timebase;
  LibXR::PlatformInit();

  NVIC_SetPriority(GPIOA_INT_IRQn, 1U);
  NVIC_SetPriority(GPIOB_INT_IRQn, 1U);
  NVIC_SetPriority(UART_0_INST_INT_IRQN, 2U);
  NVIC_SetPriority(SysTick_IRQn, 3U);

  static std::array<uint8_t, 256> uart_rx_stage_buffer{};
  static std::array<uint8_t, 32> i2c_icm42688_stage_buffer{};
  static std::array<uint8_t, 8> i2c_oled_stage_buffer{};
  static LibXR::RamFS ramfs("ramfs");
  static UserRamDatabase database;

  static LibXR::MSPM0UART uart(MSPM0_UART_INIT(UART_0, uart_rx_stage_buffer.data(),
                                               uart_rx_stage_buffer.size(), 16, 512));
  static LibXR::MSPM0I2C i2c_icm42688(
      MSPM0_I2C_INIT(I2C_0, i2c_icm42688_stage_buffer.data(),
                     i2c_icm42688_stage_buffer.size(), 8),
      LibXR::I2C::Configuration{I2C_0_BUS_SPEED_HZ});
  static LibXR::MSPM0I2C i2c_oled(
      MSPM0_I2C_INIT(I2C_1, i2c_oled_stage_buffer.data(),
                     i2c_oled_stage_buffer.size(), 0xFFFFFFFFU),
      LibXR::I2C::Configuration{I2C_1_BUS_SPEED_HZ});

  static LibXR::MSPM0PWM motor_a_pwm(
      {MOTOR_PWM_INST, GPIO_MOTOR_PWM_C3_IDX, MOTOR_PWM_INST_CLK_FREQ});
  static LibXR::MSPM0PWM motor_b_pwm(
      {MOTOR_PWM_INST, GPIO_MOTOR_PWM_C2_IDX, MOTOR_PWM_INST_CLK_FREQ});

  static LibXR::MSPM0GPIO motor_ain1(MOTOR_AIN1_PORT, MOTOR_AIN1_AIN1_PIN,
                                     MOTOR_AIN1_AIN1_IOMUX);
  static LibXR::MSPM0GPIO motor_ain2(MOTOR_AIN2_PORT, MOTOR_AIN2_AIN2_PIN,
                                     MOTOR_AIN2_AIN2_IOMUX);
  static LibXR::MSPM0GPIO motor_bin1(MOTOR_BIN1_PORT, MOTOR_BIN1_BIN1_PIN,
                                     MOTOR_BIN1_BIN1_IOMUX);
  static LibXR::MSPM0GPIO motor_bin2(MOTOR_BIN2_PORT, MOTOR_BIN2_BIN2_PIN,
                                     MOTOR_BIN2_BIN2_IOMUX);

  static LibXR::MSPM0GPIO encoder_1a(ENCODERS_PORT, ENCODERS_E1A_PIN,
                                     ENCODERS_E1A_IOMUX);
  static LibXR::MSPM0GPIO encoder_1b(ENCODERS_PORT, ENCODERS_E1B_PIN,
                                     ENCODERS_E1B_IOMUX);
  static LibXR::MSPM0GPIO encoder_2a(ENCODERS_PORT, ENCODERS_E2A_PIN,
                                     ENCODERS_E2A_IOMUX);
  static LibXR::MSPM0GPIO encoder_2b(ENCODERS_PORT, ENCODERS_E2B_PIN,
                                     ENCODERS_E2B_IOMUX);

  static LibXR::MSPM0GPIO icm42688_int(ICM42688_INT_PORT, ICM42688_INT_INT1_PIN,
                                       ICM42688_INT_INT1_IOMUX);

  static LibXR::MSPM0GPIO line_ad1(LINE_A_PORT, LINE_A_AD1_PIN,
                                   LINE_A_AD1_IOMUX);
  static LibXR::MSPM0GPIO line_ad2(LINE_B_PORT, LINE_B_AD2_PIN,
                                   LINE_B_AD2_IOMUX);
  static LibXR::MSPM0GPIO line_ad3(LINE_B_PORT, LINE_B_AD3_PIN,
                                   LINE_B_AD3_IOMUX);
  static LibXR::MSPM0GPIO line_ad4(LINE_B_PORT, LINE_B_AD4_PIN,
                                   LINE_B_AD4_IOMUX);
  static LibXR::MSPM0GPIO line_ad5(LINE_B_PORT, LINE_B_AD5_PIN,
                                   LINE_B_AD5_IOMUX);
  static LibXR::MSPM0GPIO line_ad6(LINE_A_PORT, LINE_A_AD6_PIN,
                                   LINE_A_AD6_IOMUX);
  static LibXR::MSPM0GPIO line_ad7(LINE_A_PORT, LINE_A_AD7_PIN,
                                   LINE_A_AD7_IOMUX);
  static LibXR::MSPM0GPIO line_ad8(LINE_A_PORT, LINE_A_AD8_PIN,
                                   LINE_A_AD8_IOMUX);

  static LibXR::MSPM0GPIO key1(KEYS_KEY1_PORT, KEYS_KEY1_PIN, KEYS_KEY1_IOMUX);
  static LibXR::MSPM0GPIO key2(KEYS_KEY2_PORT, KEYS_KEY2_PIN, KEYS_KEY2_IOMUX);
  static LibXR::MSPM0GPIO key3(KEYS_KEY3_PORT, KEYS_KEY3_PIN, KEYS_KEY3_IOMUX);
  static LibXR::MSPM0GPIO key4(KEYS_KEY4_PORT, KEYS_KEY4_PIN, KEYS_KEY4_IOMUX);
  static LibXR::MSPM0GPIO led1(LEDS_LED1_PORT, LEDS_LED1_PIN, LEDS_LED1_IOMUX);
  static LibXR::MSPM0GPIO led2(LEDS_LED2_PORT, LEDS_LED2_PIN, LEDS_LED2_IOMUX);

  LibXR::STDIO::read_ = uart.read_port_;
  LibXR::STDIO::write_ = uart.write_port_;

  static LibXR::HardwareContainer hw(
      LibXR::Entry<LibXR::RamFS>{ramfs, {"ramfs", "fs"}},
      LibXR::Entry<LibXR::Database>{database, {"database"}},
      LibXR::Entry<LibXR::UART>{uart, {"uart_debug", "debug_uart", "console"}},
      LibXR::Entry<LibXR::I2C>{i2c_icm42688, {"i2c_icm42688", "i2c0"}},
      LibXR::Entry<LibXR::I2C>{i2c_oled, {"i2c_oled", "i2c1"}},
      LibXR::Entry<LibXR::PWM>{motor_a_pwm, {"motor_a_pwm"}},
      LibXR::Entry<LibXR::PWM>{motor_b_pwm, {"motor_b_pwm"}},
      LibXR::Entry<LibXR::GPIO>{motor_ain1, {"motor_ain1"}},
      LibXR::Entry<LibXR::GPIO>{motor_ain2, {"motor_ain2"}},
      LibXR::Entry<LibXR::GPIO>{motor_bin1, {"motor_bin1"}},
      LibXR::Entry<LibXR::GPIO>{motor_bin2, {"motor_bin2"}},
      LibXR::Entry<LibXR::GPIO>{encoder_1a, {"encoder_1a"}},
      LibXR::Entry<LibXR::GPIO>{encoder_1b, {"encoder_1b"}},
      LibXR::Entry<LibXR::GPIO>{encoder_2a, {"encoder_2a"}},
      LibXR::Entry<LibXR::GPIO>{encoder_2b, {"encoder_2b"}},
      LibXR::Entry<LibXR::GPIO>{icm42688_int, {"icm42688_int"}},
      LibXR::Entry<LibXR::GPIO>{line_ad1, {"line_ad1"}},
      LibXR::Entry<LibXR::GPIO>{line_ad2, {"line_ad2"}},
      LibXR::Entry<LibXR::GPIO>{line_ad3, {"line_ad3"}},
      LibXR::Entry<LibXR::GPIO>{line_ad4, {"line_ad4"}},
      LibXR::Entry<LibXR::GPIO>{line_ad5, {"line_ad5"}},
      LibXR::Entry<LibXR::GPIO>{line_ad6, {"line_ad6"}},
      LibXR::Entry<LibXR::GPIO>{line_ad7, {"line_ad7"}},
      LibXR::Entry<LibXR::GPIO>{line_ad8, {"line_ad8"}},
      LibXR::Entry<LibXR::GPIO>{key1, {"key1"}},
      LibXR::Entry<LibXR::GPIO>{key2, {"key2"}},
      LibXR::Entry<LibXR::GPIO>{key3, {"key3"}},
      LibXR::Entry<LibXR::GPIO>{key4, {"key4"}},
      LibXR::Entry<LibXR::GPIO>{led1, {"led1"}},
      LibXR::Entry<LibXR::GPIO>{led2, {"led2"}});

  (void)timebase;
  XRobotMain(hw);
}
