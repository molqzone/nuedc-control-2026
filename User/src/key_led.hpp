#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "app_framework.hpp"
#include "gpio.hpp"

class KeyLed
{
 public:
  enum class Key : uint8_t
  {
    KEY1,
    KEY2,
    KEY3,
    KEY4,
    COUNT
  };

  enum class Led : uint8_t
  {
    LED1,
    LED2,
    COUNT
  };

  explicit KeyLed(LibXR::HardwareContainer& hw);

  void Process();
  bool IsPressed(Key key) const;
  uint8_t GetPressedMask() const;
  bool TakePressEvent(Key key);
  bool TakeReleaseEvent(Key key);

  void SetLed(Led led, bool on);
  void ToggleLed(Led led);
  bool IsLedOn(Led led) const;

 private:
  static constexpr uint32_t DEBOUNCE_MS = 20;
  static constexpr size_t KEY_COUNT = static_cast<size_t>(Key::COUNT);
  static constexpr size_t LED_COUNT = static_cast<size_t>(Led::COUNT);

  struct KeyState
  {
    uint32_t last_raw_change_ms = 0;
    bool raw_pressed = false;
    bool stable_pressed = false;
    bool press_event = false;
    bool release_event = false;
  };

  static constexpr size_t ToIndex(Key key) { return static_cast<size_t>(key); }

  static constexpr size_t ToIndex(Led led) { return static_cast<size_t>(led); }

  std::array<LibXR::GPIO*, KEY_COUNT> keys_;
  std::array<LibXR::GPIO*, LED_COUNT> leds_;
  std::array<KeyState, KEY_COUNT> key_states_{};
  std::array<bool, LED_COUNT> led_states_{};
};
