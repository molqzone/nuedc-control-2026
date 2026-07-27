#include "key_led.hpp"

#include "libxr.hpp"
#include "ti_msp_dl_config.h"

namespace App
{

KeyLed::KeyLed()
    : key1_(KEYS_KEY1_PORT, KEYS_KEY1_PIN, KEYS_KEY1_IOMUX),
      key2_(KEYS_KEY2_PORT, KEYS_KEY2_PIN, KEYS_KEY2_IOMUX),
      key3_(KEYS_KEY3_PORT, KEYS_KEY3_PIN, KEYS_KEY3_IOMUX),
      key4_(KEYS_KEY4_PORT, KEYS_KEY4_PIN, KEYS_KEY4_IOMUX),
      led1_(LEDS_LED1_PORT, LEDS_LED1_PIN, LEDS_LED1_IOMUX),
      led2_(LEDS_LED2_PORT, LEDS_LED2_PIN, LEDS_LED2_IOMUX),
      keys_{{&key1_, &key2_, &key3_, &key4_}},
      leds_{{&led1_, &led2_}}
{
  const LibXR::GPIO::Configuration key_config{LibXR::GPIO::Direction::INPUT,
                                              LibXR::GPIO::Pull::UP};
  for (auto* key : keys_)
  {
    ASSERT(key->SetConfig(key_config) == LibXR::ErrorCode::OK);
  }

  const LibXR::GPIO::Configuration led_config{LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
                                              LibXR::GPIO::Pull::NONE};
  for (auto* led : leds_)
  {
    ASSERT(led->SetConfig(led_config) == LibXR::ErrorCode::OK);
    led->Write(false);
  }

  const uint32_t now = LibXR::Timebase::GetMilliseconds();
  for (size_t index = 0; index < KEY_COUNT; ++index)
  {
    const bool pressed = !keys_[index]->Read();
    key_states_[index].raw_pressed = pressed;
    key_states_[index].stable_pressed = pressed;
    key_states_[index].last_raw_change_ms = now;
  }
}

void KeyLed::Process()
{
  const uint32_t now = LibXR::Timebase::GetMilliseconds();
  for (size_t index = 0; index < KEY_COUNT; ++index)
  {
    KeyState& state = key_states_[index];
    const bool raw_pressed = !keys_[index]->Read();
    if (raw_pressed != state.raw_pressed)
    {
      state.raw_pressed = raw_pressed;
      state.last_raw_change_ms = now;
      continue;
    }

    if (raw_pressed != state.stable_pressed &&
        now - state.last_raw_change_ms >= DEBOUNCE_MS)
    {
      state.stable_pressed = raw_pressed;
      if (raw_pressed)
      {
        state.press_event = true;
      }
      else
      {
        state.release_event = true;
      }
    }
  }
}

bool KeyLed::IsPressed(Key key) const
{
  const size_t index = ToIndex(key);
  return index < KEY_COUNT && key_states_[index].stable_pressed;
}

uint8_t KeyLed::GetPressedMask() const
{
  uint8_t mask = 0;
  for (size_t index = 0; index < KEY_COUNT; ++index)
  {
    if (key_states_[index].stable_pressed)
    {
      mask |= static_cast<uint8_t>(1U << index);
    }
  }
  return mask;
}

bool KeyLed::TakePressEvent(Key key)
{
  const size_t index = ToIndex(key);
  if (index >= KEY_COUNT)
  {
    return false;
  }
  const bool event = key_states_[index].press_event;
  key_states_[index].press_event = false;
  return event;
}

bool KeyLed::TakeReleaseEvent(Key key)
{
  const size_t index = ToIndex(key);
  if (index >= KEY_COUNT)
  {
    return false;
  }
  const bool event = key_states_[index].release_event;
  key_states_[index].release_event = false;
  return event;
}

void KeyLed::SetLed(Led led, bool on)
{
  const size_t index = ToIndex(led);
  if (index >= LED_COUNT)
  {
    return;
  }
  led_states_[index] = on;
  leds_[index]->Write(on);
}

void KeyLed::ToggleLed(Led led)
{
  const size_t index = ToIndex(led);
  if (index < LED_COUNT)
  {
    SetLed(led, !led_states_[index]);
  }
}

bool KeyLed::IsLedOn(Led led) const
{
  const size_t index = ToIndex(led);
  return index < LED_COUNT && led_states_[index];
}

}  // namespace App
