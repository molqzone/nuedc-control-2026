#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "gpio.hpp"
#include "libxr_def.hpp"

class GreySensor
{
 public:
  static constexpr size_t MAX_CHANNEL_COUNT = 8;
  static constexpr int16_t POSITION_SCALE = 1000;
  static constexpr uint8_t LOST_SIDE_UNKNOWN = 0;
  static constexpr uint8_t LOST_SIDE_LEFT = 1;
  static constexpr uint8_t LOST_SIDE_RIGHT = 2;

  using Channels = std::array<LibXR::GPIO*, MAX_CHANNEL_COUNT>;

  struct Sample
  {
    uint8_t raw_mask = 0;
    uint8_t active_mask = 0;
    uint8_t changed_mask = 0;
    uint8_t channel_count = 0;
    uint8_t active_count = 0;
    uint8_t line_detected = 0;
    uint8_t line_lost = 0;
    uint8_t lost_side = LOST_SIDE_UNKNOWN;
    int16_t weighted_position = 0;
    int16_t position = 0;
    int16_t remembered_position = 0;
    uint32_t lost_count = 0;
    uint32_t sequence = 0;
    std::array<uint8_t, MAX_CHANNEL_COUNT> raw = {};
    std::array<uint8_t, MAX_CHANNEL_COUNT> active = {};
  };

  explicit GreySensor(Channels channels, bool active_low = false,
                      size_t channel_count = MAX_CHANNEL_COUNT);

  LibXR::ErrorCode Initialize(LibXR::GPIO::Pull pull = LibXR::GPIO::Pull::NONE);
  Sample Read();
  uint8_t ReadRawMask() const;
  uint8_t ReadActiveMask() const;
  int16_t ReadPosition();
  size_t ChannelCount() const;

 private:
  static uint8_t BuildBit(size_t channel);
  static int16_t BuildPosition(size_t channel, size_t channel_count);
  static uint8_t GetLostSide(int16_t position);

  Sample ReadDigital() const;
  void UpdatePositionState(Sample& sample);

  Channels channels_ = {};
  size_t channel_count_ = 0;
  bool active_low_ = false;
  uint8_t last_active_mask_ = 0;
  int16_t remembered_position_ = 0;
  uint32_t lost_count_ = 0;
  uint32_t sequence_ = 0;
  bool has_position_memory_ = false;
  bool has_sampled_ = false;
};
