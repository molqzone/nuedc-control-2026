#pragma once

#include <array>
#include <cstdint>

#include "gpio.hpp"

namespace App
{

class LineSensorArray
{
 public:
  static constexpr size_t CHANNEL_COUNT = 8;
  using Channels = std::array<LibXR::GPIO*, CHANNEL_COUNT>;

  explicit LineSensorArray(Channels channels) : channels_(channels) {}

  LibXR::ErrorCode Initialize()
  {
    constexpr LibXR::GPIO::Configuration INPUT_PULL_UP = {
        LibXR::GPIO::Direction::INPUT, LibXR::GPIO::Pull::UP};

    LibXR::ErrorCode result = LibXR::ErrorCode::OK;
    for (LibXR::GPIO* channel : channels_)
    {
      ASSERT(channel != nullptr);
      const LibXR::ErrorCode current = channel->SetConfig(INPUT_PULL_UP);
      if (result == LibXR::ErrorCode::OK && current != LibXR::ErrorCode::OK)
      {
        result = current;
      }
    }
    return result;
  }

  uint8_t ReadBlackMask() const
  {
    uint8_t black_mask = 0;
    for (size_t index = 0; index < channels_.size(); index++)
    {
      if (!channels_[index]->Read())
      {
        black_mask |= static_cast<uint8_t>(1U << index);
      }
    }
    return black_mask;
  }

 private:
  Channels channels_;
};

}  // namespace App
