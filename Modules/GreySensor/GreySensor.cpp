#include "GreySensor.hpp"

uint8_t GreySensor::BuildBit(size_t channel)
{
  return static_cast<uint8_t>(1U << channel);
}

int16_t GreySensor::BuildPosition(size_t channel, size_t channel_count)
{
  const int32_t centered =
      (static_cast<int32_t>(channel) * 2) - static_cast<int32_t>(channel_count - 1);
  return static_cast<int16_t>((centered * POSITION_SCALE) / 2);
}

uint8_t GreySensor::GetLostSide(int16_t position)
{
  if (position < 0)
  {
    return LOST_SIDE_LEFT;
  }
  if (position > 0)
  {
    return LOST_SIDE_RIGHT;
  }
  return LOST_SIDE_UNKNOWN;
}

GreySensor::GreySensor(Channels channels, bool active_low, size_t channel_count)
    : channels_(channels), channel_count_(channel_count), active_low_(active_low)
{
  ASSERT(channel_count_ > 0);
  ASSERT(channel_count_ <= MAX_CHANNEL_COUNT);
}

LibXR::ErrorCode GreySensor::Initialize(LibXR::GPIO::Pull pull)
{
  const LibXR::GPIO::Configuration input_config = {LibXR::GPIO::Direction::INPUT, pull};
  LibXR::ErrorCode result = LibXR::ErrorCode::OK;

  for (size_t i = 0; i < channel_count_; i++)
  {
    ASSERT(channels_[i] != nullptr);
    const LibXR::ErrorCode current = channels_[i]->SetConfig(input_config);
    if (result == LibXR::ErrorCode::OK && current != LibXR::ErrorCode::OK)
    {
      result = current;
    }
  }

  last_active_mask_ = ReadActiveMask();
  return result;
}

GreySensor::Sample GreySensor::ReadDigital() const
{
  Sample sample;
  sample.channel_count = static_cast<uint8_t>(channel_count_);

  for (size_t i = 0; i < channel_count_; i++)
  {
    const bool raw_high = channels_[i]->Read();
    const bool active = active_low_ ? !raw_high : raw_high;

    sample.raw[i] = raw_high ? 1U : 0U;
    sample.active[i] = active ? 1U : 0U;

    if (raw_high)
    {
      sample.raw_mask |= BuildBit(i);
    }
    if (active)
    {
      sample.active_mask |= BuildBit(i);
      sample.active_count++;
    }
  }

  return sample;
}

void GreySensor::UpdatePositionState(Sample& sample)
{
  int32_t weighted_sum = 0;

  for (size_t i = 0; i < channel_count_; i++)
  {
    if (sample.active[i] != 0U)
    {
      weighted_sum += BuildPosition(i, channel_count_);
    }
  }

  if (sample.active_count != 0U)
  {
    sample.line_detected = 1;
    sample.line_lost = 0;
    sample.weighted_position = static_cast<int16_t>(weighted_sum / sample.active_count);
    sample.position = sample.weighted_position;

    remembered_position_ = sample.position;
    has_position_memory_ = true;
    lost_count_ = 0;
  }
  else
  {
    sample.line_detected = 0;
    sample.line_lost = 1;
    lost_count_++;

    sample.position = has_position_memory_ ? remembered_position_ : 0;
    sample.lost_side =
        has_position_memory_ ? GetLostSide(remembered_position_) : LOST_SIDE_UNKNOWN;
    sample.lost_count = lost_count_;
  }

  sample.remembered_position = has_position_memory_ ? remembered_position_ : 0;
}

GreySensor::Sample GreySensor::Read()
{
  Sample sample = ReadDigital();
  UpdatePositionState(sample);
  sample.changed_mask =
      has_sampled_ ? static_cast<uint8_t>(sample.active_mask ^ last_active_mask_) : 0;
  sample.sequence = sequence_++;
  last_active_mask_ = sample.active_mask;
  has_sampled_ = true;
  return sample;
}

uint8_t GreySensor::ReadRawMask() const { return ReadDigital().raw_mask; }

uint8_t GreySensor::ReadActiveMask() const { return ReadDigital().active_mask; }

int16_t GreySensor::ReadPosition() { return Read().position; }

size_t GreySensor::ChannelCount() const { return channel_count_; }
