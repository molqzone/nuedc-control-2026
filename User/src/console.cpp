#include "console.hpp"

#include <cmath>

#include "libxr.hpp"
#include "motion_control.hpp"

namespace App
{

Console::Console(MotionControl& motion_control) : motion_control_(motion_control) {}

void Console::PrintHelp() const
{
  LibXR::STDIO::Printf<
      "Motor commands: g=line, w/s/a/d=manual, x/space=stop, +/-=speed, "
      "t<deg><enter>=turn.\r\n">();
}

void Console::HandleByte(uint8_t byte)
{
  if (!reading_turn_)
  {
    if (byte == 't' || byte == 'T')
    {
      reading_turn_ = true;
      turn_length_ = 0;
      return;
    }
    motion_control_.HandleCommand(byte);
    return;
  }

  if (byte == 'x' || byte == 'X' || byte == ' ' || byte == 0x1BU)
  {
    CancelTurnInput(true);
    return;
  }
  if (byte == 'g' || byte == 'G')
  {
    CancelTurnInput(false);
    motion_control_.HandleCommand(byte);
    return;
  }
  if (byte == '\r' || byte == '\n')
  {
    FinishTurnInput();
    return;
  }
  if (byte == '\b' || byte == 0x7FU)
  {
    if (turn_length_ > 0U)
    {
      turn_length_--;
    }
    return;
  }

  if (turn_length_ >= TURN_BUFFER_SIZE - 1U)
  {
    LibXR::STDIO::Printf<"Turn command too long.\r\n">();
    CancelTurnInput(false);
    return;
  }

  turn_buffer_[turn_length_++] = static_cast<char>(byte);
}

bool Console::ParseSignedFloat(const char* text, uint8_t length, float& value)
{
  if (text == nullptr || length == 0U)
  {
    return false;
  }

  uint8_t index = 0;
  float sign = 1.0F;
  if (text[index] == '+' || text[index] == '-')
  {
    sign = text[index] == '-' ? -1.0F : 1.0F;
    index++;
  }

  bool has_digit = false;
  bool has_decimal_point = false;
  float result = 0.0F;
  float decimal_scale = 0.1F;
  for (; index < length; index++)
  {
    const char character = text[index];
    if (character >= '0' && character <= '9')
    {
      has_digit = true;
      const float digit = static_cast<float>(character - '0');
      if (has_decimal_point)
      {
        result += digit * decimal_scale;
        decimal_scale *= 0.1F;
      }
      else
      {
        result = result * 10.0F + digit;
      }
      continue;
    }
    if (character == '.' && !has_decimal_point)
    {
      has_decimal_point = true;
      continue;
    }
    return false;
  }

  value = result * sign;
  return has_digit && std::isfinite(value);
}

void Console::CancelTurnInput(bool stop_vehicle)
{
  reading_turn_ = false;
  turn_length_ = 0;
  if (stop_vehicle)
  {
    motion_control_.HandleCommand('x');
  }
}

void Console::FinishTurnInput()
{
  float delta_deg = 0.0F;
  if (!ParseSignedFloat(turn_buffer_, turn_length_, delta_deg))
  {
    LibXR::STDIO::Printf<"Invalid turn command.\r\n">();
    CancelTurnInput(false);
    return;
  }

  CancelTurnInput(false);
  if (!motion_control_.StartRelativeTurn(delta_deg))
  {
    LibXR::STDIO::Printf<"Turn rejected: use -180..180 deg and wait for IMU.\r\n">();
    return;
  }
  LibXR::STDIO::Printf<"Turn accepted: %.1f deg.\r\n">(delta_deg);
}

}  // namespace App
