#include "zigbee.hpp"

#include <cmath>

#include "motion_control.hpp"

namespace App
{

Zigbee::Zigbee(MotionControl& motion_control) : motion_control_(motion_control) {}

Zigbee::Response Zigbee::FeedByte(uint8_t byte)
{
  if (parser_state_ == ParserState::DISCARDING_TURN)
  {
    if (byte == 'x' || byte == 'X')
    {
      ResetTurnInput();
      motion_control_.HandleCommand('x');
      return {ResponseType::ACK, 'x'};
    }
    if (byte == '\r' || byte == '\n')
    {
      ResetTurnInput();
    }
    return {};
  }

  if (parser_state_ == ParserState::IDLE)
  {
    if (byte == 't' || byte == 'T')
    {
      parser_state_ = ParserState::READING_TURN;
      turn_length_ = 0;
      return {};
    }

    if (IsSimpleCommand(byte))
    {
      const uint8_t command = NormalizeCommand(byte);
      motion_control_.HandleCommand(command);
      return {ResponseType::ACK, command};
    }

    if (byte >= '!' && byte <= '~')
    {
      return {ResponseType::NACK, byte};
    }
    return {};
  }

  if (byte == 'x' || byte == 'X')
  {
    ResetTurnInput();
    motion_control_.HandleCommand('x');
    return {ResponseType::ACK, 'x'};
  }
  if (byte == '\r' || byte == '\n')
  {
    return FinishTurnInput();
  }
  if (byte == '\b' || byte == 0x7FU)
  {
    if (turn_length_ > 0U)
    {
      turn_length_--;
    }
    return {};
  }

  if (turn_length_ >= TURN_BUFFER_SIZE - 1U)
  {
    parser_state_ = ParserState::DISCARDING_TURN;
    turn_length_ = 0;
    return {ResponseType::NACK, 't'};
  }

  turn_buffer_[turn_length_++] = static_cast<char>(byte);
  return {};
}

size_t Zigbee::FormatResponse(const Response& response, char* buffer, size_t capacity)
{
  if (response.type == ResponseType::NONE)
  {
    return 0;
  }

  const bool is_ack = response.type == ResponseType::ACK;
  const size_t length = is_ack ? 7U : 8U;
  if (buffer == nullptr || capacity < length + 1U)
  {
    return 0;
  }

  size_t index = 0;
  if (is_ack)
  {
    buffer[index++] = 'A';
    buffer[index++] = 'C';
    buffer[index++] = 'K';
  }
  else
  {
    buffer[index++] = 'N';
    buffer[index++] = 'A';
    buffer[index++] = 'C';
    buffer[index++] = 'K';
  }
  buffer[index++] = ':';
  buffer[index++] = static_cast<char>(response.command);
  buffer[index++] = '\r';
  buffer[index++] = '\n';
  buffer[index] = '\0';
  return index;
}

bool Zigbee::IsSimpleCommand(uint8_t byte)
{
  switch (byte)
  {
    case 'g':
    case 'G':
    case 'w':
    case 'W':
    case 's':
    case 'S':
    case 'a':
    case 'A':
    case 'd':
    case 'D':
    case 'x':
    case 'X':
    case '+':
    case '-':
      return true;
    default:
      return false;
  }
}

uint8_t Zigbee::NormalizeCommand(uint8_t byte)
{
  if (byte >= 'A' && byte <= 'Z')
  {
    return static_cast<uint8_t>(byte - 'A' + 'a');
  }
  return byte;
}

bool Zigbee::ParseSignedFloat(const char* text, uint8_t length, float& value)
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

void Zigbee::ResetTurnInput()
{
  parser_state_ = ParserState::IDLE;
  turn_length_ = 0;
}

Zigbee::Response Zigbee::FinishTurnInput()
{
  float delta_deg = 0.0F;
  const bool parsed = ParseSignedFloat(turn_buffer_, turn_length_, delta_deg);
  ResetTurnInput();
  if (!parsed || !motion_control_.StartRelativeTurn(delta_deg))
  {
    return {ResponseType::NACK, 't'};
  }
  return {ResponseType::ACK, 't'};
}

}  // namespace App
