#pragma once

#include <cstddef>
#include <cstdint>

namespace App
{

class MotionControl;

class Zigbee
{
 public:
  enum class ResponseType : uint8_t
  {
    NONE,
    ACK,
    NACK,
  };

  struct Response
  {
    ResponseType type = ResponseType::NONE;
    uint8_t command = 0;
  };

  static constexpr size_t RESPONSE_BUFFER_SIZE = 9;

  explicit Zigbee(MotionControl& motion_control);

  Response FeedByte(uint8_t byte);

  static size_t FormatResponse(const Response& response, char* buffer, size_t capacity);

 private:
  enum class ParserState : uint8_t
  {
    IDLE,
    READING_TURN,
    DISCARDING_TURN,
  };

  static constexpr uint8_t TURN_BUFFER_SIZE = 16;

  static bool IsSimpleCommand(uint8_t byte);
  static uint8_t NormalizeCommand(uint8_t byte);
  static bool ParseSignedFloat(const char* text, uint8_t length, float& value);

  void ResetTurnInput();
  Response FinishTurnInput();

  MotionControl& motion_control_;
  char turn_buffer_[TURN_BUFFER_SIZE]{};
  uint8_t turn_length_ = 0;
  ParserState parser_state_ = ParserState::IDLE;
};

}  // namespace App
