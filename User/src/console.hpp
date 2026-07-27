#pragma once

#include <cstdint>

namespace App
{

class MotionControl;

class Console
{
 public:
  explicit Console(MotionControl& motion_control);

  void PrintHelp() const;
  void HandleByte(uint8_t byte);

 private:
  static constexpr uint8_t TURN_BUFFER_SIZE = 16;

  static bool ParseSignedFloat(const char* text, uint8_t length, float& value);
  void CancelTurnInput(bool stop_vehicle);
  void FinishTurnInput();

  MotionControl& motion_control_;
  char turn_buffer_[TURN_BUFFER_SIZE]{};
  uint8_t turn_length_ = 0;
  bool reading_turn_ = false;
};

}  // namespace App
