#pragma once

#include <cstdint>

#include "GreySensor.hpp"
#include "app_framework.hpp"

class LineSensors
{
 public:
  explicit LineSensors(LibXR::HardwareContainer& hw);

  bool Initialize();
  GreySensor::Sample Read();
  uint8_t ReadBlackMask() const;
  int16_t ReadPosition();

 private:
  GreySensor grey_sensor_;
};
