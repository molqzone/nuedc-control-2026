#pragma once

#include <cstdint>

#include "GreySensor.hpp"
#include "mspm0_gpio.hpp"

namespace App
{

class LineSensors
{
 public:
  LineSensors();

  bool Initialize();
  GreySensor::Sample Read();
  uint8_t ReadBlackMask() const;
  int16_t ReadPosition();

 private:
  LibXR::MSPM0GPIO line_ad1_;
  LibXR::MSPM0GPIO line_ad2_;
  LibXR::MSPM0GPIO line_ad3_;
  LibXR::MSPM0GPIO line_ad4_;
  LibXR::MSPM0GPIO line_ad5_;
  LibXR::MSPM0GPIO line_ad6_;
  LibXR::MSPM0GPIO line_ad7_;
  LibXR::MSPM0GPIO line_ad8_;
  GreySensor grey_sensor_;
};

}  // namespace App
