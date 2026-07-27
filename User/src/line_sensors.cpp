#include "line_sensors.hpp"

#include "ti_msp_dl_config.h"

namespace App
{

LineSensors::LineSensors()
    : line_ad1_(LINE_A_PORT, LINE_A_AD1_PIN, LINE_A_AD1_IOMUX),
      line_ad2_(LINE_B_PORT, LINE_B_AD2_PIN, LINE_B_AD2_IOMUX),
      line_ad3_(LINE_B_PORT, LINE_B_AD3_PIN, LINE_B_AD3_IOMUX),
      line_ad4_(LINE_B_PORT, LINE_B_AD4_PIN, LINE_B_AD4_IOMUX),
      line_ad5_(LINE_B_PORT, LINE_B_AD5_PIN, LINE_B_AD5_IOMUX),
      line_ad6_(LINE_A_PORT, LINE_A_AD6_PIN, LINE_A_AD6_IOMUX),
      line_ad7_(LINE_A_PORT, LINE_A_AD7_PIN, LINE_A_AD7_IOMUX),
      line_ad8_(LINE_A_PORT, LINE_A_AD8_PIN, LINE_A_AD8_IOMUX),
      grey_sensor_({{&line_ad1_, &line_ad2_, &line_ad3_, &line_ad4_, &line_ad5_,
                     &line_ad6_, &line_ad7_, &line_ad8_}},
                   true)
{
}

bool LineSensors::Initialize()
{
  return grey_sensor_.Initialize(LibXR::GPIO::Pull::UP) == LibXR::ErrorCode::OK;
}

GreySensor::Sample LineSensors::Read() { return grey_sensor_.Read(); }

uint8_t LineSensors::ReadBlackMask() const { return grey_sensor_.ReadActiveMask(); }

int16_t LineSensors::ReadPosition() { return grey_sensor_.ReadPosition(); }

}  // namespace App
