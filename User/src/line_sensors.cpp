#include "line_sensors.hpp"

LineSensors::LineSensors(LibXR::HardwareContainer& hw)
    : grey_sensor_({{hw.FindOrExit<LibXR::GPIO>({"line_ad1"}),
                     hw.FindOrExit<LibXR::GPIO>({"line_ad2"}),
                     hw.FindOrExit<LibXR::GPIO>({"line_ad3"}),
                     hw.FindOrExit<LibXR::GPIO>({"line_ad4"}),
                     hw.FindOrExit<LibXR::GPIO>({"line_ad5"}),
                     hw.FindOrExit<LibXR::GPIO>({"line_ad6"}),
                     hw.FindOrExit<LibXR::GPIO>({"line_ad7"}),
                     hw.FindOrExit<LibXR::GPIO>({"line_ad8"})}},
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
