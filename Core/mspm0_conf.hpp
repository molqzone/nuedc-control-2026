#pragma once

namespace LibXR
{

// Project-side extension point for MSPM0 helpers. The current template does not
// instantiate PWM resources, so this placeholder keeps MSPM0 driver headers
// buildable. If you later use MSPM0_PWM_CH(...), add the required
// *_INST_CLK_DIV and *_INST_CLK_PSC constants here.
struct MSPM0Config
{
};

}  // namespace LibXR
