#pragma once

#include <atomic>
#include <cstdint>

#include "gpio.hpp"

class QuadratureEncoder
{
 public:
  QuadratureEncoder(LibXR::GPIO& phase_a, LibXR::GPIO& phase_b, bool reversed = false);

  LibXR::ErrorCode Initialize();
  int32_t TakeDelta();
  int32_t GetTotal() const;

 private:
  static void OnEdgeStatic(bool in_isr, QuadratureEncoder* encoder);
  void OnEdge();
  uint8_t ReadState() const;

  LibXR::GPIO& phase_a_;
  LibXR::GPIO& phase_b_;
  bool reversed_;
  std::atomic<uint8_t> previous_state_{0};
  std::atomic<int32_t> delta_{0};
  std::atomic<int32_t> total_{0};
  LibXR::GPIO::Callback phase_a_callback_;
  LibXR::GPIO::Callback phase_b_callback_;
};
