#include "quadrature_encoder.hpp"

namespace
{
constexpr int8_t TRANSITION_DELTA[16] = {
    0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0,
};
}  // namespace

QuadratureEncoder::QuadratureEncoder(LibXR::GPIO& phase_a, LibXR::GPIO& phase_b,
                                     bool reversed)
    : phase_a_(phase_a), phase_b_(phase_b), reversed_(reversed)
{
}

uint8_t QuadratureEncoder::ReadState() const
{
  return static_cast<uint8_t>((phase_a_.Read() ? 2U : 0U) | (phase_b_.Read() ? 1U : 0U));
}

LibXR::ErrorCode QuadratureEncoder::Initialize()
{
  constexpr LibXR::GPIO::Configuration INPUT_CONFIG = {LibXR::GPIO::Direction::INPUT,
                                                       LibXR::GPIO::Pull::NONE};
  constexpr LibXR::GPIO::Configuration INTERRUPT_CONFIG = {
      LibXR::GPIO::Direction::FALL_RISING_INTERRUPT, LibXR::GPIO::Pull::NONE};

  LibXR::ErrorCode result = phase_a_.SetConfig(INPUT_CONFIG);
  if (result != LibXR::ErrorCode::OK)
  {
    return result;
  }
  result = phase_b_.SetConfig(INPUT_CONFIG);
  if (result != LibXR::ErrorCode::OK)
  {
    return result;
  }

  previous_state_.store(ReadState(), std::memory_order_relaxed);
  phase_a_callback_ = LibXR::GPIO::Callback::Create(OnEdgeStatic, this);
  phase_b_callback_ = LibXR::GPIO::Callback::Create(OnEdgeStatic, this);
  (void)phase_a_.RegisterCallback(phase_a_callback_);
  (void)phase_b_.RegisterCallback(phase_b_callback_);

  result = phase_a_.SetConfig(INTERRUPT_CONFIG);
  if (result != LibXR::ErrorCode::OK)
  {
    return result;
  }
  return phase_b_.SetConfig(INTERRUPT_CONFIG);
}

void QuadratureEncoder::OnEdgeStatic(bool in_isr, QuadratureEncoder* encoder)
{
  UNUSED(in_isr);
  encoder->OnEdge();
}

void QuadratureEncoder::OnEdge()
{
  const uint8_t current_state = ReadState();
  const uint8_t previous_state =
      previous_state_.exchange(current_state, std::memory_order_relaxed);
  int32_t step = TRANSITION_DELTA[(previous_state << 2U) | current_state];

  if (reversed_)
  {
    step = -step;
  }
  if (step != 0)
  {
    delta_.fetch_add(step, std::memory_order_relaxed);
    total_.fetch_add(step, std::memory_order_relaxed);
  }
}

int32_t QuadratureEncoder::TakeDelta()
{
  return delta_.exchange(0, std::memory_order_relaxed);
}

int32_t QuadratureEncoder::GetTotal() const
{
  return total_.load(std::memory_order_relaxed);
}
