#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Framebuffer surface compositor with dirty-frame publishing.
constructor_args:
  - config:
      expr: DisplaySurface::Config{128, 64, "display_frame", 33}
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <cstddef>
#include <cstdint>

#include "MonoCanvas.hpp"
#include "app_framework.hpp"
#include "message.hpp"
#include "timebase.hpp"
#include "timer.hpp"

class DisplaySurface : public LibXR::Application
{
 public:
  struct Config
  {
    std::uint16_t width;
    std::uint16_t height;
    const char* frame_topic_name;
    std::uint32_t refresh_interval_ms;
  };

  enum class PixelFormat : std::uint8_t
  {
    MONO_VTILED_LSB = 0,
  };

  struct Frame
  {
    const std::uint8_t* data = nullptr;
    std::uint32_t size = 0;
    std::uint16_t width = 0;
    std::uint16_t height = 0;
    std::uint16_t pitch = 0;
    std::uint16_t x = 0;
    std::uint16_t y = 0;
    std::uint16_t dirty_width = 0;
    std::uint16_t dirty_height = 0;
    std::uint32_t sequence = 0;
    PixelFormat pixel_format = PixelFormat::MONO_VTILED_LSB;
    bool full_update = true;
  };

  DisplaySurface(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                 Config config)
      : width_(CheckedDimension(config.width)),
        height_(CheckedDimension(config.height)),
        pages_(CalculatePages(height_)),
        framebuffer_size_(CalculateFramebufferSize(width_, pages_)),
        framebuffer_(new std::uint8_t[framebuffer_size_]{}),
        frame_topic_(LibXR::Topic::CreateTopic<Frame>(config.frame_topic_name)),
        canvas_(framebuffer_, framebuffer_size_, width_, height_, width_),
        refresh_interval_ms_(config.refresh_interval_ms == 0U
                                 ? 1U
                                 : config.refresh_interval_ms),
        refresh_timer_(LibXR::Timer::CreateTask<DisplaySurface*>(OnRefreshTimer, this,
                                                                 refresh_interval_ms_))
  {
    UNUSED(hw);
    canvas_.Clear(false);
    PublishFullFrame();
    LibXR::Timer::Add(refresh_timer_);
    LibXR::Timer::Start(refresh_timer_);
    app.Register(*this);
  }

  MonoCanvas& GetCanvas() { return canvas_; }
  const MonoCanvas& GetCanvas() const { return canvas_; }
  std::uint16_t Width() const { return width_; }
  std::uint16_t Height() const { return height_; }
  std::uint16_t Pages() const { return pages_; }
  std::size_t FramebufferSize() const { return framebuffer_size_; }

  bool PublishDirtyFrame()
  {
    MonoCanvas::Rect dirty{};
    bool full_update = false;
    if (!canvas_.GetDirtyRect(dirty, full_update))
    {
      return false;
    }

    PublishFrame(dirty, full_update);
    canvas_.ClearDirty();
    return true;
  }

  void PublishFullFrame()
  {
    PublishFrame({0, 0, width_, height_}, true);
    canvas_.ClearDirty();
  }

  void OnMonitor() override {}

 private:
  static std::uint16_t CheckedDimension(std::uint16_t value)
  {
    ASSERT(value != 0U);
    return value;
  }

  static std::uint16_t CalculatePages(std::uint16_t height)
  {
    return static_cast<std::uint16_t>((height + 7U) >> 3U);
  }

  static std::size_t CalculateFramebufferSize(std::uint16_t width, std::uint16_t pages)
  {
    return static_cast<std::size_t>(width) * pages;
  }

  static void OnRefreshTimer(DisplaySurface* self)
  {
    if (self == nullptr)
    {
      return;
    }

    (void)self->PublishDirtyFrame();
  }

  void PublishFrame(MonoCanvas::Rect dirty, bool full_update)
  {
    frame_.data = framebuffer_;
    frame_.size = static_cast<std::uint32_t>(framebuffer_size_);
    frame_.width = width_;
    frame_.height = height_;
    frame_.pitch = width_;
    frame_.x = dirty.x;
    frame_.y = dirty.y;
    frame_.dirty_width = dirty.width;
    frame_.dirty_height = dirty.height;
    frame_.sequence = sequence_++;
    frame_.pixel_format = PixelFormat::MONO_VTILED_LSB;
    frame_.full_update = full_update;
    frame_topic_.Publish(frame_);
  }

  std::uint16_t width_ = 0;
  std::uint16_t height_ = 0;
  std::uint16_t pages_ = 0;
  std::size_t framebuffer_size_ = 0;
  std::uint8_t* framebuffer_ = nullptr;
  LibXR::Topic frame_topic_;
  MonoCanvas canvas_;
  Frame frame_{};
  std::uint32_t refresh_interval_ms_ = 500;
  LibXR::Timer::TimerHandle refresh_timer_ = nullptr;
  std::uint32_t sequence_ = 0;
};
