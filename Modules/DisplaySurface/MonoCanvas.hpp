#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace display_detail
{

inline constexpr std::uint8_t FONT_5X7[] = {
    0x00, 0x00, 0x00, 0x00, 0x00,  // space
    0x00, 0x00, 0x5F, 0x00, 0x00,  // !
    0x00, 0x07, 0x00, 0x07, 0x00,  // "
    0x14, 0x7F, 0x14, 0x7F, 0x14,  // #
    0x24, 0x2A, 0x7F, 0x2A, 0x12,  // $
    0x23, 0x13, 0x08, 0x64, 0x62,  // %
    0x36, 0x49, 0x55, 0x22, 0x50,  // &
    0x00, 0x05, 0x03, 0x00, 0x00,  // '
    0x00, 0x1C, 0x22, 0x41, 0x00,  // (
    0x00, 0x41, 0x22, 0x1C, 0x00,  // )
    0x14, 0x08, 0x3E, 0x08, 0x14,  // *
    0x08, 0x08, 0x3E, 0x08, 0x08,  // +
    0x00, 0x50, 0x30, 0x00, 0x00,  // ,
    0x08, 0x08, 0x08, 0x08, 0x08,  // -
    0x00, 0x60, 0x60, 0x00, 0x00,  // .
    0x20, 0x10, 0x08, 0x04, 0x02,  // /
    0x3E, 0x51, 0x49, 0x45, 0x3E,  // 0
    0x00, 0x42, 0x7F, 0x40, 0x00,  // 1
    0x42, 0x61, 0x51, 0x49, 0x46,  // 2
    0x21, 0x41, 0x45, 0x4B, 0x31,  // 3
    0x18, 0x14, 0x12, 0x7F, 0x10,  // 4
    0x27, 0x45, 0x45, 0x45, 0x39,  // 5
    0x3C, 0x4A, 0x49, 0x49, 0x30,  // 6
    0x01, 0x71, 0x09, 0x05, 0x03,  // 7
    0x36, 0x49, 0x49, 0x49, 0x36,  // 8
    0x06, 0x49, 0x49, 0x29, 0x1E,  // 9
    0x00, 0x36, 0x36, 0x00, 0x00,  // :
    0x00, 0x56, 0x36, 0x00, 0x00,  // ;
    0x08, 0x14, 0x22, 0x41, 0x00,  // <
    0x14, 0x14, 0x14, 0x14, 0x14,  // =
    0x00, 0x41, 0x22, 0x14, 0x08,  // >
    0x02, 0x01, 0x51, 0x09, 0x06,  // ?
    0x32, 0x49, 0x79, 0x41, 0x3E,  // @
    0x7E, 0x11, 0x11, 0x11, 0x7E,  // A
    0x7F, 0x49, 0x49, 0x49, 0x36,  // B
    0x3E, 0x41, 0x41, 0x41, 0x22,  // C
    0x7F, 0x41, 0x41, 0x22, 0x1C,  // D
    0x7F, 0x49, 0x49, 0x49, 0x41,  // E
    0x7F, 0x09, 0x09, 0x09, 0x01,  // F
    0x3E, 0x41, 0x49, 0x49, 0x7A,  // G
    0x7F, 0x08, 0x08, 0x08, 0x7F,  // H
    0x00, 0x41, 0x7F, 0x41, 0x00,  // I
    0x20, 0x40, 0x41, 0x3F, 0x01,  // J
    0x7F, 0x08, 0x14, 0x22, 0x41,  // K
    0x7F, 0x40, 0x40, 0x40, 0x40,  // L
    0x7F, 0x02, 0x0C, 0x02, 0x7F,  // M
    0x7F, 0x04, 0x08, 0x10, 0x7F,  // N
    0x3E, 0x41, 0x41, 0x41, 0x3E,  // O
    0x7F, 0x09, 0x09, 0x09, 0x06,  // P
    0x3E, 0x41, 0x51, 0x21, 0x5E,  // Q
    0x7F, 0x09, 0x19, 0x29, 0x46,  // R
    0x46, 0x49, 0x49, 0x49, 0x31,  // S
    0x01, 0x01, 0x7F, 0x01, 0x01,  // T
    0x3F, 0x40, 0x40, 0x40, 0x3F,  // U
    0x1F, 0x20, 0x40, 0x20, 0x1F,  // V
    0x3F, 0x40, 0x38, 0x40, 0x3F,  // W
    0x63, 0x14, 0x08, 0x14, 0x63,  // X
    0x07, 0x08, 0x70, 0x08, 0x07,  // Y
    0x61, 0x51, 0x49, 0x45, 0x43,  // Z
    0x00, 0x7F, 0x41, 0x41, 0x00,  // [
    0x02, 0x04, 0x08, 0x10, 0x20,  // backslash
    0x00, 0x41, 0x41, 0x7F, 0x00,  // ]
    0x04, 0x02, 0x01, 0x02, 0x04,  // ^
    0x40, 0x40, 0x40, 0x40, 0x40,  // _
    0x00, 0x01, 0x02, 0x04, 0x00,  // `
    0x20, 0x54, 0x54, 0x54, 0x78,  // a
    0x7F, 0x48, 0x44, 0x44, 0x38,  // b
    0x38, 0x44, 0x44, 0x44, 0x20,  // c
    0x38, 0x44, 0x44, 0x48, 0x7F,  // d
    0x38, 0x54, 0x54, 0x54, 0x18,  // e
    0x08, 0x7E, 0x09, 0x01, 0x02,  // f
    0x0C, 0x52, 0x52, 0x52, 0x3E,  // g
    0x7F, 0x08, 0x04, 0x04, 0x78,  // h
    0x00, 0x44, 0x7D, 0x40, 0x00,  // i
    0x20, 0x40, 0x44, 0x3D, 0x00,  // j
    0x7F, 0x10, 0x28, 0x44, 0x00,  // k
    0x00, 0x41, 0x7F, 0x40, 0x00,  // l
    0x7C, 0x04, 0x18, 0x04, 0x78,  // m
    0x7C, 0x08, 0x04, 0x04, 0x78,  // n
    0x38, 0x44, 0x44, 0x44, 0x38,  // o
    0x7C, 0x14, 0x14, 0x14, 0x08,  // p
    0x08, 0x14, 0x14, 0x18, 0x7C,  // q
    0x7C, 0x08, 0x04, 0x04, 0x08,  // r
    0x48, 0x54, 0x54, 0x54, 0x20,  // s
    0x04, 0x3F, 0x44, 0x40, 0x20,  // t
    0x3C, 0x40, 0x40, 0x20, 0x7C,  // u
    0x1C, 0x20, 0x40, 0x20, 0x1C,  // v
    0x3C, 0x40, 0x30, 0x40, 0x3C,  // w
    0x44, 0x28, 0x10, 0x28, 0x44,  // x
    0x0C, 0x50, 0x50, 0x50, 0x3C,  // y
    0x44, 0x64, 0x54, 0x4C, 0x44,  // z
    0x00, 0x08, 0x36, 0x41, 0x00,  // {
    0x00, 0x00, 0x7F, 0x00, 0x00,  // |
    0x00, 0x41, 0x36, 0x08, 0x00,  // }
    0x10, 0x08, 0x08, 0x10, 0x08,  // ~
    0x00, 0x06, 0x09, 0x09, 0x06,  // del fallback
};

}  // namespace display_detail

class MonoCanvas
{
 public:
  enum class Color : std::uint8_t
  {
    CLEAR = 0,
    SET = 1,
    INVERT = 2,
  };

  struct Rect
  {
    std::uint16_t x = 0;
    std::uint16_t y = 0;
    std::uint16_t width = 0;
    std::uint16_t height = 0;
  };

  struct Font
  {
    const std::uint8_t* data = nullptr;
    std::size_t glyph_count = 0;
    std::uint8_t first_char = 0x20;
    std::uint8_t width = 5;
    std::uint8_t height = 7;
    std::uint8_t stride = 5;
    std::uint8_t spacing = 1;
  };

  static constexpr Font DefaultFont()
  {
    return {display_detail::FONT_5X7,
            sizeof(display_detail::FONT_5X7) / 5U,
            0x20,
            5,
            7,
            5,
            1};
  }

  MonoCanvas(std::uint8_t* data, std::size_t size, std::uint16_t width,
             std::uint16_t height, std::uint16_t pitch)
      : data_(data),
        size_(size),
        width_(width),
        height_(height),
        pitch_(pitch),
        pages_(static_cast<std::uint16_t>((height + 7U) >> 3U))
  {
  }

  std::uint16_t Width() const { return width_; }
  std::uint16_t Height() const { return height_; }
  std::uint16_t Pitch() const { return pitch_; }
  std::uint16_t Pages() const { return pages_; }
  std::size_t Size() const { return size_; }
  std::uint8_t* Data() { return data_; }
  const std::uint8_t* Data() const { return data_; }

  bool IsValid() const
  {
    return data_ != nullptr && width_ != 0U && height_ != 0U && pitch_ >= width_ &&
           size_ >= RequiredSize();
  }

  bool HasDirty() const { return dirty_; }

  bool GetDirtyRect(Rect& rect, bool& full_update) const
  {
    if (!dirty_)
    {
      return false;
    }

    full_update = full_dirty_;
    rect = full_dirty_ ? Rect{0, 0, width_, height_}
                       : Rect{dirty_x0_, dirty_y0_,
                              static_cast<std::uint16_t>(dirty_x1_ - dirty_x0_),
                              static_cast<std::uint16_t>(dirty_y1_ - dirty_y0_)};
    return true;
  }

  void ClearDirty()
  {
    dirty_ = false;
    full_dirty_ = false;
    dirty_x0_ = 0;
    dirty_y0_ = 0;
    dirty_x1_ = 0;
    dirty_y1_ = 0;
  }

  void MarkFullDirty()
  {
    if (!IsValid())
    {
      return;
    }

    dirty_ = true;
    full_dirty_ = true;
    dirty_x0_ = 0;
    dirty_y0_ = 0;
    dirty_x1_ = width_;
    dirty_y1_ = height_;
  }

  void Clear(bool on = false)
  {
    if (!IsValid())
    {
      return;
    }

    std::memset(data_, on ? 0xFF : 0x00, RequiredSize());
    if (on && (height_ & 0x7U) != 0U)
    {
      const auto mask = static_cast<std::uint8_t>((1U << (height_ & 0x7U)) - 1U);
      const std::size_t page_offset = static_cast<std::size_t>(pages_ - 1U) * pitch_;
      for (std::uint16_t x = 0; x < width_; ++x)
      {
        data_[page_offset + x] &= mask;
      }
    }

    MarkFullDirty();
  }

  bool GetPixel(std::uint16_t x, std::uint16_t y) const
  {
    if (!IsValid() || x >= width_ || y >= height_)
    {
      return false;
    }

    return (data_[Index(x, y)] & Mask(y)) != 0U;
  }

  void SetPixel(std::int16_t x, std::int16_t y, Color color = Color::SET)
  {
    if (WritePixel(x, y, color))
    {
      MarkDirty(static_cast<std::uint16_t>(x), static_cast<std::uint16_t>(y), 1, 1);
    }
  }

  void DrawHorizontalLine(std::int16_t x, std::int16_t y, std::int16_t width,
                          Color color = Color::SET)
  {
    FillRect(x, y, width, 1, color);
  }

  void DrawVerticalLine(std::int16_t x, std::int16_t y, std::int16_t height,
                        Color color = Color::SET)
  {
    FillRect(x, y, 1, height, color);
  }

  void DrawLine(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1,
                Color color = Color::SET)
  {
    const std::int16_t dx = Abs(static_cast<std::int16_t>(x1 - x0));
    const std::int16_t sx = x0 < x1 ? 1 : -1;
    const std::int16_t dy =
        static_cast<std::int16_t>(-Abs(static_cast<std::int16_t>(y1 - y0)));
    const std::int16_t sy = y0 < y1 ? 1 : -1;
    std::int16_t err = static_cast<std::int16_t>(dx + dy);

    while (true)
    {
      SetPixel(x0, y0, color);
      if (x0 == x1 && y0 == y1)
      {
        break;
      }

      const std::int16_t e2 = static_cast<std::int16_t>(2 * err);
      if (e2 >= dy)
      {
        err = static_cast<std::int16_t>(err + dy);
        x0 = static_cast<std::int16_t>(x0 + sx);
      }
      if (e2 <= dx)
      {
        err = static_cast<std::int16_t>(err + dx);
        y0 = static_cast<std::int16_t>(y0 + sy);
      }
    }
  }

  void DrawRect(std::int16_t x, std::int16_t y, std::int16_t width, std::int16_t height,
                Color color = Color::SET)
  {
    if (width <= 0 || height <= 0)
    {
      return;
    }

    DrawHorizontalLine(x, y, width, color);
    DrawHorizontalLine(x, static_cast<std::int16_t>(y + height - 1), width, color);
    DrawVerticalLine(x, y, height, color);
    DrawVerticalLine(static_cast<std::int16_t>(x + width - 1), y, height, color);
  }

  void FillRect(std::int16_t x, std::int16_t y, std::int16_t width, std::int16_t height,
                Color color = Color::SET)
  {
    Rect clipped{};
    if (!ClipRect(x, y, width, height, clipped))
    {
      return;
    }

    bool changed = false;
    for (std::uint16_t yy = clipped.y;
         yy < static_cast<std::uint16_t>(clipped.y + clipped.height); ++yy)
    {
      for (std::uint16_t xx = clipped.x;
           xx < static_cast<std::uint16_t>(clipped.x + clipped.width); ++xx)
      {
        changed |= WritePixel(static_cast<std::int16_t>(xx),
                              static_cast<std::int16_t>(yy), color);
      }
    }

    if (changed)
    {
      MarkDirty(clipped.x, clipped.y, clipped.width, clipped.height);
    }
  }

  void InvertArea(std::int16_t x, std::int16_t y, std::int16_t width, std::int16_t height)
  {
    FillRect(x, y, width, height, Color::INVERT);
  }

  void DrawCircle(std::int16_t x0, std::int16_t y0, std::int16_t radius,
                  Color color = Color::SET)
  {
    if (radius < 0)
    {
      return;
    }

    std::int16_t x = radius;
    std::int16_t y = 0;
    std::int16_t err = 0;

    while (x >= y)
    {
      PlotCirclePoints(x0, y0, x, y, color);
      ++y;
      if (err <= 0)
      {
        err = static_cast<std::int16_t>(err + 2 * y + 1);
      }
      if (err > 0)
      {
        --x;
        err = static_cast<std::int16_t>(err - 2 * x + 1);
      }
    }
  }

  void FillCircle(std::int16_t x0, std::int16_t y0, std::int16_t radius,
                  Color color = Color::SET)
  {
    if (radius < 0)
    {
      return;
    }

    std::int16_t x = radius;
    std::int16_t y = 0;
    std::int16_t err = 0;

    while (x >= y)
    {
      DrawVerticalLine(static_cast<std::int16_t>(x0 + x),
                       static_cast<std::int16_t>(y0 - y),
                       static_cast<std::int16_t>(2 * y + 1), color);
      DrawVerticalLine(static_cast<std::int16_t>(x0 - x),
                       static_cast<std::int16_t>(y0 - y),
                       static_cast<std::int16_t>(2 * y + 1), color);
      DrawVerticalLine(static_cast<std::int16_t>(x0 + y),
                       static_cast<std::int16_t>(y0 - x),
                       static_cast<std::int16_t>(2 * x + 1), color);
      DrawVerticalLine(static_cast<std::int16_t>(x0 - y),
                       static_cast<std::int16_t>(y0 - x),
                       static_cast<std::int16_t>(2 * x + 1), color);

      ++y;
      if (err <= 0)
      {
        err = static_cast<std::int16_t>(err + 2 * y + 1);
      }
      if (err > 0)
      {
        --x;
        err = static_cast<std::int16_t>(err - 2 * x + 1);
      }
    }
  }

  bool DrawChar(std::int16_t x, std::int16_t y, char ch, Color color = Color::SET,
                bool transparent = true, Color background = Color::CLEAR,
                Font font = DefaultFont())
  {
    if (!IsValid() || font.data == nullptr || font.width == 0U || font.height == 0U ||
        font.stride == 0U || font.glyph_count == 0U)
    {
      return false;
    }

    std::uint8_t code = static_cast<std::uint8_t>(ch);
    if (code < font.first_char || (code - font.first_char) >= font.glyph_count)
    {
      code = '?';
    }

    const std::size_t glyph_index =
        static_cast<std::size_t>(code - font.first_char) * font.stride;
    bool changed = false;

    for (std::uint8_t col = 0; col < font.width; ++col)
    {
      const std::uint8_t bits = font.data[glyph_index + col];
      for (std::uint8_t row = 0; row < font.height; ++row)
      {
        const bool pixel_on = (bits & (1U << row)) != 0U;
        if (pixel_on)
        {
          changed |= WritePixel(static_cast<std::int16_t>(x + col),
                                static_cast<std::int16_t>(y + row), color);
        }
        else if (!transparent)
        {
          changed |= WritePixel(static_cast<std::int16_t>(x + col),
                                static_cast<std::int16_t>(y + row), background);
        }
      }
    }

    if (!transparent && font.spacing != 0U)
    {
      for (std::uint8_t col = 0; col < font.spacing; ++col)
      {
        for (std::uint8_t row = 0; row < font.height; ++row)
        {
          changed |= WritePixel(static_cast<std::int16_t>(x + font.width + col),
                                static_cast<std::int16_t>(y + row), background);
        }
      }
    }

    if (changed)
    {
      MarkDirtyClipped(x, y, static_cast<std::int16_t>(font.width + font.spacing),
                       font.height);
    }
    return true;
  }

  std::size_t DrawText(std::int16_t x, std::int16_t y, const char* text,
                       Color color = Color::SET, bool transparent = true,
                       Color background = Color::CLEAR, Font font = DefaultFont())
  {
    if (text == nullptr)
    {
      return 0;
    }

    const std::int16_t start_x = x;
    std::size_t count = 0;
    while (*text != '\0')
    {
      if (*text == '\n')
      {
        x = start_x;
        y = static_cast<std::int16_t>(y + font.height + 1U);
        ++text;
        continue;
      }

      if (DrawChar(x, y, *text, color, transparent, background, font))
      {
        ++count;
      }
      x = static_cast<std::int16_t>(x + font.width + font.spacing);
      ++text;
    }

    return count;
  }

  static std::uint16_t TextWidth(const char* text, Font font = DefaultFont())
  {
    if (text == nullptr || *text == '\0')
    {
      return 0;
    }

    std::uint16_t width = 0;
    std::uint16_t line_width = 0;
    while (*text != '\0')
    {
      if (*text == '\n')
      {
        width = Max(width, line_width);
        line_width = 0;
      }
      else
      {
        line_width = static_cast<std::uint16_t>(line_width + font.width + font.spacing);
      }
      ++text;
    }

    width = Max(width, line_width);
    if (width != 0U && font.spacing != 0U)
    {
      width = static_cast<std::uint16_t>(width - font.spacing);
    }
    return width;
  }

 private:
  std::size_t RequiredSize() const { return static_cast<std::size_t>(pitch_) * pages_; }

  std::size_t Index(std::uint16_t x, std::uint16_t y) const
  {
    return static_cast<std::size_t>(y >> 3U) * pitch_ + x;
  }

  static std::uint8_t Mask(std::uint16_t y)
  {
    return static_cast<std::uint8_t>(1U << (y & 0x7U));
  }

  static std::int16_t Abs(std::int16_t value)
  {
    return value < 0 ? static_cast<std::int16_t>(-value) : value;
  }

  static std::uint16_t Min(std::uint16_t a, std::uint16_t b) { return a < b ? a : b; }

  static std::uint16_t Max(std::uint16_t a, std::uint16_t b) { return a > b ? a : b; }

  bool WritePixel(std::int16_t x, std::int16_t y, Color color)
  {
    if (!IsValid() || x < 0 || y < 0 || x >= static_cast<std::int16_t>(width_) ||
        y >= static_cast<std::int16_t>(height_))
    {
      return false;
    }

    const auto ux = static_cast<std::uint16_t>(x);
    const auto uy = static_cast<std::uint16_t>(y);
    const std::size_t index = Index(ux, uy);
    const std::uint8_t mask = Mask(uy);
    const std::uint8_t before = data_[index];

    switch (color)
    {
      case Color::CLEAR:
        data_[index] = static_cast<std::uint8_t>(data_[index] & ~mask);
        break;
      case Color::SET:
        data_[index] = static_cast<std::uint8_t>(data_[index] | mask);
        break;
      case Color::INVERT:
        data_[index] = static_cast<std::uint8_t>(data_[index] ^ mask);
        break;
    }

    return before != data_[index];
  }

  bool ClipRect(std::int16_t x, std::int16_t y, std::int16_t width, std::int16_t height,
                Rect& out) const
  {
    if (!IsValid() || width <= 0 || height <= 0)
    {
      return false;
    }

    std::int16_t x0 = x;
    std::int16_t y0 = y;
    std::int16_t x1 = static_cast<std::int16_t>(x + width);
    std::int16_t y1 = static_cast<std::int16_t>(y + height);

    if (x0 < 0)
    {
      x0 = 0;
    }
    if (y0 < 0)
    {
      y0 = 0;
    }
    if (x1 > static_cast<std::int16_t>(width_))
    {
      x1 = static_cast<std::int16_t>(width_);
    }
    if (y1 > static_cast<std::int16_t>(height_))
    {
      y1 = static_cast<std::int16_t>(height_);
    }
    if (x1 <= x0 || y1 <= y0)
    {
      return false;
    }

    out = {static_cast<std::uint16_t>(x0), static_cast<std::uint16_t>(y0),
           static_cast<std::uint16_t>(x1 - x0), static_cast<std::uint16_t>(y1 - y0)};
    return true;
  }

  void MarkDirty(std::uint16_t x, std::uint16_t y, std::uint16_t width,
                 std::uint16_t height)
  {
    if (!IsValid() || width == 0U || height == 0U || x >= width_ || y >= height_)
    {
      return;
    }

    const std::uint16_t x1 = Min(static_cast<std::uint16_t>(x + width), width_);
    const std::uint16_t y1 = Min(static_cast<std::uint16_t>(y + height), height_);

    if (!dirty_)
    {
      dirty_ = true;
      dirty_x0_ = x;
      dirty_y0_ = y;
      dirty_x1_ = x1;
      dirty_y1_ = y1;
      return;
    }

    dirty_x0_ = Min(dirty_x0_, x);
    dirty_y0_ = Min(dirty_y0_, y);
    dirty_x1_ = Max(dirty_x1_, x1);
    dirty_y1_ = Max(dirty_y1_, y1);
  }

  void MarkDirtyClipped(std::int16_t x, std::int16_t y, std::int16_t width,
                        std::int16_t height)
  {
    Rect clipped{};
    if (ClipRect(x, y, width, height, clipped))
    {
      MarkDirty(clipped.x, clipped.y, clipped.width, clipped.height);
    }
  }

  void PlotCirclePoints(std::int16_t x0, std::int16_t y0, std::int16_t x, std::int16_t y,
                        Color color)
  {
    SetPixel(static_cast<std::int16_t>(x0 + x), static_cast<std::int16_t>(y0 + y), color);
    SetPixel(static_cast<std::int16_t>(x0 + y), static_cast<std::int16_t>(y0 + x), color);
    SetPixel(static_cast<std::int16_t>(x0 - y), static_cast<std::int16_t>(y0 + x), color);
    SetPixel(static_cast<std::int16_t>(x0 - x), static_cast<std::int16_t>(y0 + y), color);
    SetPixel(static_cast<std::int16_t>(x0 - x), static_cast<std::int16_t>(y0 - y), color);
    SetPixel(static_cast<std::int16_t>(x0 - y), static_cast<std::int16_t>(y0 - x), color);
    SetPixel(static_cast<std::int16_t>(x0 + y), static_cast<std::int16_t>(y0 - x), color);
    SetPixel(static_cast<std::int16_t>(x0 + x), static_cast<std::int16_t>(y0 - y), color);
  }

  std::uint8_t* data_ = nullptr;
  std::size_t size_ = 0;
  std::uint16_t width_ = 0;
  std::uint16_t height_ = 0;
  std::uint16_t pitch_ = 0;
  std::uint16_t pages_ = 0;

  bool dirty_ = false;
  bool full_dirty_ = false;
  std::uint16_t dirty_x0_ = 0;
  std::uint16_t dirty_y0_ = 0;
  std::uint16_t dirty_x1_ = 0;
  std::uint16_t dirty_y1_ = 0;
};
