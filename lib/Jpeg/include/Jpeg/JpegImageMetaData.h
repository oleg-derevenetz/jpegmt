#pragma once

namespace Jpeg
{

struct Size
{
  int m_width = -1;
  int m_height = -1;

//  inline bool isValid() const;
  bool operator==(const Size& other) const
  {
    return m_width == other.m_width && m_height == other.m_height;
  }
};

struct ImageMetaData
{
  enum Format
  {
    Invalid,
    Grayscale8,
    Rgba32,
    Bgra32,
  };

  Size m_size;
  Format m_format = Invalid;
  int m_scanlineBytes = 0;
  int m_dotsPerMeterHorizontal = 96;
  int m_dotsPerMeterVertical = 96;

  bool isValid() const
  {
    return m_format != Invalid && m_size.m_width > 0 && m_size.m_width > 0;
  }
};

}
