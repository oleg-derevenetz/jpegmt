#pragma once

#include <cstdint>

#include <Helper/Platform/Cpu/cpu.h>

#include "JpegImageMetaData.h"

namespace Jpeg
{

template<ImageMetaData::Format format>
struct Rgb32
{
  typedef uint32_t PixelType;
  typedef uint8_t ComponentType;

  template<int index> static inline ComponentType getComponent(PixelType rgb);

  static inline ComponentType red(PixelType rgb);
  static inline ComponentType green(PixelType rgb);
  static inline ComponentType blue(PixelType rgb);
  static inline PixelType rgb(ComponentType r, ComponentType g, ComponentType b);
};

// TODO: detect pixel and component type for formats different from rgb32
template<ImageMetaData::Format format> struct Rgb;

template<> struct Rgb<ImageMetaData::Rgba32> : public Rgb32<ImageMetaData::Rgba32>
{
};

template<> struct Rgb<ImageMetaData::Bgra32> : public Rgb32<ImageMetaData::Bgra32>
{
};

struct FixedPoint32
{
  typedef int32_t FixedPoint;
  constexpr static int fixedPointFractionBits = 16;      /* speediest right-shift on some machines */
  constexpr static FixedPoint fixedPointOneHalf = (FixedPoint)1 << (fixedPointFractionBits - 1);

  constexpr static FixedPoint fixedPointFromFloat(double x)
  {
    return ((FixedPoint)((x) * (1L << fixedPointFractionBits) + 0.5));
  }
};

struct Rgb8ToYcc : public FixedPoint32
{
  constexpr static FixedPoint yrWeight = fixedPointFromFloat(0.29900);
  constexpr static FixedPoint ybWeight = fixedPointFromFloat(0.11400);
  constexpr static FixedPoint ygWeight = fixedPointFromFloat(0.58700);
  constexpr static FixedPoint yOffset = -((FixedPoint)128 << fixedPointFractionBits) + fixedPointOneHalf - 1;

  constexpr static FixedPoint cbrWeight = -fixedPointFromFloat(0.16874);
  constexpr static FixedPoint cbgWeight = -fixedPointFromFloat(0.33126);

  constexpr static FixedPoint crbWeight = -fixedPointFromFloat(0.08131);
  constexpr static FixedPoint crgWeight = -fixedPointFromFloat(0.41869);

  template<int cbcrAddFractionBits>
  constexpr static FixedPoint cbcrOffset()
  {
    return ((FixedPoint)1 << (fixedPointFractionBits + cbcrAddFractionBits - 1)) - 1;
  }

  constexpr static FixedPoint cbcr8Offset = (FixedPoint)128 << fixedPointFractionBits;

  template<ImageMetaData::Format format> static inline int32_t rgbToY(uint32_t rgb);
  template<ImageMetaData::Format format> static inline int32_t rgbToCb(uint32_t rgb);
  template<ImageMetaData::Format format> static inline int32_t rgbToCr(uint32_t rgb);
};

// implementation

template<ImageMetaData::Format format> template<int index>
inline typename Rgb32<format>::ComponentType Rgb32<format>::getComponent(PixelType rgb)
{
  static_assert(index >= 0 && index < 4, "component index out of range");
  return Platform::Cpu::byteOrder == Platform::Cpu::LittleEndian ? ((rgb >> (index * 8)) & 0xff) : ((rgb >> ((3 - index) * 8)) & 0xff);
}

template<ImageMetaData::Format format>
inline typename Rgb32<format>::ComponentType Rgb32<format>::red(PixelType rgb)
{
  switch (format)
  {
  case ImageMetaData::Rgba32:
    return getComponent<0>(rgb);
  case ImageMetaData::Bgra32:
    return getComponent<2>(rgb);
  }

  return 0;
}

template<ImageMetaData::Format format>
inline typename Rgb32<format>::ComponentType Jpeg::Rgb32<format>::green(PixelType rgb)
{
  switch (format)
  {
  case ImageMetaData::Rgba32:
  case ImageMetaData::Bgra32:
    return getComponent<1>(rgb);
  }

  return 0;
}

template<ImageMetaData::Format format>
inline typename Rgb32<format>::ComponentType Jpeg::Rgb32<format>::blue(PixelType rgb)
{
  switch (format)
  {
  case ImageMetaData::Rgba32:
    return getComponent<2>(rgb);
  case ImageMetaData::Bgra32:
    return getComponent<0>(rgb);
  }

  return 0;
}

template<ImageMetaData::Format format>
inline typename Rgb32<format>::PixelType Rgb32<format>::rgb(ComponentType r, ComponentType g, ComponentType b)
{
  switch (format)
  {
  case ImageMetaData::Rgba32:
    if (Platform::Cpu::byteOrder == Platform::Cpu::LittleEndian)
      return 0xff000000 | ((PixelType)b << 16) | ((PixelType)g << 8) | r;
    else
      return ((PixelType)r << 24) | ((PixelType)g << 16) | ((PixelType)b << 8) | 0xff;
  case ImageMetaData::Bgra32:
    if (Platform::Cpu::byteOrder == Platform::Cpu::LittleEndian)
      return 0xff000000 | ((PixelType)r << 16) | ((PixelType)g << 8) | b;
    else
      return ((PixelType)b << 24) | ((PixelType)g << 16) | ((PixelType)r << 8) | 0xff;
  }

  return 0;
}

template<ImageMetaData::Format format>
inline int32_t Rgb8ToYcc::rgbToY(uint32_t rgb)
{
  return (yrWeight * Rgb<format>::red(rgb) + ygWeight * Rgb<format>::green(rgb) + ybWeight * Rgb<format>::blue(rgb)) >> fixedPointFractionBits;
}

template<ImageMetaData::Format format>
inline int32_t Rgb8ToYcc::rgbToCb(uint32_t rgb)
{
  return (cbrWeight * Rgb<format>::red(rgb) + cbgWeight * Rgb<format>::green(rgb) +
    fixedPointFromFloat(0.50000) * Rgb<format>::blue(rgb) + cbcr8Offset + fixedPointOneHalf - 1) >> fixedPointFractionBits;
}

template<ImageMetaData::Format format>
inline int32_t Rgb8ToYcc::rgbToCr(uint32_t rgb)
{
  return (fixedPointFromFloat(0.50000) * Rgb<format>::red(rgb) + crgWeight * Rgb<format>::green(rgb) +
    crbWeight * Rgb<format>::blue(rgb) + cbcr8Offset + fixedPointOneHalf - 1) >> fixedPointFractionBits;
}

}
