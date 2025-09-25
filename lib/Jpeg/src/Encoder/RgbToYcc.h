#pragma once

#include <Jpeg/Rgb.h>

#include <Helper/Platform/Cpu/simd.h>

namespace Jpeg
{

struct Rgb8ToYccTable : public Rgb8ToYcc
{
  struct Weights
  {
    FixedPoint m_rWeights[256];
    FixedPoint m_gWeights[256];
    FixedPoint m_bWeights[256];

    inline int32_t rgbToComponentValue(int r, int g, int b) const;
    template<ImageMetaData::Format rgbFormat> inline int32_t rgbToComponentValue(uint32_t rgb) const;
  };

  Weights m_yTable;
  Weights m_cbTable;
  Weights m_crTable;

  template<ImageMetaData::Format format> inline int32_t rgbToY(uint32_t rgb) const;
  template<ImageMetaData::Format format> inline int32_t rgbToCb(uint32_t rgb) const;
  template<ImageMetaData::Format format> inline int32_t rgbToCr(uint32_t rgb) const;
};

template <typename T, int SimdLength, int cbcrAddFractionBits> struct RgbToYcc;

template <int SimdLength, int cbcrAddFractionBits>
struct RgbToYcc<int16_t, SimdLength, cbcrAddFractionBits> : public Rgb8ToYcc
{
  typedef Platform::Cpu::SIMD<int16_t, SimdLength> SimdHelper;
  typedef typename SimdHelper::Type SimdType;
  typedef typename Platform::Cpu::SIMD<int16_t, SimdLength>::ExtendedType ExtendedSimdType;

  static SimdType y(SimdType r, SimdType g, SimdType b)
  {
    constexpr int32_t ybgWeight = fixedPointFromFloat(0.25000);
    constexpr int32_t yrgWeight = ygWeight - ybgWeight;

    return (SimdHelper::mulAdd<yrWeight, yrgWeight>(r, g) + SimdHelper::mulAdd<ybWeight, ybgWeight>(b, g) + ExtendedSimdType::populate(yOffset)).descale<fixedPointFractionBits>();
  }

  static SimdType cb(SimdType r, SimdType g, SimdType b, int32_t bias = 0)
  {
    int32_t offset = cbcrOffset<cbcrAddFractionBits>();
    if (cbcrAddFractionBits)
      offset += bias;
    // TODO: unsigned extend()
    return (SimdHelper::mulAdd<cbrWeight, cbgWeight>(r, g) + (SimdHelper::extend(b) << (fixedPointFractionBits - 1)) + ExtendedSimdType::populate(offset)).descale<fixedPointFractionBits + cbcrAddFractionBits>();
  }

  static SimdType cr(SimdType r, SimdType g, SimdType b, int32_t bias = 0)
  {
    int32_t offset = cbcrOffset<cbcrAddFractionBits>();
    if (cbcrAddFractionBits)
      offset += bias;
    // TODO: unsigned extend()
    return (SimdHelper::mulAdd<crgWeight, crbWeight>(g, b) + (SimdHelper::extend(r) << (fixedPointFractionBits - 1)) + ExtendedSimdType::populate(offset)).descale<fixedPointFractionBits + cbcrAddFractionBits>();
  }
};

template <int SimdLength, int cbcrAddFractionBits>
struct RgbToYcc<int32_t, SimdLength, cbcrAddFractionBits> : public Rgb8ToYcc
{
  typedef Platform::Cpu::SIMD<int32_t, SimdLength> SimdHelper;
  typedef typename SimdHelper::Type SimdType;

  static SimdType y(SimdType r, SimdType g, SimdType b)
  {
    using namespace Platform::Cpu::int32;
    return (r * yrWeight + g * ygWeight + b * ybWeight + SimdHelper::populate(yOffset)) >> fixedPointFractionBits;
  }

  static SimdType cb(SimdType r, SimdType g, SimdType b, int32_t bias = 0)
  {
    using namespace Platform::Cpu::int32;
    return ((b << (fixedPointFractionBits - 1)) + r * cbrWeight + g * cbgWeight + SimdHelper::populate(cbcrOffset<cbcrAddFractionBits>() + bias)) >> (fixedPointFractionBits + cbcrAddFractionBits);
  }

  static SimdType cr(SimdType r, SimdType g, SimdType b, int32_t bias = 0)
  {
    using namespace Platform::Cpu::int32;
    return ((r << (fixedPointFractionBits - 1)) + g * crgWeight + b * crbWeight + SimdHelper::populate(cbcrOffset<cbcrAddFractionBits>() + bias)) >> (fixedPointFractionBits + cbcrAddFractionBits);
  }
};

template <typename T, int cbcrAddFractionBits>
struct RgbToYccNoSimd : public Rgb8ToYcc
{
  static T y(T r, T g, T b)
  {
    return (r * yrWeight + g * ygWeight + b * ybWeight + yOffset) >> fixedPointFractionBits;
  }

  static T cb(T r, T g, T b, int32_t bias = 0)
  {
    return ((b << (fixedPointFractionBits - 1)) + r * cbrWeight + g * cbgWeight + cbcrOffset<cbcrAddFractionBits>() + bias) >> (fixedPointFractionBits + cbcrAddFractionBits);
  }

  static T cr(T r, T g, T b, int32_t bias = 0)
  {
    return ((r << (fixedPointFractionBits - 1)) + g * crgWeight + b * crbWeight + cbcrOffset<cbcrAddFractionBits>() + bias) >> (fixedPointFractionBits + cbcrAddFractionBits);
  }
};

template <int cbcrAddFractionBits>
struct RgbToYcc<int16_t, 1, cbcrAddFractionBits> : public RgbToYccNoSimd<int16_t, cbcrAddFractionBits>
{
};

template <int cbcrAddFractionBits>
struct RgbToYcc<int32_t, 1, cbcrAddFractionBits> : public RgbToYccNoSimd<int32_t, cbcrAddFractionBits>
{
};

// implementation

inline int32_t Jpeg::Rgb8ToYccTable::Weights::rgbToComponentValue(int r, int g, int b) const
{
  return (m_rWeights[r] + m_gWeights[g] + m_bWeights[b]) >> fixedPointFractionBits;
}

template<ImageMetaData::Format rgbFormat>
inline int32_t Rgb8ToYccTable::Weights::rgbToComponentValue(uint32_t rgb) const
{
  return rgbToComponentValue(Rgb<rgbFormat>::red(rgb), Rgb<rgbFormat>::green(rgb), Rgb<rgbFormat>::blue(rgb));
}

template<ImageMetaData::Format format>
inline int32_t Rgb8ToYccTable::rgbToY(uint32_t rgb) const
{
  return m_yTable.rgbToComponentValue<format>(rgb);
}

template<ImageMetaData::Format format>
inline int32_t Rgb8ToYccTable::rgbToCb(uint32_t rgb) const
{
  return m_cbTable.rgbToComponentValue<format>(rgb);
}

template<ImageMetaData::Format format>
inline int32_t Rgb8ToYccTable::rgbToCr(uint32_t rgb) const
{
  return m_crTable.rgbToComponentValue<format>(rgb);
}

}
