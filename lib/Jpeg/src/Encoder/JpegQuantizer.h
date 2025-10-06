#pragma once

#include <cassert>
#include <cstdint>

#include <Helper/Platform/Cpu/simd.h>

#include <Jpeg/JpegDCT.h>

namespace Jpeg
{

class EncoderBuffer;
struct QuantizationTable;

class alignas(32) Quantizer
{
public:
  Quantizer(const QuantizationTable& quantizationTable);

//  void perform(const EncoderBuffer& buffer, int c, int16_t (*dst)[Dct::BlockSize2], int count) const;

  bool isUnit() const { return m_isUnit; }
  bool haveNonPositiveShift() const { return m_haveNonPositiveShift;  }

  template<int SimdSize> using SimdInt32 = typename Platform::Cpu::SIMD<int32_t, SimdSize>::Type;
  template<int SimdSize> using SimdUint32 = typename Platform::Cpu::SIMD<uint32_t, SimdSize>::Type;
  template<int SimdSize> using SimdInt16 = typename Platform::Cpu::SIMD<int16_t, SimdSize>::Type;
  template<int SimdSize> using SimdUint16 = typename Platform::Cpu::SIMD<uint16_t, SimdSize>::Type;

  template<int SimdSize>
  inline SimdInt32<SimdSize> quantize(SimdInt32<SimdSize> value, int k) const
  {
    using namespace Platform::Cpu;
    using namespace Platform::Cpu::int32;
    using namespace Platform::Cpu::uint32;
    typedef SIMD<int32_t, SimdSize> SimdHelper;
    typedef SIMD<uint32_t, SimdSize> USimdHelper;

    const Divisor& divisor = m_divisors[k];
    SimdUint32<SimdSize> hi;
//    USimdHelper::mulExtended(SimdUint32<SimdSize>{SimdHelper::abs(value)} + USimdHelper::populate(divisor.m_correction), USimdHelper::populate(divisor.m_reciprocal), hi);
    USimdHelper::mulExtended(SimdUint32<SimdSize>{SimdHelper::abs(value)} + USimdHelper::load(m_uint32x8table[k][1]), USimdHelper::load(m_uint32x8table[k][0]), hi);
    return SimdHelper::mulSign(SimdInt32<SimdSize>{hi >> divisor.m_shift}, value);
  }

  template<int SimdSize>
  inline SimdInt16<SimdSize> quantize(SimdInt16<SimdSize> value, int k) const
  {
    using namespace Platform::Cpu;
    typedef SIMD<int16_t, SimdSize> SimdHelper;
    typedef typename SimdHelper::Type SimdType;

    static_assert(SimdSize <= 16, "not implemented"); // TODO: extend m_int16xXX table if required
    assert(!m_haveNonPositiveShift);
    SimdType reciprocal = SimdType::load(m_int16x16table[k][0]);
    SimdType correction = SimdType::load(m_int16x16table[k][1]);
    SimdType scale = SimdType::load(m_int16x16table[k][2]);
    return SimdHelper::mulSign(SimdHelper::mulFixedPoint(SimdHelper::mulFixedPoint(SimdHelper::abs(value) + correction, reciprocal), scale), value);
  }

  template<int SimdSize>
  inline SimdInt16<SimdSize> quantize32(SimdInt16<SimdSize> value, int k) const
  {
    typedef Platform::Cpu::SIMD<int16_t, SimdSize> SimdHelper;

    assert(m_haveNonPositiveShift);
    typename SimdHelper::ExtendedType value32 = SimdHelper::extend(value);
    value32.lo = quantize<SimdSize / 2>(SimdInt32<SimdSize / 2>{value32.lo}, k);
    value32.hi = quantize<SimdSize / 2>(SimdInt32<SimdSize / 2>{value32.hi}, k);
    return value32.template descale<0>();
  }

  template<int SimdSize>
  inline SimdInt16<SimdSize> descale(SimdInt16<SimdSize> value) const
  {
    typedef Platform::Cpu::SIMD<int16_t, SimdSize> SimdHelper;

    return SimdHelper::mulSign((SimdHelper::abs(value) + SimdHelper::populate(4)) >> 3, value);
  }

#if !defined(TRANSPOSED_SIMD_BUFFER) && defined(PLATFORM_CPU_FEATURE_INT16x16)
  typedef typename Platform::Cpu::SIMD<int16_t, 16>::Type Int16x16;

  void quantizeBlock(Int16x16& w0, Int16x16& w1, Int16x16& w2, Int16x16& w3) const
  {
    typedef Platform::Cpu::SIMD<int16_t, 16> SimdHelper;
    
    assert(!m_haveNonPositiveShift);
    w0 = SimdHelper::mulSign(SimdHelper::mulFixedPoint(SimdHelper::mulFixedPoint(SimdHelper::abs(w0) + SimdHelper::load(m_correction16), SimdHelper::load(m_reciprocal16)), SimdHelper::load(m_scale16)), w0);
    w1 = SimdHelper::mulSign(SimdHelper::mulFixedPoint(SimdHelper::mulFixedPoint(SimdHelper::abs(w1) + SimdHelper::load(m_correction16 + 16), SimdHelper::load(m_reciprocal16 + 16)), SimdHelper::load(m_scale16 + 16)), w1);
    w2 = SimdHelper::mulSign(SimdHelper::mulFixedPoint(SimdHelper::mulFixedPoint(SimdHelper::abs(w2) + SimdHelper::load(m_correction16 + 32), SimdHelper::load(m_reciprocal16 + 32)), SimdHelper::load(m_scale16 + 32)), w2);
    w3 = SimdHelper::mulSign(SimdHelper::mulFixedPoint(SimdHelper::mulFixedPoint(SimdHelper::abs(w3) + SimdHelper::load(m_correction16 + 48), SimdHelper::load(m_reciprocal16 + 48)), SimdHelper::load(m_scale16 + 48)), w3);
  }
#endif

private:
  struct Divisor
  {
    uint32_t m_reciprocal = 1;
    int16_t m_correction = 0;
    int16_t m_shift = -(int16_t)(sizeof(int32_t) * 8);
  };

  Divisor m_divisors[Dct::BlockSize2];

  alignas(32) int16_t m_reciprocal16[Dct::BlockSize2];
  alignas(32) int16_t m_correction16[Dct::BlockSize2];
  alignas(32) int16_t m_scale16[Dct::BlockSize2];

  alignas(32) int16_t m_int16x16table[Dct::BlockSize2][3][16];
  alignas(32) uint32_t m_uint32x8table[Dct::BlockSize2][2][8];

  bool m_isUnit = true;
  bool m_haveNonPositiveShift = false;
//  int m_storingOrder[Dct::BlockSize2];
};

template<>
inline int16_t Quantizer::quantize32<1>(int16_t value, int k) const
{
  const Divisor& divisor = m_divisors[k];
  int16_t quantized = (int16_t)((uint64_t)((value < 0 ? -value : value) + divisor.m_correction) * divisor.m_reciprocal) >> (divisor.m_shift + 32);
  return value < 0 ? -quantized : quantized;
}

template<>
inline int16_t Quantizer::quantize<1>(int16_t value, int k) const
{
  return quantize32<1>(value, k);
}

template<>
inline int32_t Quantizer::quantize<1>(int32_t value, int k) const
{
  return quantize32<1>(value, k);
}

template<>
inline int16_t Quantizer::descale<1>(int16_t value) const
{
  int16_t quantized = (int16_t)((int32_t)((value < 0 ? -value : value) + 4) >> 3);
  return value < 0 ? -quantized : quantized;
}

}
