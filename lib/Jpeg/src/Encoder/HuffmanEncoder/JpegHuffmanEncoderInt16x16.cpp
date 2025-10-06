#include "JpegHuffmanEncoderTemplates.h"

namespace Jpeg
{
#if defined(PLATFORM_CPU_FEATURE_INT16x16)

static uint64_t reduce(Platform::Cpu::int16x8_t mask)
{
  Platform::Cpu::int64x2_t mask64x2{mask};
  uint64_t mask64 = (mask64x2 | mask64x2.shuffled<1, 1>()).get<0>();
  mask64 |= mask64 >> 32;
  return (mask64 | (mask64 >> 16)) & 0xffff;
}

static uint64_t reduce(Platform::Cpu::int16x16_t mask)
{
  return reduce(mask.lowPart() | mask.highPart());
}

#if 0
static Platform::Cpu::int64x4_t reduce4x4(Platform::Cpu::int16x16_t mask)
{
  using namespace Platform::Cpu;
  mask |= int16x16_t{uint64x4_t{mask} >> 32};
  return int64x4_t{mask | int16x16_t{uint64x4_t{mask} >> 16}} &int64x4_t::populate(0xffff);
}
#endif

typedef AcBitMask<Platform::Cpu::int16x16_t> AcBitMaskx16;

static AcBitMaskx16 acBitMaskx16_0 = makeAcBitMask<Platform::Cpu::int16x16_t, 0, 1, 5, 6, 14, 15, 27, 28, 2, 4, 7, 13, 16, 26, 29, 42>();
static AcBitMaskx16 acBitMaskx16_1 = makeAcBitMask<Platform::Cpu::int16x16_t, 3, 8, 12, 17, 25, 30, 41, 43, 9, 11, 18, 24, 31, 40, 44, 53>();
static AcBitMaskx16 acBitMaskx16_2 = makeAcBitMask<Platform::Cpu::int16x16_t, 10, 19, 23, 32, 39, 45, 52, 54, 20, 22, 33, 38, 46, 51, 55, 60>();
static AcBitMaskx16 acBitMaskx16_3 = makeAcBitMask<Platform::Cpu::int16x16_t, 21, 34, 37, 47, 50, 56, 59, 61, 35, 36, 48, 49, 57, 58, 62, 63>();

template<>
FORCE_INLINE uint64_t getAcMask<16>(const int16_t* block, int16_t* dst)
{
  using namespace Platform::Cpu;
  int16x16_t bits0_15 = int16x16_t::zero(), bits16_31 = int16x16_t::zero(), bits32_47 = int16x16_t::zero(), bits48_63 = int16x16_t::zero();
  int16x16_t w0 = int16x16_t::load<true>(block);
  int16x16_t w1 = int16x16_t::load<true>(block + 16);
  int16x16_t w2 = int16x16_t::load<true>(block + 32);
  int16x16_t w3 = int16x16_t::load<true>(block + 48);
  int16x16_t zero = int16x16_t::zero();

  updateMaskBits<int16x16_t, 0x7>(acBitMaskx16_0, (w0 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w0.onesComplement().store(dst);
  updateMaskBits<int16x16_t, 0xf>(acBitMaskx16_1, (w1 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w1.onesComplement().store(dst + 16);
  updateMaskBits<int16x16_t, 0xf>(acBitMaskx16_2, (w2 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w2.onesComplement().store(dst + 32);
  updateMaskBits<int16x16_t, 0xe>(acBitMaskx16_3, (w3 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w3.onesComplement().store(dst + 48);

#if 0
  int64x4_t bits = reduce4x4(bits0_15) | (reduce4x4(bits16_31) << 16) | (reduce4x4(bits32_47) << 32) | (reduce4x4(bits48_63) << 48);
  int64x2_t bits128 = int64x2_t{bits.lowPart() | bits.highPart()};
  return (bits128 | bits128.shuffled<1, 1>()).get<0>();
#else
  return reduce(bits0_15) | (reduce(bits16_31) << 16) | (reduce(bits32_47) << 32) | (reduce(bits48_63) << 48);
#endif
}

template <>
int64_t HuffmanEncoder::encodeBlocksImplementation<16>(const int16_t* block, const int* mcuComponents, int mcuBlockCount,
  const HuffmanEncoder* componentEncoders, const int* componentEncoderIndices, int* componentDc, int mcuCount,
  uint64_t* output, int64_t outputOffset, int64_t outputSizeInItems)
{
  return encodeBlocks<16>(block, mcuComponents, mcuBlockCount, componentEncoders, componentEncoderIndices, componentDc, mcuCount, output, outputOffset, outputSizeInItems);
}

#else // defined(PLATFORM_CPU_FEATURE_INT16x16)

template <>
int64_t HuffmanEncoder::encodeBlocksImplementation<16>(const int16_t*, const int*, int, const HuffmanEncoder*, const int*, int*, int, uint64_t*, int64_t, int64_t)
{
  assert(false);
  return 0;
}

#endif // defined(PLATFORM_CPU_FEATURE_INT16x16)
}
