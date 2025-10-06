#include "JpegHuffmanEncoderTemplates.h"

namespace Jpeg
{

#if defined(PLATFORM_CPU_FEATURE_INT16x8)

static uint64_t reduce(Platform::Cpu::int16x8_t mask)
{
  Platform::Cpu::int64x2_t mask64x2{mask};
  uint64_t mask64 = (mask64x2 | mask64x2.shuffled<1, 1>()).get<0>();
  mask64 |= mask64 >> 32;
  return (mask64 | (mask64 >> 16)) & 0xffff;
}

typedef AcBitMask<Platform::Cpu::int16x8_t> AcBitMaskx8;

static AcBitMaskx8 acBitMaskx8_0 = makeAcBitMask<Platform::Cpu::int16x8_t, 0, 1, 5, 6, 14, 15, 27, 28>();
static AcBitMaskx8 acBitMaskx8_1 = makeAcBitMask<Platform::Cpu::int16x8_t, 2, 4, 7, 13, 16, 26, 29, 42>();
static AcBitMaskx8 acBitMaskx8_2 = makeAcBitMask<Platform::Cpu::int16x8_t, 3, 8, 12, 17, 25, 30, 41, 43>();
static AcBitMaskx8 acBitMaskx8_3 = makeAcBitMask<Platform::Cpu::int16x8_t, 9, 11, 18, 24, 31, 40, 44, 53>();
static AcBitMaskx8 acBitMaskx8_4 = makeAcBitMask<Platform::Cpu::int16x8_t, 10, 19, 23, 32, 39, 45, 52, 54>();
static AcBitMaskx8 acBitMaskx8_5 = makeAcBitMask<Platform::Cpu::int16x8_t, 20, 22, 33, 38, 46, 51, 55, 60>();
static AcBitMaskx8 acBitMaskx8_6 = makeAcBitMask<Platform::Cpu::int16x8_t, 21, 34, 37, 47, 50, 56, 59, 61>();
static AcBitMaskx8 acBitMaskx8_7 = makeAcBitMask<Platform::Cpu::int16x8_t, 35, 36, 48, 49, 57, 58, 62, 63>();

template<>
FORCE_INLINE uint64_t getAcMask<8>(const int16_t* block, int16_t* dst)
{
  using namespace Platform::Cpu;
  int16x8_t bits0_15 = int16x8_t::zero(), bits16_31 = int16x8_t::zero(), bits32_47 = int16x8_t::zero(), bits48_63 = int16x8_t::zero();
  int16x8_t w0 = int16x8_t::load<true>(block);
  int16x8_t w1 = int16x8_t::load<true>(block + 8);
  int16x8_t w2 = int16x8_t::load<true>(block + 16);
  int16x8_t w3 = int16x8_t::load<true>(block + 24);
  int16x8_t w4 = int16x8_t::load<true>(block + 32);
  int16x8_t w5 = int16x8_t::load<true>(block + 40);
  int16x8_t w6 = int16x8_t::load<true>(block + 48);
  int16x8_t w7 = int16x8_t::load<true>(block + 56);
  int16x8_t zero = int16x8_t::zero();

  updateMaskBits<int16x8_t, 0x3>(acBitMaskx8_0, (w0 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w0.onesComplement().store(dst);
  updateMaskBits<int16x8_t, 0x7>(acBitMaskx8_1, (w1 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w1.onesComplement().store(dst + 8);
  updateMaskBits<int16x8_t, 0x7>(acBitMaskx8_2, (w2 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w2.onesComplement().store(dst + 16);
  updateMaskBits<int16x8_t, 0xf>(acBitMaskx8_3, (w3 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w3.onesComplement().store(dst + 24);
  updateMaskBits<int16x8_t, 0xf>(acBitMaskx8_4, (w4 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w4.onesComplement().store(dst + 32);
  updateMaskBits<int16x8_t, 0xe>(acBitMaskx8_5, (w5 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w5.onesComplement().store(dst + 40);
  updateMaskBits<int16x8_t, 0xe>(acBitMaskx8_6, (w6 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w6.onesComplement().store(dst + 48);
  updateMaskBits<int16x8_t, 0xc>(acBitMaskx8_7, (w7 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w7.onesComplement().store(dst + 56);

  return reduce(bits0_15) | (reduce(bits16_31) << 16) | (reduce(bits32_47) << 32) | (reduce(bits48_63) << 48);
}

template <>
int64_t HuffmanEncoder::encodeBlocksImplementation<8>(const int16_t* block, const int* mcuComponents, int mcuBlockCount,
  const HuffmanEncoder* componentEncoders, const int* componentEncoderIndices, int* componentDc, int mcuCount,
  uint64_t* output, int64_t outputOffset, int64_t outputSizeInItems)
{
  return encodeBlocks<8>(block, mcuComponents, mcuBlockCount, componentEncoders, componentEncoderIndices, componentDc, mcuCount, output, outputOffset, outputSizeInItems);
}

#else // defined(PLATFORM_CPU_FEATURE_INT16x16)

template <>
int64_t HuffmanEncoder::encodeBlocksImplementation<8>(const int16_t*, const int*, int, const HuffmanEncoder*, const int*, int*, int, uint64_t*, int64_t, int64_t)
{
  assert(false);
  return 0;
}

#endif // defined(PLATFORM_CPU_FEATURE_INT16x16)

}
