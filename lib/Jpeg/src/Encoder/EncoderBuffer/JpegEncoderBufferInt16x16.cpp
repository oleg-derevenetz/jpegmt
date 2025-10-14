#include "JpegEncoderBufferTemplates.h"

namespace Jpeg
{

#if defined(PLATFORM_CPU_FEATURE_INT32x8) && defined(PLATFORM_CPU_FEATURE_INT16x16) && defined(PLATFORM_CPU_FEATURE_INT8x32)

// memory

template <> void* EncoderBuffer::allocSimdBuffer<int16_t, 16>(int count)
{
  return Platform::Cpu::SIMD<int16_t, 16>::template allocMemory<int16_t[Dct::BlockSize2]>(count);
}

template <> void EncoderBuffer::releaseSimdBuffer<int16_t, 16>(void* buffer)
{
  Platform::Cpu::SIMD<int16_t, 16>::freeMemory(buffer);
}

// Grayscale

#ifdef TRANSPOSED_SIMD_BUFFER
template<>
FORCE_INLINE void grayScanlineSimdToYComponent<Dct::BlockSize*2, int16_t>(const uint8_t* pixels, int scanlineLength, const EncoderBuffer::McuIndex* mcu, int count, int16_t (*dst)[Dct::BlockSize*2])
{
  using namespace Platform::Cpu;
  constexpr int SimdLength = Dct::BlockSize*2;
  int simdCount = count / SimdLength;
  int x0 = mcu[0].m_x * Dct::BlockSize;
  int y0 = mcu[0].m_y * Dct::BlockSize;

//  int8x16_t zeroLevel = int8x16_t::populate((int8_t)sampleZeroLevel);
  int8x32_t zeroLevel = int8x32_t::populate((int8_t)sampleZeroLevel);
  for (int j = 0; j < Dct::BlockSize; j++)
  {
    const int8_t* pixelLine = (int8_t*)pixels + (y0 + j) * scanlineLength + x0;
    int16x16_t* dstLine = (int16x16_t*)(dst + j * Dct::BlockSize);

    for (int m = 0; m < simdCount; m++)
    {
#if 1
      int8x32_t c0 = int8x32_t::load<false>(pixelLine + 0 * 2 * SimdLength) - zeroLevel; // 0 1 2 3
      int8x32_t c1 = int8x32_t::load<false>(pixelLine + 1 * 2 * SimdLength) - zeroLevel; // 4 5 6 7
      int8x32_t c2 = int8x32_t::load<false>(pixelLine + 2 * 2 * SimdLength) - zeroLevel; // 8 9 A B
      int8x32_t c3 = int8x32_t::load<false>(pixelLine + 3 * 2 * SimdLength) - zeroLevel; // C D E F
      int8x32_t b0 = int8x32_t{int64x4_t::shuffle<0, 4, 2, 6>(c0, c2)}; // 0 8 2 A
      int8x32_t b1 = int8x32_t{int64x4_t::shuffle<1, 5, 3, 7>(c0, c2)}; // 1 9 3 B
      int8x32_t b2 = int8x32_t{int64x4_t::shuffle<0, 4, 2, 6>(c1, c3)}; // 4 C 6 E
      int8x32_t b3 = int8x32_t{int64x4_t::shuffle<1, 5, 3, 7>(c1, c3)}; // 5 D 7 F

      SIMD<int16_t, SimdLength>::transpose2x8x8(dstLine,
        int16x16_t::fromPackedInt8(b0.lowPart()), int16x16_t::fromPackedInt8(b1.lowPart()),    // 0 8, 1 9
        int16x16_t::fromPackedInt8(b0.highPart()), int16x16_t::fromPackedInt8(b1.highPart()),  // 2 A, 3 B
        int16x16_t::fromPackedInt8(b2.lowPart()), int16x16_t::fromPackedInt8(b3.lowPart()),    // 4 C, 5 D 
        int16x16_t::fromPackedInt8(b2.highPart()), int16x16_t::fromPackedInt8(b3.highPart())); // 6 E, 7 F
#else
      int8x16_t c0 = int8x16_t::load<false>(pixelLine + 0 * SimdLength) - zeroLevel;
      int8x16_t c1 = int8x16_t::load<false>(pixelLine + 1 * SimdLength) - zeroLevel;
      int8x16_t c2 = int8x16_t::load<false>(pixelLine + 2 * SimdLength) - zeroLevel;
      int8x16_t c3 = int8x16_t::load<false>(pixelLine + 3 * SimdLength) - zeroLevel;
      int8x16_t c4 = int8x16_t::load<false>(pixelLine + 4 * SimdLength) - zeroLevel;
      int8x16_t c5 = int8x16_t::load<false>(pixelLine + 5 * SimdLength) - zeroLevel;
      int8x16_t c6 = int8x16_t::load<false>(pixelLine + 6 * SimdLength) - zeroLevel;
      int8x16_t c7 = int8x16_t::load<false>(pixelLine + 7 * SimdLength) - zeroLevel;

      // NOTE: on x86 int16x16_t::transpose2x8x8 using AVX require x2 lesser operations than int8x16_t using SSE
      // on other plastforms int8x16_t::transpose2x8x8 may be faster then int16x16_t::transpose2x8x8
      SIMD<int16_t, SimdLength>::transpose2x8x8(dstLine,
        int16x16_t::fromPackedInt8(int8x16_t{int64x2_t::shuffle<0, 2>(c0, c4)}), int16x16_t::fromPackedInt8(int8x16_t{int64x2_t::shuffle<1, 3>(c0, c4)}),
        int16x16_t::fromPackedInt8(int8x16_t{int64x2_t::shuffle<0, 2>(c1, c5)}), int16x16_t::fromPackedInt8(int8x16_t{int64x2_t::shuffle<1, 3>(c1, c5)}),
        int16x16_t::fromPackedInt8(int8x16_t{int64x2_t::shuffle<0, 2>(c2, c6)}), int16x16_t::fromPackedInt8(int8x16_t{int64x2_t::shuffle<1, 3>(c2, c6)}),
        int16x16_t::fromPackedInt8(int8x16_t{int64x2_t::shuffle<0, 2>(c3, c7)}), int16x16_t::fromPackedInt8(int8x16_t{int64x2_t::shuffle<1, 3>(c3, c7)}));
#endif
      pixelLine += SimdLength * Dct::BlockSize;
      dstLine += Dct::BlockSize2;
    }
  }
}
#endif // TRANSPOSED_SIMD_BUFFER

template <>
void grayToYComponentImplementation<int16_t, 16>(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int16_t* dst)
{
  grayToYComponent<16>(imageMetaData, pixels, bufferMetaData, mcu, count, (int16_t(*)[16])dst);
}

// RGB

template <bool aligned, int block>
void loadRgbSimdBlocks(Platform::Cpu::int32x8_t(*matrix)[Dct::BlockSize][2], const int32_t(*blocks)[2][Dct::BlockSize])
{
  typedef Platform::Cpu::SIMD<int32_t, 8> SimdHelper;
  SimdHelper::transpose<true, 2, 2>(matrix[block][0], blocks[0][block]);
  SimdHelper::transpose<true, 2, 2>(matrix[block][0] + 1, blocks[Dct::BlockSize][block]);
}

template<>
void loadRgbSimdLine<Dct::BlockSize*2, Platform::Cpu::int32x8_t>(Platform::Cpu::int32x8_t* dst, const int32_t (*src)[2][Dct::BlockSize])
{
  Platform::Cpu::int32x8_t (*matrix)[Dct::BlockSize][2] = (Platform::Cpu::int32x8_t (*)[Dct::BlockSize][2])dst;
  if (Platform::Cpu::SIMD<int32_t, 8>::isPointerAligned(src))
  {
    loadRgbSimdBlocks<true, 0>(matrix, src);
    loadRgbSimdBlocks<true, 1>(matrix, src);
  }
  else
  {
    loadRgbSimdBlocks<false, 0>(matrix, src);
    loadRgbSimdBlocks<false, 1>(matrix, src);
  }
}

template<>
void rgbaToYcbr411Implementation<int16_t, 16>(const ImageMetaData& imageMetaData, const uint8_t* rgb, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int16_t* dst, int options)
{
  rgbaToYcbr411Helper<16, int16_t>::perform(imageMetaData, rgb, bufferMetaData, mcu, count, (int16_t (*)[Dct::BlockSize2][16])dst, options);
}

// export

template<>
void exportBlock<Dct::BlockSize*2, int16_t>(int16_t (*dst)[Dct::BlockSize2], const int16_t (*values)[Dct::BlockSize*2], int mcuBlocks, int count)
{
  constexpr int SimdLength = Dct::BlockSize*2;
  typedef Platform::Cpu::SIMD<int16_t, SimdLength> SimdHelper;
  typedef typename SimdHelper::Type SimdType;
  typename SimdHelper::NativeType matrix[SimdLength];

  if (count >= SimdLength)
  {
    for (int j = 0; j < Dct::BlockSize; j++)
    {
      int k0 = j * Dct::BlockSize;
      SimdHelper::template transpose2x8x8<true>((SimdType*)matrix, values[k0]);

      for (int n = 0; n < Dct::BlockSize; n++)
      {
        SimdHelper::storeLow(dst[n * mcuBlocks] + k0, matrix[n]);
        SimdHelper::storeHigh(dst[(n + Dct::BlockSize) * mcuBlocks] + k0, matrix[n]);
      }
    }
  }
  else
  {
    for (int j = 0; j < Dct::BlockSize; j++)
    {
      int k0 = j * Dct::BlockSize;
      SimdHelper::template transpose2x8x8<true>((SimdType*)matrix, values[k0]);
      int lowCount = count < Dct::BlockSize ? count : Dct::BlockSize;

      for (int n = 0; n < lowCount; n++)
        SimdHelper::storeLow(dst[n * mcuBlocks] + k0, matrix[n]);
      for (int n = lowCount; n < count; n++)
        SimdHelper::storeHigh(dst[n * mcuBlocks] + k0, matrix[n - Dct::BlockSize]);
    }
  }
}

template<>
void exportBlocksImplementation<int16_t, 16>(const EncoderBuffer::MetaData& metaData, int16_t (*dst)[Dct::BlockSize2], const int16_t* buffers, int count)
{
  exportBlocks<16>(metaData, dst, (int16_t (*)[Dct::BlockSize2 * 16])buffers, count);
}

#else // defined(PLATFORM_CPU_FEATURE_INT32x8) && defined(PLATFORM_CPU_FEATURE_INT16x16) && defined(PLATFORM_CPU_FEATURE_INT8x32)

template <> void* EncoderBuffer::allocSimdBuffer<int16_t, 16>(int count)
{
  return malloc(sizeof(int16_t) * Dct::BlockSize2 * count);
}

template <> void EncoderBuffer::releaseSimdBuffer<int16_t, 16>(void* buffer)
{
  free(buffer);
}

template <>
void grayToYComponentImplementation<int16_t, 16>(const ImageMetaData&, const uint8_t*, const EncoderBuffer::MetaData&, const EncoderBuffer::McuIndex*, int, int16_t*)
{
  assert(false);
}

template<>
void rgbaToYcbr411Implementation<int16_t, 16>(const ImageMetaData&, const uint8_t*, const EncoderBuffer::MetaData&, const EncoderBuffer::McuIndex*, int, int16_t*, int)
{
  assert(false);
}

template<>
void exportBlocksImplementation<int16_t, 16>(const EncoderBuffer::MetaData&, int16_t (*)[Dct::BlockSize2], const int16_t*, int)
{
  assert(false);
}

#endif // defined(PLATFORM_CPU_FEATURE_INT32x8) && defined(PLATFORM_CPU_FEATURE_INT16x16) && defined(PLATFORM_CPU_FEATURE_INT8x32)

template<>
int EncoderBuffer::MetaData::detectSimdLength<int16_t>(uint64_t features, int lengthLimit)
{
  return Platform::Cpu::SimdDetector<int16_t>::maxSimdLength(features, lengthLimit);
}

}
