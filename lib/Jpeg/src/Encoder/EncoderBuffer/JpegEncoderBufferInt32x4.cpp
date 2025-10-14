#include "JpegEncoderBufferTemplates.h"

namespace Jpeg
{

#if defined(PLATFORM_CPU_FEATURE_INT32x4)

// memory

template <> void* EncoderBuffer::allocSimdBuffer<int32_t, 4>(int count)
{
  return Platform::Cpu::SIMD<int32_t, 4>::template allocMemory<int32_t[Dct::BlockSize2]>(count);
}

template <> void EncoderBuffer::releaseSimdBuffer<int32_t, 4>(void* buffer)
{
  Platform::Cpu::SIMD<int32_t, 4>::freeMemory(buffer);
}

// Grayscale

template<>
void grayScanlineSimdToYComponent<4, int32_t>(const uint8_t* pixels, int scanlineLength, const EncoderBuffer::McuIndex* mcu, int count, int32_t (*dst)[4])
{
  constexpr int SimdLength = 4;
  typedef Platform::Cpu::SIMD<int32_t, SimdLength> SimdHelper;
  typedef typename SimdHelper::Type SimdType;
  int simdCount = count / SimdLength;
  int x0 = mcu[0].m_x * Dct::BlockSize;
  int y0 = mcu[0].m_y * Dct::BlockSize;

  SimdType zeroLevel = SimdHelper::populate(sampleZeroLevel);
  for (int j = 0; j < Dct::BlockSize; j++)
  {
    const uint32_t* pixelLine = (const uint32_t*)(pixels + (y0 + j) * scanlineLength + x0);
    SimdType* dstLine = (SimdType*)(dst + j * Dct::BlockSize);

    for (int m = 0; m < simdCount; m++)
    {
      SimdHelper::transpose(dstLine,
        SimdType::fromPackedUint8(pixelLine[0]) - zeroLevel,
        SimdType::fromPackedUint8(pixelLine[2]) - zeroLevel,
        SimdType::fromPackedUint8(pixelLine[4]) - zeroLevel,
        SimdType::fromPackedUint8(pixelLine[6]) - zeroLevel);
      SimdHelper::transpose(dstLine + 4,
        SimdType::fromPackedUint8(pixelLine[1]) - zeroLevel,
        SimdType::fromPackedUint8(pixelLine[3]) - zeroLevel,
        SimdType::fromPackedUint8(pixelLine[5]) - zeroLevel,
        SimdType::fromPackedUint8(pixelLine[7]) - zeroLevel);

      pixelLine += Dct::BlockSize;
      dstLine += Dct::BlockSize2;
    }
  }
}

template <>
void grayToYComponentImplementation<int32_t, 4>(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int32_t* dst)
{
  grayToYComponent<4>(imageMetaData, pixels, bufferMetaData, mcu, count, (int32_t(*)[4])dst);
}

// RGB

template<>
void loadRgbSimdLine<4>(Platform::Cpu::int32x4_t* dst, const int32_t (*src)[2][Dct::BlockSize])
{
#if 1
  typedef Platform::Cpu::SIMD<int32_t, 4> SimdHelper;
  if (SimdHelper::isPointerAligned(src))
  {
    SimdHelper::transpose<true, 1, 4>(dst,      src[0][0]);
    SimdHelper::transpose<true, 1, 4>(dst + 4,  src[0][0] + 4);
    SimdHelper::transpose<true, 1, 4>(dst + 8,  src[0][1]);
    SimdHelper::transpose<true, 1, 4>(dst + 12, src[0][1] + 4);
  }
  else
  {
    SimdHelper::transpose<false, 1, 4>(dst,      src[0][0]);
    SimdHelper::transpose<false, 1, 4>(dst + 4,  src[0][0] + 4);
    SimdHelper::transpose<false, 1, 4>(dst + 8,  src[0][1]);
    SimdHelper::transpose<false, 1, 4>(dst + 12, src[0][1] + 4);
  }
#else
  constexpr int SimdLength = 4;
  int32_t (*dsti)[SimdLength] = (int32_t (*)[SimdLength])dst;
  for (int i = 0; i < SimdLength; i++)
  {
    const int32_t *srci = src[i][0];

    for (int j = 0; j < 2 * Dct::BlockSize; j++)
      dsti[j][i] = srci[j];
  }
#endif
}

template<>
void rgbaToYcbr411Implementation<int32_t, 4>(const ImageMetaData& imageMetaData, const uint8_t* rgb, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int32_t* dst, int options)
{
  rgbaToYcbr411Helper<4, int32_t>::perform(imageMetaData, rgb, bufferMetaData, mcu, count, (int32_t (*)[Dct::BlockSize2][4])dst, options);
}

// export
template<>
void exportBlock<4, int32_t>(int16_t (*dst)[Dct::BlockSize2], const int32_t (*values)[4], int mcuBlocks, int count)
{
  exportBlockInt32<4>(dst, values, mcuBlocks, count);
}

template<>
void exportBlocksImplementation<int32_t, 4>(const EncoderBuffer::MetaData& metaData, int16_t (*dst)[Dct::BlockSize2], const int32_t* buffers, int count)
{
  exportBlocks<4>(metaData, dst, (int32_t (*)[Dct::BlockSize2 * 4])buffers, count);
}

#else // defined(PLATFORM_CPU_FEATURE_INT32x8)

template <> void* EncoderBuffer::allocSimdBuffer<int32_t, 4>(int count)
{
  return malloc(sizeof(int32_t) * Dct::BlockSize2 * count);
}

template <> void EncoderBuffer::releaseSimdBuffer<int32_t, 4>(void* buffer)
{
  free(buffer);
}

template <>
void grayToYComponentImplementation<int32_t, 4>(const ImageMetaData&, const uint8_t*, const EncoderBuffer::MetaData&, const EncoderBuffer::McuIndex*, int, int32_t*)
{
  assert(false);
}

template<>
void rgbaToYcbr411Implementation<int32_t, 4>(const ImageMetaData&, const uint8_t*, const EncoderBuffer::MetaData&, const EncoderBuffer::McuIndex*, int, int32_t*, int)
{
  assert(false);
}

template<>
void exportBlocksImplementation<int32_t, 4>(const EncoderBuffer::MetaData&, int16_t (*)[Dct::BlockSize2], const int32_t*, int)
{
  assert(false);
}

#endif // defined(PLATFORM_CPU_FEATURE_INT32x8)

}
