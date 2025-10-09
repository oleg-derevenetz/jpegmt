#include "JpegEncoderBufferTemplates.h"

namespace Jpeg
{

#if defined(PLATFORM_CPU_FEATURE_INT32x8)

// memory

template <> void* allocSimdBuffer<int32_t, 8>(int count)
{
  return Platform::Cpu::SIMD<int32_t, 8>::template allocMemory<int32_t[Dct::BlockSize2 * 8]>(count);
}

template <> void releaseSimdBuffer<int32_t, 8>(void* buffer)
{
  Platform::Cpu::SIMD<int32_t, 8>::freeMemory(buffer);
}

// Grayscale

#ifdef TRANSPOSED_SIMD_BUFFER
template<>
void grayScanlineSimdToYComponent<Dct::BlockSize, int32_t>(const uint8_t* pixels, int scanlineLength, const EncoderBuffer::McuIndex* mcu, int count, int32_t(*dst)[Dct::BlockSize])
{
  grayScanlineSimdToYComponentSquare(pixels, scanlineLength, mcu, count, dst);
}
#endif

template <>
void grayToYComponentImplementation<int32_t, 8>(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int32_t* dst)
{
  grayToYComponent<8>(imageMetaData, pixels, bufferMetaData, mcu, count, (int32_t(*)[8])dst);
}

// RGB

template<>
void loadRgbSimdLine<Dct::BlockSize>(Platform::Cpu::int32x8_t* dst, const int32_t (*src)[2][Dct::BlockSize])
{
  constexpr int SimdLength = Dct::BlockSize;
#if 1
  typedef Platform::Cpu::SIMD<int32_t, SimdLength> SimdHelper;
  if (SimdHelper::isPointerAligned(src))
  {
    SimdHelper::transpose<true, 1, 2>(dst, src[0][0]);
    SimdHelper::transpose<true, 1, 2>(dst + SimdLength, src[0][1]);
  }
  else
  {
    SimdHelper::transpose<false, 1, 2>(dst, src[0][0]);
    SimdHelper::transpose<false, 1, 2>(dst + SimdLength, src[0][1]);
  }
#else
  int32_t (*dsti)[SimdLength] = (int32_t(*)[SimdLength])dst;
  for (int i = 0; i < SimdLength; i++)
  {
    const int32_t *srci = src + i * 2 * SimdLength;

    for (int j = 0; j < SimdLength; j++)
      dsti[j][i] = srci[j];
    for (int j = 0; j < SimdLength; j++)
      dsti[j + SimdLength][i] = srci[j + SimdLength];
  }
#endif
}

template<>
void rgbaToYcbr411Implementation<int32_t, 8>(const ImageMetaData& imageMetaData, const uint8_t* rgb, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int32_t* dst, int options)
{
  rgbaToYcbr411Helper<8, int32_t>::perform(imageMetaData, rgb, bufferMetaData, mcu, count, (int32_t (*)[Dct::BlockSize2][8])dst, options);
}

// export

template<>
void exportBlock<8, int32_t>(int16_t (*dst)[Dct::BlockSize2], const int32_t (*values)[8], int mcuBlocks, int count)
{
  exportBlockInt32<8>(dst, values, mcuBlocks, count);
}

template<>
void exportBlocksImplementation<int32_t, 8>(const EncoderBuffer::MetaData& metaData, int16_t (*dst)[Dct::BlockSize2], const int32_t* buffers, int count)
{
  exportBlocks<8>(metaData, dst, (int32_t (*)[Dct::BlockSize2 * 8])buffers, count);
}

#else // defined(PLATFORM_CPU_FEATURE_INT32x8)

template <> void* allocSimdBuffer<int32_t, 8>(int count)
{
  return malloc(sizeof(int32_t) * Dct::BlockSize2 * 8 * count);
}

template <> void releaseSimdBuffer<int32_t, 8>(void* buffer)
{
  free(buffer);
}

template <>
void grayToYComponentImplementation<int32_t, 8>(const ImageMetaData&, const uint8_t*, const EncoderBuffer::MetaData&, const EncoderBuffer::McuIndex*, int, int32_t*)
{
  assert(false);
}

template<>
void rgbaToYcbr411Implementation<int32_t, 8>(const ImageMetaData&, const uint8_t*, const EncoderBuffer::MetaData&, const EncoderBuffer::McuIndex*, int, int32_t*, int)
{
  assert(false);
}

template<>
void exportBlocksImplementation<int32_t, 8>(const EncoderBuffer::MetaData&, int16_t (*)[Dct::BlockSize2], const int32_t*, int)
{
  assert(false);
}

#endif // defined(PLATFORM_CPU_FEATURE_INT32x8)

template<>
int EncoderBuffer::MetaData::detectSimdLength<int32_t>(uint64_t features, int lengthLimit)
{
  return Platform::Cpu::SimdDetector<int32_t>::maxSimdLength(features, lengthLimit);
}

}
