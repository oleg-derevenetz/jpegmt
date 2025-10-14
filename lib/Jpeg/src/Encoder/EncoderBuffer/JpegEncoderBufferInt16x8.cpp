#include "JpegEncoderBufferTemplates.h"

namespace Jpeg
{
#if defined(PLATFORM_CPU_FEATURE_INT16x8)

// memory

template <> void* EncoderBuffer::allocSimdBuffer<int16_t, 8>(int count)
{
  return Platform::Cpu::SIMD<int16_t, 8>::template allocMemory<int16_t[Dct::BlockSize2]>(count);
}

template <> void EncoderBuffer::releaseSimdBuffer<int16_t, 8>(void* buffer)
{
  Platform::Cpu::SIMD<int16_t, 8>::freeMemory(buffer);
}

// Grayscale

template<>
void grayScanlineSimdToYComponent<Dct::BlockSize, int16_t>(const uint8_t* pixels, int scanlineLength, const EncoderBuffer::McuIndex* mcu, int count, int16_t(*dst)[Dct::BlockSize])
{
  grayScanlineSimdToYComponentSquare(pixels, scanlineLength, mcu, count, dst);
}

template <>
void grayToYComponentImplementation<int16_t, 8>(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int16_t* dst)
{
  grayToYComponent<8>(imageMetaData, pixels, bufferMetaData, mcu, count, (int16_t(*)[8])dst);
}

// RGB

template <bool aligned, int block>
void loadRgbSimdBlocks(Platform::Cpu::int32x4_t(*matrix)[Dct::BlockSize][2], const int32_t(*blocks)[2][Dct::BlockSize])
{
  typedef Platform::Cpu::SIMD<int32_t, 4> SimdHelper;
  SimdHelper::transpose<aligned, 2, 4>(matrix[block][0], blocks[0][block]);
  SimdHelper::transpose<aligned, 2, 4>(matrix[block][4], blocks[0][block] + 4);
  SimdHelper::transpose<aligned, 2, 4>(matrix[block][0] + 1, blocks[4][block]);
  SimdHelper::transpose<aligned, 2, 4>(matrix[block][4] + 1, blocks[4][block] + 4);
}

template<>
void loadRgbSimdLine<Dct::BlockSize, Platform::Cpu::int32x4_t>(Platform::Cpu::int32x4_t* dst, const int32_t(*src)[2][Dct::BlockSize])
{
  Platform::Cpu::int32x4_t(*matrix)[Dct::BlockSize][2] = (Platform::Cpu::int32x4_t(*)[Dct::BlockSize][2])dst;
  if (Platform::Cpu::SIMD<int32_t, 4>::isPointerAligned(src))
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
void rgbaToYcbr411Implementation<int16_t, 8>(const ImageMetaData& imageMetaData, const uint8_t* rgb, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int16_t* dst, int options)
{
  rgbaToYcbr411Helper<8, int16_t>::perform(imageMetaData, rgb, bufferMetaData, mcu, count, (int16_t (*)[Dct::BlockSize2][8])dst, options);
}

// export

template<>
void exportBlock<Dct::BlockSize, int16_t>(int16_t (*dst)[Dct::BlockSize2], const int16_t (*values)[Dct::BlockSize], int mcuBlocks, int count)
{
  constexpr int SimdLength = Dct::BlockSize;
  typedef Platform::Cpu::SIMD<int16_t, SimdLength> SimdHelper;
  typedef typename SimdHelper::Type SimdType;
  typename SimdHelper::NativeType matrix[SimdLength];

  if (count >= SimdLength)
  {
    for (int j = 0; j < Dct::BlockSize; j++)
    {
      int k0 = j * Dct::BlockSize;
      SimdHelper::transpose<true>((SimdType*)matrix, values[k0]);

      for (int n = 0; n < SimdLength; n++)
        SimdHelper::store(dst[n * mcuBlocks] + k0, matrix[n]);
    }
  }
  else
  {
    for (int j = 0; j < Dct::BlockSize; j++)
    {
      int k0 = j * Dct::BlockSize;
      SimdHelper::transpose<true>((SimdType*)matrix, values[k0]);

      for (int n = 0; n < count; n++)
        SimdHelper::store(dst[n * mcuBlocks] + k0, matrix[n]);
    }
  }
}

template<>
void exportBlocksImplementation<int16_t, 8>(const EncoderBuffer::MetaData& metaData, int16_t (*dst)[Dct::BlockSize2], const int16_t* buffers, int count)
{
  exportBlocks<8>(metaData, dst, (int16_t(*)[Dct::BlockSize2 * 8])buffers, count);
}

#else // defined(PLATFORM_CPU_FEATURE_INT16x8)

template <> void* EncoderBuffer::allocSimdBuffer<int16_t, 8>(int count)
{
  return malloc(sizeof(int16_t) * Dct::BlockSize2 * count);
}

template <> void EncoderBuffer::releaseSimdBuffer<int16_t, 8>(void* buffer)
{
  free(buffer);
}

template <>
void grayToYComponentImplementation<int16_t, 8>(const ImageMetaData&, const uint8_t*, const EncoderBuffer::MetaData&, const EncoderBuffer::McuIndex*, int, int16_t*)
{
  assert(false);
}

template<>
void rgbaToYcbr411Implementation<int16_t, 8>(const ImageMetaData&, const uint8_t*, const EncoderBuffer::MetaData&, const EncoderBuffer::McuIndex*, int, int16_t*, int)
{
  assert(false);
}

template<>
void exportBlocksImplementation<int16_t, 8>(const EncoderBuffer::MetaData&, int16_t (*)[Dct::BlockSize2], const int16_t*, int)
{
  assert(false);
}

#endif // defined(PLATFORM_CPU_FEATURE_INT16x8)
}
