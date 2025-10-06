#include "JpegEncoderBuffer.h"

#include <cassert>
#include <cstring>

#include <Helper/Platform/Cpu/simd.h>

#include <Jpeg/JpegImageMetaData.h>

#include "../Common.h"
#include "RgbToYcc.h"

namespace Jpeg
{

constexpr int sampleZeroLevel = 128;

static inline int getScanlineSimdMcuCount(const EncoderBuffer::McuIndex* mcu, int count, const EncoderBuffer::MetaData& bufferMetaData, int imageWidth, int imageHeight)
{
#ifndef TRANSPOSED_SIMD_BUFFER
  constexpr int mcuBlockLength = 1;
#else
  int mcuBlockLength = bufferMetaData.m_simdLength;
#endif

  if ((mcu[0].m_y + 1) * bufferMetaData.m_vScale * Dct::BlockSize > imageHeight)
    return 0;

  int simdBufferCount = 0;
  for (int y0 = mcu[0].m_y; count >= mcuBlockLength; simdBufferCount++, mcu += mcuBlockLength, count -= mcuBlockLength)
  {
    if (mcu[mcuBlockLength - 1].m_y != y0 || (mcu[mcuBlockLength - 1].m_x + 1) * bufferMetaData.m_hScale * Dct::BlockSize > imageWidth)
      break;
  }

  return simdBufferCount * mcuBlockLength;
}

// memory

template <typename T, int SimdLength> void* allocSimdBuffer(int count);
template <typename T, int SimdLength> void releaseSimdBuffer(void* buffer);

// Grayscale

template<typename T>
static void grayToYComponent(const ImageMetaData& imageMetaData, const uint8_t* rgb, int x0, int y0, T* component, int outStride)
{
  int xoffsets[Dct::BlockSize];
  for (int i = 0; i < Dct::BlockSize; i++)
    xoffsets[i] = x0 + i < imageMetaData.m_size.m_width ? x0 + i : imageMetaData.m_size.m_width - 1;

  for (int j = 0; j < Dct::BlockSize; j++)
  {
    int y = y0 + j < imageMetaData.m_size.m_height ? y0 + j : imageMetaData.m_size.m_height - 1;
    const uint8_t* grayLine = rgb + y * imageMetaData.m_scanlineBytes;
    T* outLine = component + j * Dct::BlockSize * outStride;

    for (int i = 0; i < Dct::BlockSize; i++)
      outLine[i * outStride] = grayLine[xoffsets[i]] - sampleZeroLevel;
  }
}

#ifdef TRANSPOSED_SIMD_BUFFER
template<int SimdLength, typename T>
static void grayScanlineSimdToYComponent(const uint8_t* pixels, int scanlineLength, const EncoderBuffer::McuIndex* mcu, int count, T (*dst)[SimdLength])
{
  int simdCount = count / SimdLength;
  int x0 = mcu[0].m_x * Dct::BlockSize;
  int y0 = mcu[0].m_y * Dct::BlockSize;

  for (int j = 0; j < Dct::BlockSize; j++)
  {
    const uint8_t* pixelLine = pixels + (y0 + j) * scanlineLength + x0;
    T(*dstLine)[SimdLength] = dst + j * Dct::BlockSize;

    for (int m = 0; m < simdCount; m++)
    {
#if 1
      for (int n = 0; n < SimdLength; n++)
      {
        const uint8_t* pixelLineBlock = pixelLine + n * Dct::BlockSize;

        for (int i = 0; i < Dct::BlockSize; i++)
          dstLine[i][n] = pixelLineBlock[i] - sampleZeroLevel;
      }
#else
      for (int k = 0; k < Dct::BlockSize * SimdLength; k++)
        dstLine[k & ((1 << Dct::BlockSizeLog2) - 1)][k >> Dct::BlockSizeLog2] = pixelLine[k] - sampleZeroLevel;
#endif
      pixelLine += Dct::BlockSize * SimdLength;
      dstLine += Dct::BlockSize2;
    }
  }
}

template<typename T>
static void grayScanlineSimdToYComponentSquare(const uint8_t* pixels, int scanlineLength, const EncoderBuffer::McuIndex* mcu, int count, T(*dst)[Dct::BlockSize])
{
  constexpr int SimdLength = Dct::BlockSize;
  typedef Platform::Cpu::SIMD<T, SimdLength> SimdHelper;
  typedef typename SimdHelper::Type SimdType;
  int simdCount = count / SimdLength;
  int x0 = mcu[0].m_x * Dct::BlockSize;
  int y0 = mcu[0].m_y * Dct::BlockSize;

  SimdType zeroLevel = SimdHelper::populate(sampleZeroLevel);
  for (int j = 0; j < Dct::BlockSize; j++)
  {
    const uint64_t* pixelLine = (const uint64_t*)(pixels + (y0 + j) * scanlineLength + x0);
    SimdType* dstLine = (SimdType*)(dst + j * Dct::BlockSize);

    for (int m = 0; m < simdCount; m++)
    {
      SimdHelper::transpose(dstLine,
        SimdType::fromPackedUint8(pixelLine[0]) - zeroLevel, SimdType::fromPackedUint8(pixelLine[1]) - zeroLevel,
        SimdType::fromPackedUint8(pixelLine[2]) - zeroLevel, SimdType::fromPackedUint8(pixelLine[3]) - zeroLevel,
        SimdType::fromPackedUint8(pixelLine[4]) - zeroLevel, SimdType::fromPackedUint8(pixelLine[5]) - zeroLevel,
        SimdType::fromPackedUint8(pixelLine[6]) - zeroLevel, SimdType::fromPackedUint8(pixelLine[7]) - zeroLevel);

      pixelLine += Dct::BlockSize;
      dstLine += Dct::BlockSize2;
    }
  }
}

template<int SimdLength, typename T>
static void grayToYComponent(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, T (*dst)[SimdLength])
{
  for (; count > 0;)
  {
    int simdMcuCount = getScanlineSimdMcuCount(mcu, count, bufferMetaData, imageMetaData.m_size.m_width, imageMetaData.m_size.m_height);
    if (simdMcuCount > 0)
      grayScanlineSimdToYComponent<SimdLength>(pixels, imageMetaData.m_scanlineBytes, mcu, simdMcuCount, dst);
    mcu += simdMcuCount; count -= simdMcuCount; dst += Dct::BlockSize2 * simdMcuCount / SimdLength * bufferMetaData.m_mcuBlockCount;

    int wrapMcuCount = std::min(count, SimdLength);
    for (int j = 0; j < wrapMcuCount; j++)
      grayToYComponent(imageMetaData, pixels, mcu[j].m_x * Dct::BlockSize, mcu[j].m_y * Dct::BlockSize, dst[0] + j, SimdLength);
    mcu += SimdLength; count -= SimdLength; dst += Dct::BlockSize2 * bufferMetaData.m_mcuBlockCount;
  }
}

template<typename T, int SimdLength>
void grayToYComponentImplementation(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, T* dst);

// RGB

static inline Rgb8ToYccTable makeRgb8ToYccTable()
{
  Rgb8ToYccTable table;

  for (int i = 0; i < 256; i++)
  {
    table.m_yTable.m_rWeights[i] = Rgb8ToYcc::yrWeight * i;
    table.m_yTable.m_gWeights[i] = Rgb8ToYcc::ygWeight * i;
    table.m_yTable.m_bWeights[i] = Rgb8ToYcc::ybWeight * i + Rgb8ToYcc::fixedPointOneHalf;
    table.m_cbTable.m_rWeights[i] = Rgb8ToYcc::cbrWeight * i;
    table.m_cbTable.m_gWeights[i] = Rgb8ToYcc::cbgWeight * i;
    table.m_cbTable.m_bWeights[i] = Rgb8ToYcc::fixedPointFromFloat(0.50000) * i + Rgb8ToYcc::cbcr8Offset + Rgb8ToYcc::fixedPointOneHalf - 1;
    table.m_crTable.m_rWeights[i] = table.m_cbTable.m_bWeights[i];
    table.m_crTable.m_gWeights[i] = Rgb8ToYcc::crgWeight * i;
    table.m_crTable.m_bWeights[i] = Rgb8ToYcc::crbWeight * i;
  }

  return table;
}

template<ImageMetaData::Format rgbFormat, bool evenLinePadding, int SimdLength, typename T>
FORCE_INLINE static void convert2x2RgbaToYcbr411(T(*dst)[Dct::BlockSize2][SimdLength], int i, int j, int n, uint32_t rgb0, uint32_t rgb1, uint32_t rgb2, uint32_t rgb3, int bias)
{
  static Rgb8ToYccTable rgb8ToYcc = makeRgb8ToYccTable();
  constexpr int yComponentOffset = 0;
  constexpr int cbComponentOffset = 4;
  constexpr int crComponentOffset = 5;

  // Y component
  int subBlock = (2 * j / Dct::BlockSize) * 2 + 2 * i / Dct::BlockSize;
  T(*ydst)[SimdLength] = dst[yComponentOffset + subBlock];
  int yx0 = 2 * i % Dct::BlockSize, yy0 = 2 * j % Dct::BlockSize;
  ydst[(yy0 + 0) * Dct::BlockSize + yx0 + 0][n] = rgb8ToYcc.rgbToY<rgbFormat>(evenLinePadding ? rgb2 : rgb0) - sampleZeroLevel;
  ydst[(yy0 + 0) * Dct::BlockSize + yx0 + 1][n] = rgb8ToYcc.rgbToY<rgbFormat>(evenLinePadding ? rgb3 : rgb1) - sampleZeroLevel;
  ydst[(yy0 + 1) * Dct::BlockSize + yx0 + 0][n] = rgb8ToYcc.rgbToY<rgbFormat>(rgb2) - sampleZeroLevel;
  ydst[(yy0 + 1) * Dct::BlockSize + yx0 + 1][n] = rgb8ToYcc.rgbToY<rgbFormat>(rgb3) - sampleZeroLevel;

  int k = j * Dct::BlockSize + i;
  // Cb component
  dst[cbComponentOffset][k][n] = ((rgb8ToYcc.rgbToCb<rgbFormat>(rgb0) + rgb8ToYcc.rgbToCb<rgbFormat>(rgb1) + rgb8ToYcc.rgbToCb<rgbFormat>(rgb2) + rgb8ToYcc.rgbToCb<rgbFormat>(rgb3) + bias) >> 2) - sampleZeroLevel;

  // Cr component
  dst[crComponentOffset][k][n] = ((rgb8ToYcc.rgbToCr<rgbFormat>(rgb0) + rgb8ToYcc.rgbToCr<rgbFormat>(rgb1) + rgb8ToYcc.rgbToCr<rgbFormat>(rgb2) + rgb8ToYcc.rgbToCr<rgbFormat>(rgb3) + bias) >> 2) - sampleZeroLevel;
}

template<int SimdLength, typename DstType>
static void loadRgbSimdLine(DstType* dst, const int32_t(*blocks)[2][Dct::BlockSize]);

template<int SimdLength>
struct RgbComponent
{
  typedef typename Platform::Cpu::SIMD<uint8_t, SimdLength>::Type Type;
};

template<>
struct RgbComponent<8>
{
  typedef uint64_t Type;
};

template<>
struct RgbComponent<4>
{
  typedef uint32_t Type;
};

template <ImageMetaData::Format rgbFormat, typename T, int SimdLength> struct RgbComponentExtractor;

template <ImageMetaData::Format rgbFormat, int SimdLength>
struct RgbComponentExtractor<rgbFormat, int16_t, SimdLength>
{
  constexpr static int RgbSimdLength = SimdLength / 2;

  static void extract(const typename Platform::Cpu::SIMD<int32_t, RgbSimdLength>::NativeType* rgb,
    typename Platform::Cpu::SIMD<int16_t, SimdLength>::Type& r, typename Platform::Cpu::SIMD<int16_t, SimdLength>::Type& g, typename Platform::Cpu::SIMD<int16_t, SimdLength>::Type& b)
  {
    using namespace Platform::Cpu;
    typedef SIMD<int32_t, SimdLength / 2> RgbSimdHelper;
    typename RgbComponent<SimdLength>::Type r8, g8, b8, a8;

    if (rgbFormat == ImageMetaData::Rgba32)
    {
      if (Platform::Cpu::byteOrder == Platform::Cpu::LittleEndian)
        RgbSimdHelper::extractByteComponents(rgb[0], rgb[1], r8, g8, b8, a8);
      else
        RgbSimdHelper::extractByteComponents(rgb[0], rgb[1], a8, b8, g8, r8);
    }
    if (rgbFormat == ImageMetaData::Bgra32)
    {
      if (Platform::Cpu::byteOrder == Platform::Cpu::LittleEndian)
        RgbSimdHelper::extractByteComponents(rgb[0], rgb[1], b8, g8, r8, a8);
      else
        RgbSimdHelper::extractByteComponents(rgb[0], rgb[1], a8, r8, g8, b8);
    }

    r.setFromPackedUint8(r8);
    g.setFromPackedUint8(g8);
    b.setFromPackedUint8(b8);
  }
};

template <ImageMetaData::Format rgbFormat, int SimdLength>
struct RgbComponentExtractor<rgbFormat, int32_t, SimdLength>
{
  static void extract(const typename Platform::Cpu::SIMD<int32_t, SimdLength>::NativeType* rgb,
    typename Platform::Cpu::SIMD<int32_t, SimdLength>::Type& r, typename Platform::Cpu::SIMD<int32_t, SimdLength>::Type& g, typename Platform::Cpu::SIMD<int32_t, SimdLength>::Type& b)
  {
    using namespace Platform::Cpu;
    typedef SIMD<int32_t, SimdLength> RgbSimdHelper;
    typename RgbComponent<SimdLength>::Type r8, g8, b8, a8;

    if (rgbFormat == ImageMetaData::Rgba32)
    {
      if (Platform::Cpu::byteOrder == Platform::Cpu::LittleEndian)
        RgbSimdHelper::extractByteComponents(rgb[0], r8, g8, b8, a8);
      else
        RgbSimdHelper::extractByteComponents(rgb[0], a8, b8, g8, r8);
    }
    if (rgbFormat == ImageMetaData::Bgra32)
    {
      if (Platform::Cpu::byteOrder == Platform::Cpu::LittleEndian)
        RgbSimdHelper::extractByteComponents(rgb[0], b8, g8, r8, a8);
      else
        RgbSimdHelper::extractByteComponents(rgb[0], a8, r8, g8, b8);
    }

    r.setFromPackedUint8(r8);
    g.setFromPackedUint8(g8);
    b.setFromPackedUint8(b8);
  }
};

//#define PER_COMPONENT_TRANSPOSING

#ifdef PER_COMPONENT_TRANSPOSING
template<ImageMetaData::Format rgbFormat, bool aligned>
void loadRgbBlock(Platform::Cpu::uint8x16_t(*r)[2], Platform::Cpu::uint8x16_t(*g)[2], Platform::Cpu::uint8x16_t(*b)[2], const int32_t(*src)[2][Dct::BlockSize])
{
  using namespace Platform::Cpu;

  for (int i = 0; i < Dct::BlockSize; i++)
  {
    extractRgbComponents<rgbFormat, Dct::BlockSize * 2>(int32x8_t::load<aligned>(src[i][0]), int32x8_t::load<aligned>(src[i + Dct::BlockSize][0]), r[i][0], g[i][0], b[i][0]);
    extractRgbComponents<rgbFormat, Dct::BlockSize * 2>(int32x8_t::load<aligned>(src[i][1]), int32x8_t::load<aligned>(src[i + Dct::BlockSize][1]), r[i][1], g[i][1], b[i][1]);
  }
}

template<ImageMetaData::Format rgbFormat>
static void loadRgbSimdLine(Platform::Cpu::int8x32_t* r, Platform::Cpu::int8x32_t* g, Platform::Cpu::int8x32_t* b, const int32_t(*src)[2][Dct::BlockSize])
{
  using namespace Platform::Cpu;
  typedef SIMD<int32_t, Dct::BlockSize> SimdHelper;
  int8x32_t r8[Dct::BlockSize], g8[Dct::BlockSize], b8[Dct::BlockSize];

  if (SimdHelper::isPointerAligned(src))
    loadRgbBlock<rgbFormat, true>((uint8x16_t(*)[2])r8, (uint8x16_t(*)[2])g8, (uint8x16_t(*)[2])b8, src);
  else
    loadRgbBlock<rgbFormat, false>((uint8x16_t(*)[2])r8, (uint8x16_t(*)[2])g8, (uint8x16_t(*)[2])b8, src);

  SIMD<int8_t, 32>::transpose4x8x8(r, r8[0], r8[1], r8[2], r8[3], r8[4], r8[5], r8[6], r8[7]);
  SIMD<int8_t, 32>::transpose4x8x8(g, g8[0], g8[1], g8[2], g8[3], g8[4], g8[5], g8[6], g8[7]);
  SIMD<int8_t, 32>::transpose4x8x8(b, b8[0], b8[1], b8[2], b8[3], b8[4], b8[5], b8[6], b8[7]);
}
#endif // PER_COMPONENT_TRANSPOSING

template<int SimdLength, typename T>
FORCE_INLINE static void rgbToYcbr(typename Platform::Cpu::SIMD<T, SimdLength>::Type r, typename Platform::Cpu::SIMD<T, SimdLength>::Type g, typename Platform::Cpu::SIMD<T, SimdLength>::Type b,
  typename Platform::Cpu::SIMD<T, SimdLength>::Type& y, typename Platform::Cpu::SIMD<T, SimdLength>::Type& cb, typename Platform::Cpu::SIMD<T, SimdLength>::Type& cr)
{
  y = RgbToYcc<T, SimdLength, 0>::y(r, g, b);
  cb = RgbToYcc<T, SimdLength, 0>::cb(r, g, b);
  cr = RgbToYcc<T, SimdLength, 0>::cr(r, g, b);
}

namespace
{
  enum RgbToYccOptions
  {
    AverageInRgbSpace = 0x1,
  };

  template<ImageMetaData::Format rgbFormat, int options, int SimdLength, bool evenLinePadding, typename T>
  struct RgbaScanlinePairToYcbr411
  {
    static void perform(int j, const int32_t* rgb0, const int32_t* rgb1, int nsimd, T (*dst)[Dct::BlockSize2][SimdLength])
    {
      constexpr int componentBlockCount = 6;

#if 1
      constexpr int yComponentOffset = 0;
      constexpr int cbComponentOffset = 4;
      constexpr int crComponentOffset = 5;
      constexpr int RgbSimdLength = std::max<int>(1, SimdLength * sizeof(T) / sizeof(int32_t)); //rgbSimdLength<SimdLength>();

      typedef Platform::Cpu::SIMD<int32_t, RgbSimdLength> RgbSimdHelper;
      typedef typename RgbSimdHelper::Type RgbSimdType;
      typedef Platform::Cpu::SIMD<T, SimdLength> SimdHelper;
      typedef typename SimdHelper::Type SimdType;
      using namespace Platform::Cpu::int32;

      int ySubBlock = (2 * j / Dct::BlockSize) * 2;
      int yy0 = 2 * j % Dct::BlockSize;

      for (int m = 0; m < nsimd; m++)
      {
        T (*dstm)[Dct::BlockSize2][SimdLength] = dst + m * componentBlockCount;
        const int32_t (*rgb0m)[2][Dct::BlockSize] = ((int32_t(*)[2][Dct::BlockSize])rgb0) + SimdLength * m;
        const int32_t (*rgb1m)[2][Dct::BlockSize] = ((int32_t(*)[2][Dct::BlockSize])rgb1) + SimdLength * m;
#ifndef PER_COMPONENT_TRANSPOSING
        typename RgbSimdHelper::NativeType matrix0[Dct::BlockSize * 2][SimdLength / RgbSimdLength], matrix1[Dct::BlockSize * 2][SimdLength / RgbSimdLength];

        loadRgbSimdLine<SimdLength>((RgbSimdType*)matrix0, rgb0m);
        loadRgbSimdLine<SimdLength>((RgbSimdType*)matrix1, rgb1m);
#else
        typedef typename Platform::Cpu::SIMD<int8_t, SimdLength * 2>::Type RgbComponentType;
        typedef typename Platform::Cpu::SIMD<uint8_t, SimdLength>::NativeType RgbComponentMatrixType[Dct::BlockSize * 2];
        RgbComponentMatrixType r0, g0, b0, r1, g1, b1;
        loadRgbSimdLine<rgbFormat>((RgbComponentType*)r0, (RgbComponentType*)g0, (RgbComponentType*)b0, rgb0m);
        loadRgbSimdLine<rgbFormat>((RgbComponentType*)r1, (RgbComponentType*)g1, (RgbComponentType*)b1, rgb1m);
#endif

        for (int i = 0, bias = 1; i < Dct::BlockSize; i++, bias ^= 3)
        {
//          for (int n = 0; n < SimdLength; n++)
//            convert2x2RgbaToYcbr411<rgbFormat>(dstm, i, j, n, ((uint32_t (*)[SimdLength])matrix0)[i*2][n], ((uint32_t(*)[SimdLength])matrix0)[i*2 + 1][n], ((uint32_t(*)[SimdLength])matrix1)[i*2][n], ((uint32_t(*)[SimdLength])matrix1)[i*2 + 1][n], bias);

          int subBlock = ySubBlock + 2 * i / Dct::BlockSize;
          int yx0 = 2 * i % Dct::BlockSize;
          T (*ydst)[SimdLength] = dstm[yComponentOffset + subBlock];

#ifdef PER_COMPONENT_TRANSPOSING
          int k0 = (i * 4) % 16 + i / 4;
          int k1 = k0 + 2;
          SimdType r = SimdType::fromPackedUint8(r0[k0]), g = SimdType::fromPackedUint8(g0[k0]), b = SimdType::fromPackedUint8(b0[k0]);
#else
          SimdType r, g, b;
          RgbComponentExtractor<rgbFormat, T, SimdLength>::extract(matrix0[2 * i], r, g, b);
#endif
          if (!(options & AverageInRgbSpace))
          {
            // TODO: 2 + 1 transpose 4x8x8 for int_8 yx2 + (cb + cr)x1 instead of 2x4 tranpose for int32 rgb
            SimdType y, cb, cr, cbsum, crsum;
            rgbToYcbr<SimdLength, T>(r, g, b, y, cbsum, crsum);
            if (!evenLinePadding)
              SimdHelper::store(ydst[(yy0 + 0) * Dct::BlockSize + yx0 + 0], y);

            RgbComponentExtractor<rgbFormat, T, SimdLength>::extract(matrix0[2 * i + 1], r, g, b);
            rgbToYcbr<SimdLength, T>(r, g, b, y, cb, cr);
            if (!evenLinePadding)
              SimdHelper::store(ydst[(yy0 + 0) * Dct::BlockSize + yx0 + 1], y);
            cbsum += cb; crsum += cr;

            RgbComponentExtractor<rgbFormat, T, SimdLength>::extract(matrix1[2 * i], r, g, b);
            rgbToYcbr<SimdLength, T>(r, g, b, y, cb, cr);
            if (evenLinePadding)
              SimdHelper::store(ydst[(yy0 + 0) * Dct::BlockSize + yx0 + 0], y);
            SimdHelper::store(ydst[(yy0 + 1) * Dct::BlockSize + yx0 + 0], y);
            cbsum += cb; crsum += cr;

            RgbComponentExtractor<rgbFormat, T, SimdLength>::extract(matrix1[2 * i + 1], r, g, b);
            rgbToYcbr<SimdLength, T>(r, g, b, y, cb, cr);
            if (evenLinePadding)
              SimdHelper::store(ydst[(yy0 + 0) * Dct::BlockSize + yx0 + 1], y);
            SimdHelper::store(ydst[(yy0 + 1) * Dct::BlockSize + yx0 + 1], y);
            cbsum += cb; crsum += cr;

            int k = j * Dct::BlockSize + i;
            SimdType simdBias = SimdHelper::populate(bias);
            SimdHelper::store(dstm[cbComponentOffset][k], (cbsum + simdBias) >> 2);
            SimdHelper::store(dstm[crComponentOffset][k], (crsum + simdBias) >> 2);
          }
          else
          {
            if (!evenLinePadding)
              SimdHelper::store(ydst[(yy0 + 0) * Dct::BlockSize + yx0 + 0], RgbToYcc<T, SimdLength, 0>::y(r, g, b));

#ifdef PER_COMPONENT_TRANSPOSING
            SimdType ri = SimdType::fromPackedUint8(r0[k1]), gi = SimdType::fromPackedUint8(g0[k1]), bi = SimdType::fromPackedUint8(b0[k1]);
#else
            SimdType ri, gi, bi;
            RgbComponentExtractor<rgbFormat, T, SimdLength>::extract(matrix0[2 * i + 1], ri, gi, bi);
#endif
            if (!evenLinePadding)
              SimdHelper::store(ydst[(yy0 + 0) * Dct::BlockSize + yx0 + 1], RgbToYcc<T, SimdLength, 0>::y(ri, gi, bi));
            r += ri; g += gi; b += bi;

#ifdef PER_COMPONENT_TRANSPOSING
            ri = SimdType::fromPackedUint8(r1[k0]), gi = SimdType::fromPackedUint8(g1[k0]), bi = SimdType::fromPackedUint8(b1[k0]);
#else
            RgbComponentExtractor<rgbFormat, T, SimdLength>::extract(matrix1[2 * i], ri, gi, bi);
#endif
            SimdType y = RgbToYcc<T, SimdLength, 0>::y(ri, gi, bi);
            if (evenLinePadding)
              SimdHelper::store(ydst[(yy0 + 0) * Dct::BlockSize + yx0 + 0], y);
            SimdHelper::store(ydst[(yy0 + 1) * Dct::BlockSize + yx0 + 0], y);
            r += ri; g += gi; b += bi;

#ifdef PER_COMPONENT_TRANSPOSING
            ri = SimdType::fromPackedUint8(r1[k1]), gi = SimdType::fromPackedUint8(g1[k1]), bi = SimdType::fromPackedUint8(b1[k1]);
#else
            RgbComponentExtractor<rgbFormat, T, SimdLength>::extract(matrix1[2 * i + 1], ri, gi, bi);
#endif
            y = RgbToYcc<T, SimdLength, 0>::y(ri, gi, bi);
            if (evenLinePadding)
              SimdHelper::store(ydst[(yy0 + 0) * Dct::BlockSize + yx0 + 1], y);
            SimdHelper::store(ydst[(yy0 + 1) * Dct::BlockSize + yx0 + 1], y);
            r += ri; g += gi; b += bi;

            int k = j * Dct::BlockSize + i;
            SimdHelper::store(dstm[cbComponentOffset][k], RgbToYcc<T, SimdLength, 2>::cb(r, g, b, bias));
            SimdHelper::store(dstm[crComponentOffset][k], RgbToYcc<T, SimdLength, 2>::cr(r, g, b, bias));
          }
        }
      }
#else
      for (int m = 0; m < nsimd; m++)
      {
        const uint32_t* rgbLine0 = (const uint32_t*)rgb0 + SimdLength * 2 * Dct::BlockSize * m;
        const uint32_t* rgbLine1 = (const uint32_t*)rgb1 + SimdLength * 2 * Dct::BlockSize * m;
        T(*dstm)[Dct::BlockSize2][SimdLength] = dst + m * componentBlockCount;

#if 0
        for (int n = 0, x = 0; n < SimdLength; n++, x += 2 * Dct::BlockSize)
        {
          if (Dct::BlockSize == 8)
          {
            convert2x2RgbaToYcbr411<rgbFormat, evenLinePadding>(dstm, 0, j, n, rgbLine0[x], rgbLine0[x + 1], rgbLine1[x], rgbLine1[x + 1], 1);
            convert2x2RgbaToYcbr411<rgbFormat, evenLinePadding>(dstm, 1, j, n, rgbLine0[x + 2], rgbLine0[x + 3], rgbLine1[x + 2], rgbLine1[x + 3], 2);
            convert2x2RgbaToYcbr411<rgbFormat, evenLinePadding>(dstm, 2, j, n, rgbLine0[x + 4], rgbLine0[x + 5], rgbLine1[x + 4], rgbLine1[x + 5], 1);
            convert2x2RgbaToYcbr411<rgbFormat, evenLinePadding>(dstm, 3, j, n, rgbLine0[x + 6], rgbLine0[x + 7], rgbLine1[x + 6], rgbLine1[x + 7], 2);
            convert2x2RgbaToYcbr411<rgbFormat, evenLinePadding>(dstm, 4, j, n, rgbLine0[x + 8], rgbLine0[x + 9], rgbLine1[x + 8], rgbLine1[x + 9], 1);
            convert2x2RgbaToYcbr411<rgbFormat, evenLinePadding>(dstm, 5, j, n, rgbLine0[x + 10], rgbLine0[x + 11], rgbLine1[x + 10], rgbLine1[x + 11], 2);
            convert2x2RgbaToYcbr411<rgbFormat, evenLinePadding>(dstm, 6, j, n, rgbLine0[x + 12], rgbLine0[x + 13], rgbLine1[x + 12], rgbLine1[x + 13], 1);
            convert2x2RgbaToYcbr411<rgbFormat, evenLinePadding>(dstm, 7, j, n, rgbLine0[x + 14], rgbLine0[x + 15], rgbLine1[x + 14], rgbLine1[x + 15], 2);
          }
          else
          {
            for (int i = 0, bias = 1; i < Dct::BlockSize; i++, bias ^= 3)
              convert2x2RgbaToYcbr411<rgbFormat, evenLinePadding>(dstm, i, j, n, rgbLine0[x + i * 2], rgbLine0[x + i * 2 + 1], rgbLine1[x + i * 2], rgbLine1[x + i * 2 + 1], bias);
          }
        }
#else
        for (int i = 0, bias = 1; i < Dct::BlockSize; i++, bias ^= 3)
        {
          for (int n = 0, x = 0; n < SimdLength; n++, x += 2 * Dct::BlockSize)
            convert2x2RgbaToYcbr411<rgbFormat, evenLinePadding>(dstm, i, j, n, rgbLine0[x + i * 2], rgbLine0[x + i * 2 + 1], rgbLine1[x + i * 2], rgbLine1[x + i * 2 + 1], bias);
        }
#endif
      }
#endif
    }
  }; // struct RgbaScanlinePairToYcbr411
}

template<int SimdLength, bool aligned>
void copyRgbaScanline(int32_t* dst, const int32_t* src, int simdCount)
{
  typedef Platform::Cpu::SIMD<int32_t, SimdLength> SimdHelper;
  for (int i = 0; i < simdCount; i++)
    SimdHelper::store(dst + i * SimdLength, SimdHelper::template load<aligned>(src + i * SimdLength));
}

template<int SimdLength>
void loadNonWrappedRgbaSimdLinesPair(int32_t* pair0, int32_t* pair1, const uint8_t* rgb, int j, bool evenLinePadding,
  const ImageMetaData& imageMetaData, const EncoderBuffer::McuIndex* mcu, int count)
{
  int x0 = mcu[0].m_x * 2 * Dct::BlockSize;
  int y0 = mcu[0].m_y * 2 * Dct::BlockSize;
  int yoffset0 = ((y0 + j * 2 + 0 < imageMetaData.m_size.m_height) ? y0 + j * 2 + 0 : imageMetaData.m_size.m_height - (evenLinePadding ? 2 : 1)) * imageMetaData.m_scanlineBytes;
  int yoffset1 = ((y0 + j * 2 + 1 < imageMetaData.m_size.m_height) ? y0 + j * 2 + 1 : imageMetaData.m_size.m_height - 1) * imageMetaData.m_scanlineBytes;
  const int32_t* rgbLine0 = (const int32_t*)(rgb + yoffset0) + x0;
  const int32_t* rgbLine1 = (const int32_t*)(rgb + yoffset1) + x0;
  int simdCount = 2 * Dct::BlockSize / SimdLength * count;

  if (Platform::Cpu::SIMD<int32_t, SimdLength>::isPointerAligned(rgbLine0))
    copyRgbaScanline<SimdLength, true>(pair0, rgbLine0, simdCount);
  else
    copyRgbaScanline<SimdLength, false>(pair0, rgbLine0, simdCount);

  if (Platform::Cpu::SIMD<int32_t, SimdLength>::isPointerAligned(rgbLine1))
    copyRgbaScanline<SimdLength, true>(pair1, rgbLine1, simdCount);
  else
    copyRgbaScanline<SimdLength, false>(pair1, rgbLine1, simdCount);
}

static inline void loadWrappedRgbaSimdLinesPair(int32_t* pair0, int32_t* pair1, const uint8_t* rgb, int j, bool evenLinePadding,
  const int (*xoffsets)[Dct::BlockSize * 2], const ImageMetaData& imageMetaData, const EncoderBuffer::McuIndex* mcu, int count)
{
  for (int n = 0; n < count; n++)
  {
    int y0 = mcu[n].m_y * 2 * Dct::BlockSize;
    int yoffset0 = ((y0 + j * 2 + 0 < imageMetaData.m_size.m_height) ? y0 + j * 2 + 0 : imageMetaData.m_size.m_height - (evenLinePadding ? 2 : 1)) * imageMetaData.m_scanlineBytes;
    int yoffset1 = ((y0 + j * 2 + 1 < imageMetaData.m_size.m_height) ? y0 + j * 2 + 1 : imageMetaData.m_size.m_height - 1) * imageMetaData.m_scanlineBytes;
    const int32_t* rgbLine0 = (const int32_t*)(rgb + yoffset0);
    const int32_t* rgbLine1 = (const int32_t*)(rgb + yoffset1);
    int offset = n * 2 * Dct::BlockSize;

    for (int i = 0; i < Dct::BlockSize; i++)
    {
      int x0offset = xoffsets[n][i * 2 + 0];
      int x1offset = xoffsets[n][i * 2 + 1];

      pair0[offset + i * 2] = rgbLine0[x0offset];
      pair0[offset + i * 2 + 1] = rgbLine0[x1offset];
      pair1[offset + i * 2] = rgbLine1[x0offset];
      pair1[offset + i * 2 + 1] = rgbLine1[x1offset];
    }
  }
}

template<ImageMetaData::Format rgbFormat, int options, int SimdLength, typename T>
static void rgbaToYcbr411(const ImageMetaData& imageMetaData, const uint8_t* rgb, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, T(*dst)[Dct::BlockSize2][SimdLength])
{
  //  Q_ASSERT(count < SimdLength);

  for (; count > 0;)
  {
    int simdMcuCount = getScanlineSimdMcuCount(mcu, count, bufferMetaData, imageMetaData.m_size.m_width, imageMetaData.m_size.m_height);
    if (simdMcuCount > 0)
    {
      assert(simdMcuCount % SimdLength == 0 && mcu[0].m_y == mcu[simdMcuCount - 1].m_y);
      int nsimd = simdMcuCount / SimdLength;
      int x0 = mcu[0].m_x * 2 * Dct::BlockSize;
      int y0 = mcu[0].m_y * 2 * Dct::BlockSize;

      for (int j = 0; j < Dct::BlockSize; j++)
        RgbaScanlinePairToYcbr411<rgbFormat, options, SimdLength, false, T>::perform(j, (const int32_t*)(rgb + (y0 + j * 2 + 0) * imageMetaData.m_scanlineBytes) + x0, (const int32_t*)(rgb + (y0 + j * 2 + 1) * imageMetaData.m_scanlineBytes) + x0, nsimd, dst);
    }
    mcu += simdMcuCount; count -= simdMcuCount; dst += simdMcuCount / SimdLength * bufferMetaData.m_mcuBlockCount;

    if (!count)
      break;

    int wrapMcuCount = std::min(count, SimdLength);
    if ((mcu[wrapMcuCount - 1].m_y + 1) * 2 * Dct::BlockSize <= imageMetaData.m_size.m_height || mcu[0].m_y == mcu[wrapMcuCount - 1].m_y)
    {
      alignas(32) int32_t pair[2][Dct::BlockSize * SimdLength * 2];

      if (mcu[0].m_y <= mcu[wrapMcuCount - 1].m_y + 1)
      {
        int wrappedMcuIndex = 0;
        for (; wrappedMcuIndex < wrapMcuCount - 1; wrappedMcuIndex++)
        {
          if (mcu[wrappedMcuIndex].m_y != mcu[wrappedMcuIndex + 1].m_y)
            break;
        }

        bool isMcuWrapped = (mcu[wrappedMcuIndex].m_x + 1) * 2 * Dct::BlockSize > imageMetaData.m_size.m_width;
        int firstLineMcuCount = wrappedMcuIndex + (isMcuWrapped ? 0 : 1);
        int secondLineMcuCount = wrapMcuCount - wrappedMcuIndex - 1;
        constexpr int perMcuStep = 2 * Dct::BlockSize;

        int xoffsets[Dct::BlockSize * 2];
        if (isMcuWrapped)
        {
          int x0 = mcu[wrappedMcuIndex].m_x * 2 * Dct::BlockSize;
          for (int i = 0; i < Dct::BlockSize * 2; i++)
            xoffsets[i] = (x0 + i < imageMetaData.m_size.m_width) ? x0 + i : imageMetaData.m_size.m_width - 1;
        }

        for (int j = 0; j < Dct::BlockSize; j++)
        {
          constexpr int loadSimdLength = std::max<int>(1, SimdLength * sizeof(T) / sizeof(int32_t));
          bool evenLinePadding = (imageMetaData.m_size.m_height & 1) == 0 && mcu[0].m_y * 2 * Dct::BlockSize + j * 2 >= imageMetaData.m_size.m_height; // to mimic libjpeg logic
          if (firstLineMcuCount)
            loadNonWrappedRgbaSimdLinesPair<loadSimdLength>(pair[0], pair[1], rgb, j, evenLinePadding, imageMetaData, mcu, firstLineMcuCount);
          if (isMcuWrapped)
            loadWrappedRgbaSimdLinesPair(pair[0] + wrappedMcuIndex * perMcuStep, pair[1] + wrappedMcuIndex * perMcuStep, rgb, j, evenLinePadding, &xoffsets, imageMetaData, mcu + wrappedMcuIndex, 1);
          if (secondLineMcuCount)
            loadNonWrappedRgbaSimdLinesPair<loadSimdLength>(pair[0] + (wrappedMcuIndex + 1) * perMcuStep, pair[1] + (wrappedMcuIndex + 1) * perMcuStep, rgb, j, evenLinePadding, imageMetaData, mcu + wrappedMcuIndex + 1, secondLineMcuCount);

          if (evenLinePadding)
            RgbaScanlinePairToYcbr411<rgbFormat, options, SimdLength, true, T>::perform(j, pair[0], pair[1], 1, dst);
          else
            RgbaScanlinePairToYcbr411<rgbFormat, options, SimdLength, false, T>::perform(j, pair[0], pair[1], 1, dst);
        }
      }
      else
      {
        int xoffsets[SimdLength][Dct::BlockSize * 2];
        for (int n = 0; n < wrapMcuCount; n++)
        {
          int x0 = mcu[n].m_x * 2 * Dct::BlockSize;

          for (int i = 0; i < Dct::BlockSize * 2; i++)
            xoffsets[n][i] = (x0 + i < imageMetaData.m_size.m_width) ? x0 + i : imageMetaData.m_size.m_width - 1;
        }

        if (wrapMcuCount < SimdLength) // just to prevent UB accesing uninitialized memory
        {
          memset(pair[0] + wrapMcuCount * 2 * Dct::BlockSize, 0, (SimdLength - wrapMcuCount) * 2 * Dct::BlockSize * sizeof(pair[0][0]));
          memset(pair[1] + wrapMcuCount * 2 * Dct::BlockSize, 0, (SimdLength - wrapMcuCount) * 2 * Dct::BlockSize * sizeof(pair[0][0]));
        }

        for (int j = 0; j < Dct::BlockSize; j++)
        {
          bool evenLinePadding = (imageMetaData.m_size.m_height & 1) == 0 && mcu[0].m_y * 2 * Dct::BlockSize + j * 2 >= imageMetaData.m_size.m_height; // to mimic libjpeg logic
          loadWrappedRgbaSimdLinesPair(pair[0], pair[1], rgb, j, evenLinePadding, xoffsets, imageMetaData, mcu, wrapMcuCount);

          if (evenLinePadding)
            RgbaScanlinePairToYcbr411<rgbFormat, options, SimdLength, true, T>::perform(j, pair[0], pair[1], 1, dst);
          else
            RgbaScanlinePairToYcbr411<rgbFormat, options, SimdLength, false, T>::perform(j, pair[0], pair[1], 1, dst);
        }
      }
    }
    else
    {
      for (int n = 0; n < wrapMcuCount; n++)
      {
        int xoffsets[Dct::BlockSize * 2];
        int x0 = mcu[n].m_x * 2 * Dct::BlockSize;
        int y0 = mcu[n].m_y * 2 * Dct::BlockSize;

        for (int j = 0; j < Dct::BlockSize * 2; j++)
          xoffsets[j] = (x0 + j < imageMetaData.m_size.m_width) ? x0 + j : imageMetaData.m_size.m_width - 1;

        for (int j = 0; j < Dct::BlockSize; j++)
        {
          bool evenLinePadding = (imageMetaData.m_size.m_height & 1) == 0 && y0 + j * 2 >= imageMetaData.m_size.m_height;
          int yoffset0 = ((y0 + j * 2 + 0 < imageMetaData.m_size.m_height) ? y0 + j * 2 + 0 : imageMetaData.m_size.m_height - (evenLinePadding ? 2 : 1)) * imageMetaData.m_scanlineBytes;
          int yoffset1 = ((y0 + j * 2 + 1 < imageMetaData.m_size.m_height) ? y0 + j * 2 + 1 : imageMetaData.m_size.m_height - 1) * imageMetaData.m_scanlineBytes;
          const uint32_t* rgbLine0 = (const uint32_t*)(rgb + yoffset0);
          const uint32_t* rgbLine1 = (const uint32_t*)(rgb + yoffset1);

          for (int i = 0, bias = 1; i < Dct::BlockSize; i++, bias ^= 3)
          {
            int x0offset = xoffsets[i * 2 + 0];
            int x1offset = xoffsets[i * 2 + 1];

            if (evenLinePadding)
              convert2x2RgbaToYcbr411<rgbFormat, true, SimdLength>(dst, i, j, n, rgbLine0[x0offset], rgbLine0[x1offset], rgbLine1[x0offset], rgbLine1[x1offset], bias);
            else
              convert2x2RgbaToYcbr411<rgbFormat, false, SimdLength>(dst, i, j, n, rgbLine0[x0offset], rgbLine0[x1offset], rgbLine1[x0offset], rgbLine1[x1offset], bias);
          }
        }
      }
    }
    mcu += SimdLength; count -= SimdLength; dst += bufferMetaData.m_mcuBlockCount;
  }
}

template<typename T, int SimdLength>
void rgbaToYcbr411Implementation(const ImageMetaData& imageMetaData, const uint8_t* rgb, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, T* dst, int options);

namespace
{

template<int SimdLength, typename T>
struct rgbaToYcbr411Helper
{
  template<ImageMetaData::Format rgbFormat>
  static void perform(const ImageMetaData& imageMetaData, const uint8_t* rgb, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, T (*dst)[Dct::BlockSize2][SimdLength], int options)
  {
    if (options & AverageInRgbSpace)
      rgbaToYcbr411<rgbFormat, AverageInRgbSpace, SimdLength, T>(imageMetaData, rgb, bufferMetaData, mcu, count, dst);
    else
      rgbaToYcbr411<rgbFormat, 0, SimdLength, T>(imageMetaData, rgb, bufferMetaData, mcu, count, dst);
  }

  static void perform(const ImageMetaData& imageMetaData, const uint8_t* rgb, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, T (*dst)[Dct::BlockSize2][SimdLength], int options)
  {
#ifdef TRANSPOSED_SIMD_BUFFER
    switch (imageMetaData.m_format)
    {
    case ImageMetaData::Rgba32:
      return perform<ImageMetaData::Rgba32>(imageMetaData, rgb, bufferMetaData, mcu, count, dst, options);
    case ImageMetaData::Bgra32:
      return perform<ImageMetaData::Bgra32>(imageMetaData, rgb, bufferMetaData, mcu, count, dst, options);
    case ImageMetaData::Grayscale8:
      assert(false);
    }
#else
#error "not implemented..."
#endif
  }
};

}

//export

template<int SimdLength, typename T>
static inline void exportBlock(int16_t (*dst)[Dct::BlockSize2], const T(*values)[SimdLength], int mcuBlocks, int count)
{
  int nmax = std::min(count, SimdLength);
  for (int k = 0; k < Dct::BlockSize2; k++)
  {
    for (int n = 0; n < nmax; n++)
      dst[n * mcuBlocks][k] = (int16_t)values[k][n];
  }
}

template<int SimdLength>
void exportBlockInt32(int16_t (*dst)[Dct::BlockSize2], const int32_t (*values)[SimdLength], int mcuBlocks, int count)
{
  typedef Platform::Cpu::SIMD<int32_t, SimdLength> SimdHelper;
  typedef typename SimdHelper::Type SimdType;
  typename SimdHelper::NativeType matrix[SimdLength * 2];

  if (count >= SimdLength)
  {
    for (int j = 0; j < 4 * Dct::BlockSize / SimdLength; j++)
    {
      int k0 = 2 * j * SimdLength;
      SimdHelper::template transpose<true, 1, 2>((SimdType*)matrix, values[k0]);
      SimdHelper::template transpose<true, 1, 2>((SimdType*)matrix + SimdLength, values[k0] + SimdLength);

      for (int n = 0; n < SimdLength; n++)
        SimdHelper::store((int32_t*)(dst[n * mcuBlocks] + k0), SimdHelper::interleaveLow16Bit(matrix[n], matrix[SimdLength + n]));
    }
  }
  else
  {
    for (int j = 0; j < 4 * Dct::BlockSize / SimdLength; j++)
    {
      int k0 = 2 * j * SimdLength;
      SimdHelper::template transpose<true, 1, 2>((SimdType*)matrix, values[k0]);
      SimdHelper::template transpose<true, 1, 2>((SimdType*)matrix + SimdLength, values[k0] + SimdLength);

      for (int n = 0; n < count; n++)
        SimdHelper::store((int32_t*)(dst[n * mcuBlocks] + k0), SimdHelper::interleaveLow16Bit(matrix[n], matrix[SimdLength + n]));
    }
  }
}

template<int SimdLength, typename T>
void exportBlocks(const EncoderBuffer::MetaData& metaData, int16_t(*dst)[Dct::BlockSize2], const T (*buffers)[Dct::BlockSize2 * SimdLength], int count)
{
  int mcuBlocks = metaData.m_mcuBlockCount;
  for (; count > 0; count -= SimdLength, buffers += mcuBlocks, dst += SimdLength * mcuBlocks)
  {
    for (size_t c = 0; c < metaData.m_components.size(); c++)
    {
      const EncoderBuffer::MetaData::Component& component = metaData.m_components.at(c);

      for (int m = 0; m < component.m_blockCount; m++)
      {
        int srcOffset = component.m_blockOffset + m;
        int dstOffset = component.m_blockOffset + m;
        int16_t (*dsti)[Dct::BlockSize2] = dst + dstOffset;
/*
        if (SimdLength == Dct::BlockSize)
        {
          const T (*values)[Dct::BlockSize] = (T (*)[Dct::BlockSize])buffers[srcOffset];
          exportBlock<Dct::BlockSize>(dsti, values, mcuBlocks, count);
        }
        else
*/
        {
          const T (*values)[SimdLength] = (T (*)[SimdLength])buffers[srcOffset];
          exportBlock<SimdLength>(dsti, values, mcuBlocks, count);
        }
      }
    }
  }
}

template<typename T, int SimdLength>
void exportBlocksImplementation(const EncoderBuffer::MetaData& metaData, int16_t (*dst)[Dct::BlockSize2], const T* buffers, int count);

#endif // TRANSPOSED_SIMD_BUFFER

}
