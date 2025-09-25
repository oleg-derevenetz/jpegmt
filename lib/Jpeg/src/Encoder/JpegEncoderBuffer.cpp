#include "JpegEncoderBuffer.h"

#include <cassert>

//#define DIRECT_YCC_COMPUTE

#include <Helper/Platform/Cpu/simd.h>

#include <Jpeg/JpegImageMetaData.h>
#include <Jpeg/JpegWriter.h>
#include "RgbToYcc.h"

#define AVERAGE_IN_RGB_SPACE

namespace Jpeg
{

constexpr int sampleZeroLevel = 128;

static Rgb8ToYccTable makeRgb8ToYccTable()
{
  Rgb8ToYccTable table;

  for (int i = 0; i < 256; i++)
  {
    table.m_yTable.m_rWeights[i] = Rgb8ToYcc::fixedPointFromFloat(0.29900) * i;
    table.m_yTable.m_gWeights[i] = Rgb8ToYcc::fixedPointFromFloat(0.58700) * i;
    table.m_yTable.m_bWeights[i] = Rgb8ToYcc::fixedPointFromFloat(0.11400) * i + Rgb8ToYcc::fixedPointOneHalf;
    table.m_cbTable.m_rWeights[i] = (-Rgb8ToYcc::fixedPointFromFloat(0.16874)) * i;
    table.m_cbTable.m_gWeights[i] = (-Rgb8ToYcc::fixedPointFromFloat(0.33126)) * i;
    /* We use a rounding fudge-factor of 0.5-epsilon for Cb and Cr.
     * This ensures that the maximum output will round to MAXJSAMPLE
     * not MAXJSAMPLE+1, and thus that we don't have to range-limit.
     */
    table.m_cbTable.m_bWeights[i] = Rgb8ToYcc::fixedPointFromFloat(0.50000) * i + Rgb8ToYcc::cbcr8Offset + Rgb8ToYcc::fixedPointOneHalf - 1;
    /*  B=>Cb and R=>Cr tables are the same
        rgb_ycc_tab[i + R_CR_OFF] = FIX(0.50000) * i  + CBCR_OFFSET + ONE_HALF - 1;
    */
    table.m_crTable.m_rWeights[i] = table.m_cbTable.m_bWeights[i];
    table.m_crTable.m_gWeights[i] = (-Rgb8ToYcc::fixedPointFromFloat(0.41869)) * i;
    table.m_crTable.m_bWeights[i] = (-Rgb8ToYcc::fixedPointFromFloat(0.08131)) * i;
  }

  return table;
}

static Rgb8ToYccTable rgb8ToYcc = makeRgb8ToYccTable();

// EncoderBuffer::MetaData

EncoderBuffer::MetaData::MetaData(const std::vector<ComponentInfo>& components)
{
  int blockOffset = 0;
  for (int i = 0; i < components.size(); i++)
  {
    Component component;

    component.m_info = components.at(i);
    component.m_blockCount = component.m_info.m_hBlocks * component.m_info.m_vBlocks;
    assert(component.m_blockCount > 0 && component.m_blockCount <= 4);
    component.m_blockOffset = blockOffset;
    blockOffset += component.m_blockCount;

    for (int j = 0; j < component.m_blockCount; j++)
      m_mcuComponents.push_back(i);

    if (component.m_info.m_hBlocks > m_hScale)
      m_hScale = component.m_info.m_hBlocks;
    if (component.m_info.m_vBlocks > m_vScale)
      m_vScale = component.m_info.m_vBlocks;

    m_components.push_back(component);
  }

  m_isYcbcr411components = components.size() == 3 &&
    components.at(0).m_type == Y && m_components.at(0).m_info.hSamplingFactor(m_hScale) == 1 && m_components.at(0).m_info.vSamplingFactor(m_vScale) == 1 &&
    components.at(1).m_type == Cb && m_components.at(1).m_info.hSamplingFactor(m_hScale) == 2 && m_components.at(1).m_info.vSamplingFactor(m_vScale) == 2 &&
    components.at(2).m_type == Cr && m_components.at(2).m_info.hSamplingFactor(m_hScale) == 2 && m_components.at(2).m_info.vSamplingFactor(m_vScale) == 2;

  m_mcuBlockCount = blockOffset;

  switch (m_itemType)
  {
  case Int16:
    setSimdLength(Platform::Cpu::SimdDetector<int16_t>::maxSimdLength());
    break;
  case Int32:
    setSimdLength(Platform::Cpu::SimdDetector<int32_t>::maxSimdLength());
    break;
  }
}

bool EncoderBuffer::MetaData::setSimdLength(int simdLength)
{
  switch(simdLength)
  {
  case 16:
    m_simdLengthLog2 = 4;
    break;
  case 8:
    m_simdLengthLog2 = 3;
    break;
  case 1:
    m_simdLengthLog2 = 0;
    break;
  default:
    return false;
  }

  m_simdLength = simdLength;
  return true;
}

int EncoderBuffer::MetaData::getMcuBlockCount() const
{
  return m_mcuBlockCount;
}

Size EncoderBuffer::MetaData::computeImageSizeInMcu(const Size& imageSize) const
{
  return Size{1 + (imageSize.m_width - 1) / (Dct::BlockSize * m_hScale), 1 + (imageSize.m_height - 1) / (Dct::BlockSize * m_vScale)};
}

int EncoderBuffer::MetaData::blockXPos(const McuIndex& mcu, int subBlock) const
{
  return (mcu.m_x * m_hScale + subBlock % 2) * Dct::BlockSize;
}

int EncoderBuffer::MetaData::blockYPos(const McuIndex& mcu, int subBlock) const
{
  return (mcu.m_y * m_vScale + subBlock / 2) * Dct::BlockSize;
}

EncoderBuffer::MetaData::Component EncoderBuffer::MetaData::getComponent(int component) const
{
  return m_components.at(component);
}

int EncoderBuffer::MetaData::ComponentInfo::hSamplingFactor(int hScale) const
{
  return hScale / m_hBlocks;
}

int EncoderBuffer::MetaData::ComponentInfo::vSamplingFactor(int vScale) const
{
  return vScale / m_vBlocks;
}

#ifndef TRANSPOSED_SIMD_BUFFER
int EncoderBuffer::MetaData::computeImageBufferCount(const Size& imageSize, int blockCount) const
{
  Size mcu = computeImageSizeInMcu(imageSize);

  return 1 + (mcu.m_width * mcu.m_height - 1) / blockCount;
}
#else
int EncoderBuffer::MetaData::computeImageBufferCount(const Size& imageSize, int simdBlockCount) const
{
  Size mcu = computeImageSizeInMcu(imageSize);

  return 1 + (mcu.m_width * mcu.m_height - 1) / (m_simdLength * simdBlockCount);
}
#endif

// EncoderBuffer

#ifndef TRANSPOSED_SIMD_BUFFER
EncoderBuffer::EncoderBuffer(const MetaData& metaData, int blockCount) : m_metaData(metaData), m_blockCount(blockCount)
{
  if (blockCount > 0)
    m_componentBuffers = Platform::Cpu::SIMD<int16_t, SimdLength>::allocMemory<int16_t[Dct::BlockSize2]>(blockCount * metaData.m_mcuBlockCount);
}
#else
template <typename T>
void* allocSimdBuffer(int simdLength, int count)
{
  switch (simdLength)
  {
  case 16:
    return Platform::Cpu::SIMD<T, 16>::template allocMemory<T[Dct::BlockSize2 * 16]>(count);
  case 8:
    return Platform::Cpu::SIMD<T, 8>::template allocMemory<T[Dct::BlockSize2 * 8]>(count);
  case 4:
    return Platform::Cpu::SIMD<T, 4>::template allocMemory<T[Dct::BlockSize2 * 4]>(count);
  case 1:
    return Platform::Cpu::SIMD<T, 1>::template allocMemory<T[Dct::BlockSize2]>(count);
  }

  return nullptr;
}

template <typename T>
void releaseSimdBuffer(int simdLength, void* buffer)
{
  switch (simdLength)
  {
  case 16:
    return Platform::Cpu::SIMD<T, 16>::freeMemory(buffer);
  case 8:
    return Platform::Cpu::SIMD<T, 8>::freeMemory(buffer);
  case 4:
    return Platform::Cpu::SIMD<T, 4>::freeMemory(buffer);
  case 1:
    return Platform::Cpu::SIMD<T, 1>::freeMemory(buffer);
  }
}

EncoderBuffer::EncoderBuffer(const MetaData& metaData, int simdBlockCount) : m_metaData(metaData), m_simdBlockCount(simdBlockCount)
{
  if (simdBlockCount > 0)
  {
    switch(metaData.m_itemType)
    {
    case MetaData::Int16:
      m_itemBuffer = allocSimdBuffer<int16_t>(metaData.m_simdLength, simdBlockCount * metaData.m_mcuBlockCount);
      break;
    case MetaData::Int32:
      m_itemBuffer = allocSimdBuffer<int32_t>(metaData.m_simdLength, simdBlockCount * metaData.m_mcuBlockCount);
      break;
    }
  }
}
#endif

EncoderBuffer::~EncoderBuffer()
{
  if (m_itemBuffer)
  {
    switch (m_metaData.m_itemType)
    {
    case MetaData::Int16:
      releaseSimdBuffer<int16_t>(m_metaData.m_simdLength, m_itemBuffer);
      break;
    case MetaData::Int32:
      releaseSimdBuffer<int32_t>(m_metaData.m_simdLength, m_itemBuffer);
      break;
    }
  }
}

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

template<ImageMetaData::Format rgbFormat, typename T>
static void rgbaToYccComponent1x1(const uint8_t* rgb, int width, int height, int scanlineLength, int x0, int y0, T* component, int outStride, const Rgb8ToYccTable::Weights& weights)
{
  int xoffsets[Dct::BlockSize];
  for (int i = 0; i < Dct::BlockSize; i++)
    xoffsets[i] = x0 + i < width ? x0 + i : width - 1;

  for(int j = 0; j < Dct::BlockSize; j++)
  {
    int y = y0 + j < height ? y0 + j : height - 1;
    const uint32_t* rgbLine = (const uint32_t*)(rgb + y * scanlineLength);
    T* outLine = component + j * Dct::BlockSize * outStride;

    for(int i = 0; i < Dct::BlockSize; i++)
    {
      uint32_t rgb = rgbLine[xoffsets[i]];
      int32_t componentValue = weights.rgbToComponentValue<rgbFormat>(rgb);

      outLine[i * outStride] = componentValue - sampleZeroLevel;
    }
  }
}

template<ImageMetaData::Format rgbFormat, typename T>
static void rgbaToYccComponent2x1(const uint8_t* rgb, int width, int height, int scanlineLength, int x0, int y0, T* component, int outStride, const Rgb8ToYccTable::Weights& weights)
{
  int xoffsets[Dct::BlockSize*2];
  for (int i = 0; i < Dct::BlockSize*2; i++)
    xoffsets[i] = x0 + i < width ? x0 + i : width - 1;

  for (int j = 0; j < Dct::BlockSize; j++)
  {
    int y = y0 + j < height ? y0 + j : height - 1;
    const uint32_t* rgbLine = (const uint32_t*)(rgb + y * scanlineLength);
    T* outLine = component + j * Dct::BlockSize * outStride;

    for (int i = 0, bias = 0; i < Dct::BlockSize; i++, bias ^= 1)
    {
      int32_t componentValue0 = weights.rgbToComponentValue<rgbFormat>(rgbLine[xoffsets[2*i]]);
      int32_t componentValue1 = weights.rgbToComponentValue<rgbFormat>(rgbLine[xoffsets[2*i + 1]]);

      outLine[i * outStride] = ((componentValue0 + componentValue1 + bias) >> 1) - sampleZeroLevel;
    }
  }
}

template<ImageMetaData::Format rgbFormat, typename T>
static void rgbaToYccComponent2x2(const uint8_t* rgb, int width, int height, int scanlineLength, int x0, int y0, T* component, int outStride, const Rgb8ToYccTable::Weights& weights)
{
  int xoffsets[Dct::BlockSize * 2];
  for (int i = 0; i < Dct::BlockSize * 2; i++)
    xoffsets[i] = x0 + i < width ? x0 + i : width - 1;

  for (int j = 0; j < Dct::BlockSize; j++)
  {
    int y_0 = y0 + 2 * j < height ? y0 + 2 * j : height - 1;
    int y_1 = y0 + 2 * j + 1 < height ? y0 + 2 * j  + 1: height - 1;
    const uint32_t* rgbLine0 = (const uint32_t*)(rgb + y_0 * scanlineLength);
    const uint32_t* rgbLine1 = (const uint32_t*)(rgb + y_1 * scanlineLength);
    T* outLine = component + j * Dct::BlockSize * outStride;

    for (int i = 0, bias = 1; i < Dct::BlockSize; i++, bias ^= 3)
    {
      int32_t componentValue00 = weights.rgbToComponentValue<rgbFormat>(rgbLine0[xoffsets[2*i]]);
      int32_t componentValue10 = weights.rgbToComponentValue<rgbFormat>(rgbLine1[xoffsets[2*i]]);
      int32_t componentValue01 = weights.rgbToComponentValue<rgbFormat>(rgbLine0[xoffsets[2*i + 1]]);
      int32_t componentValue11 = weights.rgbToComponentValue<rgbFormat>(rgbLine1[xoffsets[2*i + 1]]);

      outLine[i * outStride] = ((componentValue00 + componentValue10 + componentValue01 + componentValue11 + bias) >> 2) - sampleZeroLevel;
    }
  }
}

#ifndef FORCE_INLINE
#ifdef Q_CC_MSVC
#define FORCE_INLINE __forceinline
#else
#define FORCE_INLINE inline
#endif
#endif

template<ImageMetaData::Format rgbFormat, bool evenLinePadding, int SimdLength, typename T>
FORCE_INLINE static void convert2x2RgbaToYcbr411(T (*dst)[Dct::BlockSize2][SimdLength], int i, int j, int n, uint32_t rgb0, uint32_t rgb1, uint32_t rgb2, uint32_t rgb3, int bias)
{
  constexpr int yComponentOffset = 0;
  constexpr int cbComponentOffset = 4;
  constexpr int crComponentOffset = 5;

  // Y component
  int subBlock = (2 * j / Dct::BlockSize) * 2 + 2 * i / Dct::BlockSize;
  T (*ydst)[SimdLength] = dst[yComponentOffset + subBlock];
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

#ifndef TRANSPOSED_SIMD_BUFFER
static void grayBlockScanlineToYComponent(const uint8_t* pixels, int scanlineLength, const EncoderBuffer::McuIndex* mcu, int count, int16_t *dst)
{
  using namespace Platform::Cpu;
  int x0 = mcu[0].m_x * Dct::BlockSize;
  int y0 = mcu[0].m_y * Dct::BlockSize;

  if (SIMD<int16_t, 8>::isSupported())
  {
    typedef SIMD<int16_t, 8> SimdHelper;
    typedef typename SimdHelper::Type SimdType;
    using namespace Platform::Cpu::int16;

    SimdType zeroLevel = SimdHelper::populate(sampleZeroLevel);
    for (int j = 0; j < Dct::BlockSize; j++)
    {
      const uint64_t* pixelLine = (const uint64_t*)(pixels + (y0 + j) * scanlineLength + x0);
      SimdType* dstLine = (SimdType*)(dst + j * Dct::BlockSize);

      for (int i = 0; i < count; i++, pixelLine++, dstLine += Dct::BlockSize)
        *dstLine = SimdHelper::fromPackedUint8(*pixelLine) - zeroLevel;
    }
  }
  else
  {
    for (int j = 0; j < Dct::BlockSize; j++)
    {
      const uint8_t* pixelLine = pixels + (y0 + j) * scanlineLength + x0;
      int16_t* dstLine = dst + j * Dct::BlockSize;

      for (int i = 0; i < count; i++, pixelLine += Dct::BlockSize, dstLine += Dct::BlockSize2)
      {
        for(int k = 0; k < Dct::BlockSize; k++)
          dstLine[k] = pixelLine[k] - sampleZeroLevel;
      }
    }
  }
}
#else
template<int SimdLength, typename T>
static void grayScanlineSimdToYComponent(const uint8_t* pixels, int scanlineLength, const EncoderBuffer::McuIndex* mcu, int count, T (*dst)[SimdLength])
{
  int simdCount = count / SimdLength;
  int x0 = mcu[0].m_x * Dct::BlockSize;
  int y0 = mcu[0].m_y * Dct::BlockSize;

  for (int j = 0; j < Dct::BlockSize; j++)
  {
    const uint8_t* pixelLine = pixels + (y0 + j) * scanlineLength + x0;
    T (*dstLine)[SimdLength] = dst + j * Dct::BlockSize;

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

#if 1
template<typename T>
static void grayScanlineSimdToYComponentSquare(const uint8_t* pixels, int scanlineLength, const EncoderBuffer::McuIndex* mcu, int count, T (*dst)[Dct::BlockSize])
{
  constexpr int SimdLength = Dct::BlockSize;
  typedef Platform::Cpu::SIMD<T, SimdLength> SimdHelper;
  typedef typename SimdHelper::Type SimdType;
  using namespace Platform::Cpu::int32;
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
        SimdHelper::fromPackedUint8(pixelLine[0]) - zeroLevel, SimdHelper::fromPackedUint8(pixelLine[1]) - zeroLevel,
        SimdHelper::fromPackedUint8(pixelLine[2]) - zeroLevel, SimdHelper::fromPackedUint8(pixelLine[3]) - zeroLevel,
        SimdHelper::fromPackedUint8(pixelLine[4]) - zeroLevel, SimdHelper::fromPackedUint8(pixelLine[5]) - zeroLevel,
        SimdHelper::fromPackedUint8(pixelLine[6]) - zeroLevel, SimdHelper::fromPackedUint8(pixelLine[7]) - zeroLevel);

      pixelLine += Dct::BlockSize;
      dstLine += Dct::BlockSize2;
    }
  }
}

template<>
static void grayScanlineSimdToYComponent<Dct::BlockSize, int16_t>(const uint8_t* pixels, int scanlineLength, const EncoderBuffer::McuIndex* mcu, int count, int16_t (*dst)[Dct::BlockSize])
{
  grayScanlineSimdToYComponentSquare(pixels, scanlineLength, mcu, count, dst);
}

template<>
static void grayScanlineSimdToYComponent<Dct::BlockSize, int32_t>(const uint8_t* pixels, int scanlineLength, const EncoderBuffer::McuIndex* mcu, int count, int32_t (*dst)[Dct::BlockSize])
{
  grayScanlineSimdToYComponentSquare(pixels, scanlineLength, mcu, count, dst);
}

template<>
FORCE_INLINE static void grayScanlineSimdToYComponent<Dct::BlockSize*2, int16_t>(const uint8_t* pixels, int scanlineLength, const EncoderBuffer::McuIndex* mcu, int count, int16_t (*dst)[Dct::BlockSize*2])
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
#endif
#endif


static int getScanlineSimdMcuCount(const EncoderBuffer::McuIndex* mcu, int count, const EncoderBuffer::MetaData& bufferMetaData, int imageWidth, int imageHeight)
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

#ifndef TRANSPOSED_SIMD_BUFFER
static void grayToYComponent(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int16_t *dst)
{
  for (; count > 0;)
  {
    int simdMcuCount = getScanlineSimdMcuCount(mcu, count, bufferMetaData, imageMetaData.m_size.m_width, imageMetaData.m_size.m_height);
    if (simdMcuCount > 0)
      grayBlockScanlineToYComponent(pixels, imageMetaData.m_scanlineBytes, mcu, simdMcuCount, dst);
    mcu += simdMcuCount; count -= simdMcuCount; dst += Dct::BlockSize2 * simdMcuCount * bufferMetaData.m_mcuBlockCount;

    if (count > 0)
    {
      grayToYComponent(imageMetaData, pixels, mcu[0].m_x * Dct::BlockSize, mcu[0].m_y * Dct::BlockSize, dst, 1);
      mcu += 1; count -= 1; dst += Dct::BlockSize2 * bufferMetaData.m_mcuBlockCount;
    }
  }
}
#else
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
#endif

template<int SimdLength, typename DstType>
static void loadRgbSimdLine(DstType* dst, const int32_t (*blocks)[2][Dct::BlockSize]);

template<>
static void loadRgbSimdLine<Dct::BlockSize>(Platform::Cpu::int32x8_t* dst, const int32_t (*src)[2][Dct::BlockSize])
{
  constexpr int SimdLength = Dct::BlockSize;
#if 1
  typedef Platform::Cpu::SIMD<int32_t, SimdLength> SimdHelper;
  if (SimdHelper::isPointerAligned(src))
  {
    SimdHelper::template transpose<true, 1, 2>(dst, src[0][0]);
    SimdHelper::template transpose<true, 1, 2>(dst + SimdLength, src[0][1]);
  }
  else
  {
    SimdHelper::template transpose<false, 1, 2>(dst, src[0][0]);
    SimdHelper::template transpose<false, 1, 2>(dst + SimdLength, src[0][1]);
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

template <bool aligned, int block>
void loadRgbSimdBlocks(Platform::Cpu::int32x8_t (*matrix)[Dct::BlockSize][2], const int32_t (*blocks)[2][Dct::BlockSize])
{
  typedef Platform::Cpu::SIMD<int32_t, 8> SimdHelper;
  SimdHelper::transpose<true, 2, 2>(matrix[block][0], blocks[0][block]);
  SimdHelper::transpose<true, 2, 2>(matrix[block][0] + 1, blocks[Dct::BlockSize][block]);
}

template<>
static void loadRgbSimdLine<Dct::BlockSize*2, Platform::Cpu::int32x8_t>(Platform::Cpu::int32x8_t* dst, const int32_t (*src)[2][Dct::BlockSize])
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

template <bool aligned, int block>
void loadRgbSimdBlocks(Platform::Cpu::int32x4_t (*matrix)[Dct::BlockSize][2], const int32_t (*blocks)[2][Dct::BlockSize])
{
  typedef Platform::Cpu::SIMD<int32_t, 4> SimdHelper;
  SimdHelper::transpose<aligned, 2, 4>(matrix[block][0], blocks[0][block]);
  SimdHelper::transpose<aligned, 2, 4>(matrix[block][4], blocks[0][block] + 4);
  SimdHelper::transpose<aligned, 2, 4>(matrix[block][0] + 1, blocks[4][block]);
  SimdHelper::transpose<aligned, 2, 4>(matrix[block][4] + 1, blocks[4][block] + 4);
}

template<>
static void loadRgbSimdLine<Dct::BlockSize, Platform::Cpu::int32x4_t>(Platform::Cpu::int32x4_t* dst, const int32_t (*src)[2][Dct::BlockSize])
{
  Platform::Cpu::int32x4_t (*matrix)[Dct::BlockSize][2] = (Platform::Cpu::int32x4_t (*)[Dct::BlockSize][2])dst;
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
static void loadRgbSimdLine<1, int32_t>(int32_t* dst, const int32_t (*src)[2][Dct::BlockSize])
{
  memcpy(dst, src, sizeof(src[0]));
}

/*
template <int SimdLength>
constexpr int rgbSimdLength()
{
  return SimdLength;
}

template <>
constexpr int rgbSimdLength<16>()
{
  return 8;
}
*/
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

/*
template <ImageMetaData::Format rgbFormat, int SimdLength>
void extractRgbComponents(typename Platform::Cpu::SIMD<int32_t, SimdLength/2>::ParamType rgb0, typename Platform::Cpu::SIMD<int32_t, SimdLength/2>::ParamType rgb1,
  typename RgbComponent<SimdLength>::Type& r8, typename RgbComponent<SimdLength>::Type& g8, typename RgbComponent<SimdLength>::Type& b8)
{
  using namespace Platform::Cpu;
  typedef SIMD<int32_t, SimdLength/2> RgbSimdHelper;
  typename RgbComponent<SimdLength>::Type a8;
  static_assert(rgbFormat == ImageMetaData::Bgra32, "not implemented");

  if (rgbFormat == ImageMetaData::Bgra32)
  {
    if (Platform::Cpu::byteOrder == Platform::Cpu::LittleEndian)
      RgbSimdHelper::extractByteComponents(rgb0, rgb1, b8, g8, r8, a8);
    else
      RgbSimdHelper::extractByteComponents(rgb0, rgb1, a8, r8, g8, b8);
  }
}

template <ImageMetaData::Format rgbFormat, int RgbSimdLength, typename T, int SimdLength>
void extractRgbComponents(typename Platform::Cpu::SIMD<int32_t, RgbSimdLength>::ParamType rgb0, typename Platform::Cpu::SIMD<int32_t, RgbSimdLength>::ParamType rgb1,
  typename Platform::Cpu::SIMD<T, SimdLength>::Type& r, typename Platform::Cpu::SIMD<T, SimdLength>::Type& g, typename Platform::Cpu::SIMD<T, SimdLength>::Type& b)
{
  typedef typename Platform::Cpu::SIMD<T, SimdLength>::Type SimdType;
  typename RgbComponent<SimdLength>::Type r8, g8, b8;

  extractRgbComponents<rgbFormat, SimdLength>(rgb0, rgb1, r8, g8, b8);

  r.setFromPackedUint8(r8);
  g.setFromPackedUint8(g8);
  b.setFromPackedUint8(b8);
}

template <ImageMetaData::Format rgbFormat, int RgbSimdLength, typename T, int SimdLength>
void extractRgbComponents(const typename Platform::Cpu::SIMD<int32_t, RgbSimdLength>::NativeType* rgb,
  typename Platform::Cpu::SIMD<T, SimdLength>::Type& r, typename Platform::Cpu::SIMD<T, SimdLength>::Type& g, typename Platform::Cpu::SIMD<T, SimdLength>::Type& b)
{
  extractRgbComponents<rgbFormat, RgbSimdLength, T, SimdLength>(rgb[0], rgb[1], r, g, b);
}
*/
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
    typedef typename SIMD<int16_t, SimdLength>::Type SimdType;
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
    typedef typename SIMD<int16_t, SimdLength>::Type SimdType;
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

template <ImageMetaData::Format rgbFormat, typename T>
struct NoSimdRgbComponentExtractor
{
  static void extract(const int32_t* rgb, T& r, T& g, T& b)
  {
    r = Rgb<rgbFormat>::red(rgb[0]);
    g = Rgb<rgbFormat>::green(rgb[0]);
    b = Rgb<rgbFormat>::blue(rgb[0]);
  }
};

template <ImageMetaData::Format rgbFormat>
struct RgbComponentExtractor<rgbFormat, int16_t, 1> : public NoSimdRgbComponentExtractor<rgbFormat, int16_t>
{
};

template <ImageMetaData::Format rgbFormat>
struct RgbComponentExtractor<rgbFormat, int32_t, 1> : public NoSimdRgbComponentExtractor<rgbFormat, int32_t>
{
};

//#define PER_COMPONENT_TRANSPOSING

#ifdef PER_COMPONENT_TRANSPOSING
template<ImageMetaData::Format rgbFormat, bool aligned>
void loadRgbBlock(Platform::Cpu::uint8x16_t (*r)[2], Platform::Cpu::uint8x16_t (*g)[2], Platform::Cpu::uint8x16_t (*b)[2], const int32_t (*src)[2][Dct::BlockSize])
{
  using namespace Platform::Cpu;

  for (int i = 0; i < Dct::BlockSize; i++)
  {
    extractRgbComponents<rgbFormat, Dct::BlockSize * 2>(int32x8_t::load<aligned>(src[i][0]), int32x8_t::load<aligned>(src[i + Dct::BlockSize][0]), r[i][0], g[i][0], b[i][0]);
    extractRgbComponents<rgbFormat, Dct::BlockSize * 2>(int32x8_t::load<aligned>(src[i][1]), int32x8_t::load<aligned>(src[i + Dct::BlockSize][1]), r[i][1], g[i][1], b[i][1]);
  }
}

template<ImageMetaData::Format rgbFormat>
static void loadRgbSimdLine(Platform::Cpu::int8x32_t* r, Platform::Cpu::int8x32_t* g, Platform::Cpu::int8x32_t* b, const int32_t (*src)[2][Dct::BlockSize])
{
  using namespace Platform::Cpu;
  typedef SIMD<int32_t, Dct::BlockSize> SimdHelper;
  int8x32_t r8[Dct::BlockSize], g8[Dct::BlockSize], b8[Dct::BlockSize];

  if (SimdHelper::isPointerAligned(src))
    loadRgbBlock<rgbFormat, true>((uint8x16_t (*)[2])r8, (uint8x16_t (*)[2])g8, (uint8x16_t (*)[2])b8, src);
  else
    loadRgbBlock<rgbFormat, false>((uint8x16_t (*)[2])r8, (uint8x16_t (*)[2])g8, (uint8x16_t (*)[2])b8, src);

  SIMD<int8_t, 32>::transpose4x8x8(r, r8[0], r8[1], r8[2], r8[3], r8[4], r8[5], r8[6], r8[7]);
  SIMD<int8_t, 32>::transpose4x8x8(g, g8[0], g8[1], g8[2], g8[3], g8[4], g8[5], g8[6], g8[7]);
  SIMD<int8_t, 32>::transpose4x8x8(b, b8[0], b8[1], b8[2], b8[3], b8[4], b8[5], b8[6], b8[7]);
}
#endif

template<int SimdLength, typename T>
FORCE_INLINE void rgbToYcbr(typename Platform::Cpu::SIMD<T, SimdLength>::Type r, typename Platform::Cpu::SIMD<T, SimdLength>::Type g, typename Platform::Cpu::SIMD<T, SimdLength>::Type b,
  typename Platform::Cpu::SIMD<T, SimdLength>::Type& y, typename Platform::Cpu::SIMD<T, SimdLength>::Type& cb, typename Platform::Cpu::SIMD<T, SimdLength>::Type& cr)
{
  y  = RgbToYcc<T, SimdLength, 0>::y(r, g, b);
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
      const int32_t (*rgb0m)[2][Dct::BlockSize] = ((int32_t (*)[2][Dct::BlockSize])rgb0) + SimdLength * m;
      const int32_t (*rgb1m)[2][Dct::BlockSize] = ((int32_t (*)[2][Dct::BlockSize])rgb1) + SimdLength * m;
#ifndef PER_COMPONENT_TRANSPOSING
      typename RgbSimdHelper::NativeType matrix0[Dct::BlockSize * 2][SimdLength/RgbSimdLength], matrix1[Dct::BlockSize * 2][SimdLength / RgbSimdLength];

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
//        for (int n = 0; n < SimdLength; n++)
//          convert2x2RgbaToYcbr411<rgbFormat>(dstm, i, j, n, ((uint32_t (*)[SimdLength])matrix0)[i*2][n], ((uint32_t(*)[SimdLength])matrix0)[i*2 + 1][n], ((uint32_t(*)[SimdLength])matrix1)[i*2][n], ((uint32_t(*)[SimdLength])matrix1)[i*2 + 1][n], bias);

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
      T (*dstm)[Dct::BlockSize2][SimdLength] = dst + m * componentBlockCount;

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

} // namespace

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

void loadWrappedRgbaSimdLinesPair(int32_t* pair0, int32_t* pair1, const uint8_t* rgb, int j, bool evenLinePadding,
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
static void rgbaToYcbr411(const ImageMetaData& imageMetaData, const uint8_t* rgb, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, T (*dst)[Dct::BlockSize2][SimdLength])
{
//  Q_ASSERT(count < SimdLength);

  for (;count > 0;)
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
    if ((mcu[wrapMcuCount - 1].m_y  + 1) * 2 * Dct::BlockSize <= imageMetaData.m_size.m_height || mcu[0].m_y == mcu[wrapMcuCount - 1].m_y)
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

template<ImageMetaData::Format rgbFormat, int SimdLength, typename T>
static void rgbaToYcbr411(const ImageMetaData& imageMetaData, const uint8_t* rgb, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, T (*dst)[Dct::BlockSize2][SimdLength], int options)
{
  if (options & AverageInRgbSpace)
    rgbaToYcbr411<rgbFormat, AverageInRgbSpace, SimdLength, T>(imageMetaData, rgb, bufferMetaData, mcu, count, dst);
  else
    rgbaToYcbr411<rgbFormat, 0, SimdLength, T>(imageMetaData, rgb, bufferMetaData, mcu, count, dst);
}

static void grayToYComponent(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int16_t* dst)
{
  switch (bufferMetaData.m_simdLength)
  {
  case 16:
    return grayToYComponent<16>(imageMetaData, pixels, bufferMetaData, mcu, count, (int16_t (*)[16])dst);
  case 8:
    return grayToYComponent<8>(imageMetaData, pixels, bufferMetaData, mcu, count, (int16_t(*)[8])dst);
  case 1:
    return grayToYComponent<1>(imageMetaData, pixels, bufferMetaData, mcu, count, (int16_t(*)[1])dst);
  }
}

static void grayToYComponent(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int32_t* dst)
{
  switch (bufferMetaData.m_simdLength)
  {
  case 8:
    return grayToYComponent<8>(imageMetaData, pixels, bufferMetaData, mcu, count, (int32_t(*)[8])dst);
  case 4:
    return grayToYComponent<4>(imageMetaData, pixels, bufferMetaData, mcu, count, (int32_t(*)[4])dst);
  case 1:
    return grayToYComponent<1>(imageMetaData, pixels, bufferMetaData, mcu, count, (int32_t(*)[1])dst);
  }
}

template<ImageMetaData::Format rgbFormat>
static void rgbaToYcbr411(const ImageMetaData& imageMetaData, const uint8_t* rgb, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int16_t *dst, int simdLength, int options)
{
  switch(simdLength)
  {
  case 16:
    return rgbaToYcbr411<rgbFormat, 16, int16_t>(imageMetaData, rgb, bufferMetaData, mcu, count, (int16_t (*)[Dct::BlockSize2][16])dst, options);
  case 8:
    return rgbaToYcbr411<rgbFormat, 8, int16_t>(imageMetaData, rgb, bufferMetaData, mcu, count, (int16_t (*)[Dct::BlockSize2][8])dst, options);
  case 1:
    return rgbaToYcbr411<rgbFormat, 1, int16_t>(imageMetaData, rgb, bufferMetaData, mcu, count, (int16_t (*)[Dct::BlockSize2][1])dst, options);
  }

  assert(false);
}

template<ImageMetaData::Format rgbFormat>
static void rgbaToYcbr411(const ImageMetaData& imageMetaData, const uint8_t* rgb, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int32_t *dst, int simdLength, int options)
{
  switch(simdLength)
  {
  case 8:
    return rgbaToYcbr411<rgbFormat, 8, int32_t>(imageMetaData, rgb, bufferMetaData, mcu, count, (int32_t (*)[Dct::BlockSize2][8])dst, options);
//  case 4:
//    return rgbaToYcbr411<rgbFormat, 4, int32_t>(imageMetaData, rgb, bufferMetaData, mcu, count, (int32_t (*)[Dct::BlockSize2][4])dst, options);
  case 1:
    return rgbaToYcbr411<rgbFormat, 1, int32_t>(imageMetaData, rgb, bufferMetaData, mcu, count, (int32_t (*)[Dct::BlockSize2][1])dst, options);
  }

  assert(false);
}

int EncoderBuffer::computeImageBufferCount(const Size& imageSize) const
{
#ifndef TRANSPOSED_SIMD_BUFFER
  return m_metaData.computeImageBufferCount(imageSize, m_blockCount);
#else
  return m_metaData.computeImageBufferCount(imageSize, m_simdBlockCount);
#endif
}

int EncoderBuffer::getBufferMcuIndices(const Size& imageSizeInMcu, int bufferNumber, McuIndex* mcu) const
{
  int xMcu = imageSizeInMcu.m_width;
  int yMcu = imageSizeInMcu.m_height;
#ifndef TRANSPOSED_SIMD_BUFFER
  int bufferMcuCount = m_blockCount;
#else
  int bufferMcuCount = m_metaData.m_simdLength * m_simdBlockCount;
#endif
  int y = bufferNumber * bufferMcuCount / xMcu;
  int x = bufferNumber * bufferMcuCount % xMcu;

  assert(y < yMcu);
  for(int i = 0; i < bufferMcuCount; i++)
  {
    if (mcu)
    {
      mcu[i].m_x = x;
      mcu[i].m_y = y;
    }

    if (++x >= xMcu)
    {
      x = 0;
      if (++y >= yMcu)
        return i + 1;
    }
  }

  return bufferMcuCount;
}

template <typename T>
void padComponentBuffers(const EncoderBuffer::MetaData& metaData, T* componentBuffers, int count)
{
  componentBuffers += (count >> metaData.m_simdLengthLog2) * metaData.m_mcuBlockCount * Dct::BlockSize2 * metaData.m_simdLength;
  for (int n = count & ((1 << metaData.m_simdLengthLog2) - 1); n < metaData.m_simdLength; n++)
  {
    for (int j = 0; j < Dct::BlockSize2; j++)
    {
      for (int c = 0; c < metaData.m_components.size(); c++)
        componentBuffers[(c * Dct::BlockSize2 + j) * metaData.m_simdLength + n] = 0;
    }
  }
}

bool EncoderBuffer::importImageBlocks(const ImageMetaData& imageMetaData, const uint8_t* pixels, const McuIndex* mcu, int count, const EncodingOptions& options)
{
//  int w = imageMetaData.m_size.m_width;
//  int h = imageMetaData.m_size.m_height;
//  int xlen = imageMetaData.m_scanlineBytes;

  int rgbToYccOptions = options.m_averageInRgbSpace ? AverageInRgbSpace : 0;
  switch(imageMetaData.m_format)
  {
  case ImageMetaData::Grayscale8:
    assert(m_metaData.m_components.size() == 1 && m_metaData.m_components.at(0).m_info.m_type == MetaData::Y);
    assert(m_metaData.m_components.at(0).m_info.m_hBlocks == 1 && m_metaData.m_components.at(0).m_info.m_vBlocks == 1);
    if (m_metaData.m_components.size() != 1 || m_metaData.m_components.at(0).m_info.m_type != MetaData::Y)
      return false;

#ifndef TRANSPOSED_SIMD_BUFFER
    grayToYComponent(imageMetaData, pixels, m_metaData, mcu, count, dst[0]);
#else
    switch (m_metaData.m_itemType)
    {
    case MetaData::Int16:
      grayToYComponent(imageMetaData, pixels, m_metaData, mcu, count, (int16_t*)m_itemBuffer);
      break;
    case MetaData::Int32:
      grayToYComponent(imageMetaData, pixels, m_metaData, mcu, count, (int32_t*)m_itemBuffer);
      break;
    }
#endif
    break;
  case ImageMetaData::Rgba32:
#ifdef TRANSPOSED_SIMD_BUFFER
    if (m_metaData.m_isYcbcr411components)
    {
      switch (m_metaData.m_itemType)
      {
      case MetaData::Int16:
        rgbaToYcbr411<ImageMetaData::Rgba32>(imageMetaData, pixels, m_metaData, mcu, count, (int16_t*)m_itemBuffer, m_metaData.m_simdLength, rgbToYccOptions);
        break;
      case MetaData::Int32:
        rgbaToYcbr411<ImageMetaData::Rgba32>(imageMetaData, pixels, m_metaData, mcu, count, (int32_t*)m_itemBuffer, m_metaData.m_simdLength, rgbToYccOptions);
        break;
      }
    }
    break;
#else
#error "not implemented..."
#endif
  case ImageMetaData::Bgra32:
#ifdef TRANSPOSED_SIMD_BUFFER
    if (m_metaData.m_isYcbcr411components)
    {
      switch(m_metaData.m_itemType)
      {
      case MetaData::Int16:
        rgbaToYcbr411<ImageMetaData::Bgra32>(imageMetaData, pixels, m_metaData, mcu, count, (int16_t*)m_itemBuffer, m_metaData.m_simdLength, rgbToYccOptions);
        break;
      case MetaData::Int32:
        rgbaToYcbr411<ImageMetaData::Bgra32>(imageMetaData, pixels, m_metaData, mcu, count, (int32_t*)m_itemBuffer, m_metaData.m_simdLength, rgbToYccOptions);
        break;
      }
    }
    break;
#else
#error "not implemented..."
#endif
/*
    for (int i = 0; i < m_metaData.m_components.size(); i++)
    {
      const MetaData::Component& component = m_metaData.m_components.at(i);
      const Rgb8ToYcc::Weights* weights = nullptr;
      switch(component.m_info.m_type)
      {
      case MetaData::Y:
        weights = &rgb8ToYcc.m_yTable;
        break;
      case MetaData::Cb:
        weights = &rgb8ToYcc.m_cbTable;
        break;
      case MetaData::Cr:
        weights = &rgb8ToYcc.m_crTable;
        break;
      default:
        break;
      }
      Q_ASSERT(weights);
      if (!weights)
        return false;

#ifndef TRANSPOSED_SIMD_BUFFER
      constexpr int outStride = 1;
#else
      int outStride = m_metaData.m_simdLength;
#endif
      const MetaData::ComponentInfo& info = component.m_info;
      int nblocks = info.m_hBlocks * info.m_vBlocks;
      if (info.hSamplingFactor(m_metaData.m_hScale) == 1 && info.vSamplingFactor(m_metaData.m_vScale) == 1)
      {
        for (int j = 0; j < count; j++)
        {
          for (int k = 0; k < nblocks; k++)
            rgbaToYccComponent1x1<ImageMetaData::Bgra32>(pixels, w, h, xlen, m_metaData.blockXPos(mcu[j], k), m_metaData.blockYPos(mcu[j], k), m_componentBuffers[component.m_blockOffset + k] + j, outStride, *weights);
        }
      }
      else if (info.hSamplingFactor(m_metaData.m_hScale) == 2 && info.vSamplingFactor(m_metaData.m_vScale) == 1)
      {
        for (int j = 0; j < count; j++)
        {
          for (int k = 0; k < nblocks; k++)
            rgbaToYccComponent2x1<ImageMetaData::Bgra32>(pixels, w, h, xlen, m_metaData.blockXPos(mcu[j], k), m_metaData.blockYPos(mcu[j], k), m_componentBuffers[component.m_blockOffset + k] + j, outStride, *weights);
        }
      }
      else if (info.hSamplingFactor(m_metaData.m_hScale) == 2 && info.vSamplingFactor(m_metaData.m_vScale) == 2)
      {
        for (int j = 0; j < count; j++)
        {
          for (int k = 0; k < nblocks; k++)
            rgbaToYccComponent2x2<ImageMetaData::Bgra32>(pixels, w, h, xlen, m_metaData.blockXPos(mcu[j], k), m_metaData.blockYPos(mcu[j], k), m_componentBuffers[component.m_blockOffset + k] + j, outStride, *weights);
        }
      }
      else
      {
        Q_ASSERT(false);
        return false;
      }
    }
*/
/*
    if (m_isYcbcr411components)
    {
      for(int i = 0; i < 6; i++)
      {
        for(int j = 0; j < Dct::BlockSize2 * SimdLength; j++)
        {
          if (m_componentBuffers[i][j] != ycbcr41ComponentBuffers[i][j])
            __debugbreak();
        }
      }
    }
*/
    break;
  default:
    assert(false);
    return false;
  }

#ifdef TRANSPOSED_SIMD_BUFFER
  assert(count > 0);
  if (count & ((1 << m_metaData.m_simdLengthLog2) - 1))
  {
    switch (m_metaData.m_itemType)
    {
    case MetaData::Int16:
      padComponentBuffers(m_metaData, (int16_t*)m_itemBuffer, count);
      break;
    case MetaData::Int32:
      padComponentBuffers(m_metaData, (int32_t*)m_itemBuffer, count);
      break;
    }
  }
#endif

  return true;
}

template<int SimdLength, typename T>
void exportBlock(int16_t (*dst)[Dct::BlockSize2], const T (*values)[SimdLength], int mcuBlocks, int count)
{
  int nmax = std::min(count, SimdLength);
  for (int k = 0; k < Dct::BlockSize2; k++)
  {
    for (int n = 0; n < nmax; n++)
      dst[n * mcuBlocks][k] = (int16_t)values[k][n];
  }
}

template<>
void exportBlock<Dct::BlockSize, int16_t>(int16_t (*dst)[Dct::BlockSize2], const int16_t(*values)[Dct::BlockSize], int mcuBlocks, int count)
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
      SimdHelper::template transpose<true>((SimdType*)matrix, values[k0]);

      for (int n = 0; n < SimdLength; n++)
        SimdHelper::store(dst[n * mcuBlocks] + k0, matrix[n]);
    }
  }
  else
  {
    for (int j = 0; j < Dct::BlockSize; j++)
    {
      int k0 = j * Dct::BlockSize;
      SimdHelper::template transpose<true>((SimdType*)matrix, values[k0]);

      for (int n = 0; n < count; n++)
        SimdHelper::store(dst[n * mcuBlocks] + k0, matrix[n]);
    }
  }
}

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
      int lowCount = std::min(Dct::BlockSize, count);

      for (int n = 0; n < lowCount; n++)
        SimdHelper::storeLow(dst[n * mcuBlocks] + k0, matrix[n]);
      for (int n = lowCount; n < count; n++)
        SimdHelper::storeHigh(dst[n * mcuBlocks] + k0, matrix[n - Dct::BlockSize]);
    }
  }
}

template<>
void exportBlock<Dct::BlockSize, int32_t>(int16_t (*dst)[Dct::BlockSize2], const int32_t(*values)[Dct::BlockSize], int mcuBlocks, int count)
{
  constexpr int SimdLength = Dct::BlockSize;
  typedef Platform::Cpu::SIMD<int32_t, SimdLength> SimdHelper;
  typedef typename SimdHelper::Type SimdType;
  typename SimdHelper::NativeType matrix[SimdLength * 2];

  if (count >= SimdLength)
  {
    for (int j = 0; j < Dct::BlockSize / 2; j++)
    {
      int k0 = 2 * j * Dct::BlockSize;
      SimdHelper::template transpose<true, 1, 2>((SimdType*)matrix, values[k0]);
      SimdHelper::template transpose<true, 1, 2>((SimdType*)matrix + SimdLength, values[k0] + SimdLength);

      for (int n = 0; n < SimdLength; n++)
        SimdHelper::store((int32_t*)(dst[n * mcuBlocks] + k0), SimdHelper::interleaveLow16Bit(matrix[n], matrix[SimdLength + n]));
    }
  }
  else
  {
    for (int j = 0; j < Dct::BlockSize / 2; j++)
    {
      int k0 = 2 * j * Dct::BlockSize;
      SimdHelper::template transpose<true, 1, 2>((SimdType*)matrix, values[k0]);
      SimdHelper::template transpose<true, 1, 2>((SimdType*)matrix + SimdLength, values[k0] + SimdLength);

      for (int n = 0; n < count; n++)
        SimdHelper::store((int32_t*)(dst[n * mcuBlocks] + k0), SimdHelper::interleaveLow16Bit(matrix[n], matrix[SimdLength + n]));
    }
  }
}

template<int SimdLength, typename T>
void exportBlocks(const EncoderBuffer::MetaData& metaData, int16_t (*dst)[Dct::BlockSize2], const T (*buffers)[Dct::BlockSize2 * SimdLength], int count)
{
  int mcuBlocks = metaData.m_mcuBlockCount;
  for (; count > 0; count -= SimdLength, buffers += mcuBlocks, dst += SimdLength * mcuBlocks)
  {
    for (int c = 0; c < metaData.m_components.size(); c++)
    {
      const EncoderBuffer::MetaData::Component& component = metaData.m_components.at(c);

      for (int m = 0; m < component.m_blockCount; m++)
      {
        int srcOffset = component.m_blockOffset + m;
        int dstOffset = component.m_blockOffset + m;
        int16_t (*dsti)[Dct::BlockSize2] = dst + dstOffset;

        if (SimdLength == Dct::BlockSize)
        {
          const T (*values)[Dct::BlockSize] = (T (*)[Dct::BlockSize])buffers[srcOffset];
          exportBlock<Dct::BlockSize>(dsti, values, mcuBlocks, count);
        }
        else
        {
          const T (*values)[SimdLength] = (T (*)[SimdLength])buffers[srcOffset];
          exportBlock<SimdLength>(dsti, values, mcuBlocks, count);
        }
      }
    }
  }
}

static void exportBlocks(const EncoderBuffer::MetaData& metaData, int16_t (*dst)[Dct::BlockSize2], const int16_t *buffers, int count)
{
  switch (metaData.m_simdLength)
  {
  case 16:
    return exportBlocks<16>(metaData, dst, (int16_t (*)[Dct::BlockSize2 * 16])buffers, count);
  case 8:
    return exportBlocks<8>(metaData, dst, (int16_t (*)[Dct::BlockSize2 * 8])buffers, count);
  case 1:
    return exportBlocks<1>(metaData, dst, (int16_t (*)[Dct::BlockSize2 * 1])buffers, count);
  }
}

static void exportBlocks(const EncoderBuffer::MetaData& metaData, int16_t (*dst)[Dct::BlockSize2], const int32_t *buffers, int count)
{
  switch (metaData.m_simdLength)
  {
  case 8:
    return exportBlocks<8>(metaData, dst, (int32_t (*)[Dct::BlockSize2 * 8])buffers, count);
  case 4:
    return exportBlocks<4>(metaData, dst, (int32_t (*)[Dct::BlockSize2 * 4])buffers, count);
  case 1:
    return exportBlocks<1>(metaData, dst, (int32_t (*)[Dct::BlockSize2 * 1])buffers, count);
  }
}

void EncoderBuffer::exportQuantizedBlocks(int16_t (*dst)[Dct::BlockSize2], int count) const
{
#ifndef TRANSPOSED_SIMD_BUFFER
  memcpy(dst, m_componentBuffers, sizeof(dst[0]) * count);
#else
  switch (m_metaData.m_itemType)
  {
  case MetaData::Int16:
    exportBlocks(m_metaData, dst, (int16_t*)m_itemBuffer, count);
    break;
  case MetaData::Int32:
    exportBlocks(m_metaData, dst, (int32_t*)m_itemBuffer, count);
    break;
  }
#endif
}

}
