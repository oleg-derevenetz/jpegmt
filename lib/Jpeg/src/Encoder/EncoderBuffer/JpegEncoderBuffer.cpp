#include "JpegEncoderBuffer.h"

#include <Helper/Platform/Cpu/intrinsics.h>

//#define DIRECT_YCC_COMPUTE

#include <Jpeg/JpegWriter.h>

#include "JpegEncoderBufferTemplates.h"

namespace Jpeg
{

static int itemTypeMaxSimdLength(const EncodingOptions& options)
{
  switch(options.m_encoderBufferItemType)
  {
  case Int16:
    return options.m_encoderBufferMaxInt16SimdLength;
  case Int32:
    return options.m_encoderBufferMaxInt32SimdLength;
  }

  assert(false);
  return 1;
}

// EncoderBuffer::MetaData

EncoderBuffer::MetaData::MetaData(const std::vector<ComponentInfo>& components, const EncodingOptions& options) :
  m_itemType(options.m_encoderBufferItemType), m_maxSimdLength(itemTypeMaxSimdLength(options))
{
  int blockOffset = 0;
  for (int i = 0; i < (int)components.size(); i++)
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

  setSimdLength(getSimdLength(m_itemType, m_maxSimdLength));
}

int EncoderBuffer::MetaData::getSimdLength(const EncodingOptions& options)
{
  return getSimdLength(options.m_encoderBufferItemType, itemTypeMaxSimdLength(options));
}

int EncoderBuffer::MetaData::getSimdLength(ItemType itemType, int maxSimdLength)
{
  using namespace Platform::Cpu;
  SimdFeatures commonSimdFeatures = SimdFeature::Abs | SimdFeature::MulSign | SimdFeature::InitFromUint8;
  switch (itemType)
  {
  case Int16:
    return detectSimdLength<int16_t>(commonSimdFeatures, maxSimdLength);
    break;
  case Int32:
    return detectSimdLength<int32_t>(commonSimdFeatures | Multiplication, maxSimdLength);
    break;
  }

  assert(false);
  return 1;
}

bool EncoderBuffer::MetaData::setSimdLength(int simdLength)
{
  switch(simdLength)
  {
  case 16:
  case 8:
  case 4:
  case 1:
    m_simdLengthLog2 = Platform::Cpu::mostSignificantSetBit(simdLength);
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

template <> void* EncoderBuffer::allocSimdBuffer<int16_t, 1>(int count)
{
  return malloc(sizeof(int16_t) * Dct::BlockSize2 * count);
}

template <> void EncoderBuffer::releaseSimdBuffer<int16_t, 1>(void* buffer)
{
  return free(buffer);
}

template <> void* EncoderBuffer::allocSimdBuffer<int32_t, 1>(int count)
{
  return malloc(sizeof(int32_t) * Dct::BlockSize2 * count);
}

template <> void EncoderBuffer::releaseSimdBuffer<int32_t, 1>(void* buffer)
{
  return free(buffer);
}

namespace
{

struct SimdBufferAllocator
{
  typedef void* ReturnType;

  static ReturnType makeDefaultValue()
  {
    return nullptr;
  }

  template <typename T, int SimdLength>
  static void* perform(int count)
  {
    return EncoderBuffer::allocSimdBuffer<T, SimdLength>(count * SimdLength);
  }
};

struct SimdBufferDeallocator : public NoReturnValueCallable
{
  template <typename T, int SimdLength>
  static void perform(void* buffer)
  {
    return EncoderBuffer::releaseSimdBuffer<T, SimdLength>(buffer);
  }
};

}

EncoderBuffer::EncoderBuffer(const MetaData& metaData, int simdBlockCount) : m_metaData(metaData), m_simdBlockCount(simdBlockCount)
{
  if (simdBlockCount > 0)
  {
    switch(metaData.m_itemType)
    {
    case MetaData::ItemType::Int16:
      m_itemBuffer = SimdFunctionChooser<SimdBufferAllocator>::perform<int16_t>(metaData.m_simdLength, simdBlockCount * metaData.m_mcuBlockCount);
      break;
    case Int32:
      m_itemBuffer = SimdFunctionChooser<SimdBufferAllocator>::perform<int32_t>(metaData.m_simdLength, simdBlockCount * metaData.m_mcuBlockCount);
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
    case MetaData::ItemType::Int16:
      SimdFunctionChooser<SimdBufferDeallocator>::perform<int16_t>(m_metaData.m_simdLength, m_itemBuffer);
      break;
    case MetaData::ItemType::Int32:
      SimdFunctionChooser<SimdBufferDeallocator>::perform<int32_t>(m_metaData.m_simdLength, m_itemBuffer);
      break;
    }
  }
}
/*
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
*/

#ifndef TRANSPOSED_SIMD_BUFFER // int16x8
static void grayBlockScanlineToYComponent(const uint8_t* pixels, int scanlineLength, const EncoderBuffer::McuIndex* mcu, int count, int16_t* dst)
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
        for (int k = 0; k < Dct::BlockSize; k++)
          dstLine[k] = pixelLine[k] - sampleZeroLevel;
      }
    }
  }
}

static void grayToYComponent(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int16_t* dst)
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
#endif

template<>
void loadRgbSimdLine<1, int32_t>(int32_t* dst, const int32_t (*src)[2][Dct::BlockSize])
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

template <>
void grayToYComponentImplementation<int16_t, 1>(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int16_t* dst)
{
  grayToYComponent<1>(imageMetaData, pixels, bufferMetaData, mcu, count, (int16_t (*)[1])dst);
}

template <>
void grayToYComponentImplementation<int32_t, 1>(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int32_t* dst)
{
  grayToYComponent<1>(imageMetaData, pixels, bufferMetaData, mcu, count, (int32_t (*)[1])dst);
}

template<>
void rgbaToYcbr411Implementation<int16_t, 1>(const ImageMetaData& imageMetaData, const uint8_t* rgb, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int16_t* dst, int options)
{
  rgbaToYcbr411Helper<1, int16_t>::perform(imageMetaData, rgb, bufferMetaData, mcu, count, (int16_t (*)[Dct::BlockSize2][1])dst, options);
}

template<>
void rgbaToYcbr411Implementation<int32_t, 1>(const ImageMetaData& imageMetaData, const uint8_t* rgb, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, int32_t* dst, int options)
{
  rgbaToYcbr411Helper<1, int32_t>::perform(imageMetaData, rgb, bufferMetaData, mcu, count, (int32_t (*)[Dct::BlockSize2][1])dst, options);
}

namespace
{

struct GrayToYComponentCallable : public NoReturnValueCallable
{
  template <typename T, int SimdLength>
  static void perform(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, T* dst)
  {
    return grayToYComponentImplementation<T, SimdLength>(imageMetaData, pixels, bufferMetaData, mcu, count, dst);
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

struct RgbaToYcbr411Callable : public NoReturnValueCallable
{
  template <typename T, int SimdLength>
  static void perform(const ImageMetaData& imageMetaData, const uint8_t* rgb, const EncoderBuffer::MetaData& bufferMetaData, const EncoderBuffer::McuIndex* mcu, int count, T* dst, int options)
  {
    return rgbaToYcbr411Implementation<T, SimdLength>(imageMetaData, rgb, bufferMetaData, mcu, count, dst, options);
  }
};

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
static void padComponentBuffers(const EncoderBuffer::MetaData& metaData, T* componentBuffers, int count)
{
  componentBuffers += (count >> metaData.m_simdLengthLog2) * metaData.m_mcuBlockCount * Dct::BlockSize2 * metaData.m_simdLength;
  for (int n = count & ((1 << metaData.m_simdLengthLog2) - 1); n < metaData.m_simdLength; n++)
  {
    for (int j = 0; j < Dct::BlockSize2; j++)
    {
      for (size_t c = 0; c < metaData.m_components.size(); c++)
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
    case MetaData::ItemType::Int16:
      SimdFunctionChooser<GrayToYComponentCallable>::perform<int16_t>(m_metaData.m_simdLength, imageMetaData, pixels, m_metaData, mcu, count, (int16_t*)m_itemBuffer);
      break;
    case MetaData::ItemType::Int32:
      SimdFunctionChooser<GrayToYComponentCallable>::perform<int32_t>(m_metaData.m_simdLength, imageMetaData, pixels, m_metaData, mcu, count, (int32_t*)m_itemBuffer);
      break;
    }
#endif
    break;
  case ImageMetaData::Rgba32:
  case ImageMetaData::Bgra32:
    if (m_metaData.m_isYcbcr411components)
    {
      switch (m_metaData.m_itemType)
      {
      case MetaData::ItemType::Int16:
        SimdFunctionChooser<RgbaToYcbr411Callable>::perform<int16_t>(m_metaData.m_simdLength, imageMetaData, pixels, m_metaData, mcu, count, (int16_t*)m_itemBuffer, rgbToYccOptions);
        break;
      case MetaData::ItemType::Int32:
        SimdFunctionChooser<RgbaToYcbr411Callable>::perform<int32_t>(m_metaData.m_simdLength, imageMetaData, pixels, m_metaData, mcu, count, (int32_t*)m_itemBuffer, rgbToYccOptions);
        break;
      }
    }
    break;
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
    case MetaData::ItemType::Int16:
      padComponentBuffers(m_metaData, (int16_t*)m_itemBuffer, count);
      break;
    case MetaData::ItemType::Int32:
      padComponentBuffers(m_metaData, (int32_t*)m_itemBuffer, count);
      break;
    }
  }
#endif

  return true;
}

template<>
void exportBlocksImplementation<int16_t, 1>(const EncoderBuffer::MetaData& metaData, int16_t (*dst)[Dct::BlockSize2], const int16_t* buffers, int count)
{
  exportBlocks<1>(metaData, dst, (int16_t (*)[Dct::BlockSize2 * 1])buffers, count);
}

template<>
void exportBlocksImplementation<int32_t, 1>(const EncoderBuffer::MetaData& metaData, int16_t (*dst)[Dct::BlockSize2], const int32_t* buffers, int count)
{
  exportBlocks<1>(metaData, dst, (int32_t (*)[Dct::BlockSize2 * 1])buffers, count);
}

namespace
{

struct ExportBlocksCallable : public NoReturnValueCallable
{
  template <typename T, int SimdLength>
  static void perform(const EncoderBuffer::MetaData& metaData, int16_t (*dst)[Dct::BlockSize2], const T* buffers, int count)
  {
    return exportBlocksImplementation<T, SimdLength>(metaData, dst, buffers, count);
  }
};

}

void EncoderBuffer::exportQuantizedBlocks(int16_t (*dst)[Dct::BlockSize2], int count) const
{
#ifndef TRANSPOSED_SIMD_BUFFER
  memcpy(dst, m_componentBuffers, sizeof(dst[0]) * count);
#else
  switch (m_metaData.m_itemType)
  {
  case MetaData::ItemType::Int16:
    SimdFunctionChooser<ExportBlocksCallable>::perform<int16_t>(m_metaData.m_simdLength, m_metaData, dst, (int16_t*)m_itemBuffer, count);
    break;
  case MetaData::ItemType::Int32:
    SimdFunctionChooser<ExportBlocksCallable>::perform<int32_t>(m_metaData.m_simdLength, m_metaData, dst, (int32_t*)m_itemBuffer, count);
    break;
  }
#endif
}

}
