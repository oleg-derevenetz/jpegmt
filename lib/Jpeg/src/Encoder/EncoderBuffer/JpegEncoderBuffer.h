#pragma once

#include <cstdint>
#include <vector>

#include <Jpeg/JpegDCT.h>

#define TRANSPOSED_SIMD_BUFFER

namespace Jpeg
{

struct EncodingOptions;
struct ImageMetaData;
struct Size;

class EncoderBuffer
{
public:
  struct MetaData
  {
    enum ItemType
    {
      Int16,
      Int32,
    };

    enum ComponentType
    {
      InvalidComponentType,
      Y,
      Cb,
      Cr
    };

    struct ComponentInfo
    {
      int m_hBlocks = 1;
      int m_vBlocks = 1;
      ComponentType m_type = InvalidComponentType;

      int hSamplingFactor(int hScale) const;
      int vSamplingFactor(int vScale) const;
    };

    struct Component
    {
      ComponentInfo m_info;
      int m_blockCount = 1;
      int m_blockOffset = 0;
    };

    struct McuIndex
    {
      int m_x = -1;
      int m_y = -1;
    };

    std::vector<Component> m_components;
    std::vector<int> m_mcuComponents;
    ItemType m_itemType = Int16;
    int m_simdLengthLog2 = 0;
    int m_simdLength = 1;
    bool m_isYcbcr411components = false;
    int m_mcuBlockCount = 0;
    int m_hScale = 1;
    int m_vScale = 1;

    MetaData(const std::vector<ComponentInfo>& components);

    bool setSimdLength(int simdLength);

    int getMcuBlockCount() const;
    Size computeImageSizeInMcu(const Size& imageSize) const;
#ifndef TRANSPOSED_SIMD_BUFFER
    int computeImageBufferCount(const Size& imageSize, int blockCount) const;
#else
    int computeImageBufferCount(const Size& imageSize, int simdBlockCount) const;
#endif

    int blockXPos(const McuIndex& mcu, int subBlock) const;
    int blockYPos(const McuIndex& mcu, int subBlock) const;

    Component getComponent(int component) const;
  };

  typedef MetaData::McuIndex McuIndex;

#ifndef TRANSPOSED_SIMD_BUFFER
  static constexpr int SimdLength = 16;
#endif

#ifndef TRANSPOSED_SIMD_BUFFER
  EncoderBuffer(const MetaData& metaData, int blockCount);
#else
  EncoderBuffer(const MetaData& metaData, int simdBlockCount = 1);
#endif
  ~EncoderBuffer();

  bool importImageBlocks(const ImageMetaData& imageMetaData, const uint8_t* pixels, const MetaData::McuIndex* mcu, int count, const EncodingOptions& options);
  void exportQuantizedBlocks(int16_t (*dst)[Dct::BlockSize2], int count) const;

  int computeImageBufferCount(const Size& imageSize) const;
  int getBufferMcuIndices(const Size& imageSizeInMcu, int bufferNumber, McuIndex* mcu) const;

private:
  EncoderBuffer(const EncoderBuffer&) = delete;
  EncoderBuffer& operator=(const EncoderBuffer&) = delete;

private:
  MetaData m_metaData;
#ifndef TRANSPOSED_SIMD_BUFFER
  int m_blockCount = 1;
  int16_t (*m_componentBuffers)[Dct::BlockSize2] = nullptr;
#else
  int m_simdBlockCount = 1;
  void* m_itemBuffer = nullptr;
#endif

  friend class Encoder;
  friend class ForwardDct;
};

}
