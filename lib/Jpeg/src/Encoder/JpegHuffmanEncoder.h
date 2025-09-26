#pragma once

#include <cstdint>

//#define JPEG_HUFFMAN_ENCODER_ZIGZAG_INPUT

namespace Jpeg
{

class EncoderBuffer;
struct HuffmanTable;

class HuffmanEncoder
{
public:
  HuffmanEncoder(const HuffmanTable& dcTable, const HuffmanTable& acTable);

  int64_t encode(const int16_t* block, int prevDc, uint64_t* output, int64_t outputOffset, int64_t outputSizeInItems) const;
  static int64_t encode(const int16_t* blocks, const int* mcuComponents, int mcuBlockCount, 
    const HuffmanEncoder* componentEncoders, const int* componentEncoderIndices, int* componentDc, int mcuCount,
    uint64_t* output, int64_t outputOffset, int64_t outputSizeInItems);
  static int64_t padToByteBoundary(uint64_t* output, int64_t outputOffset);
  static int64_t byteStuffingByteCount(const uint64_t* output, int64_t outputOffset);
  static int64_t byteStuffing(uint64_t* output, int64_t outputOffset, int64_t bytesToAdd);

  static uint64_t* allocBitBuffer(int64_t wordCount);
  static void freeBitBuffer(uint64_t* buffer);

  struct LookupTable
  {
    enum Type
    {
      Dc, Ac
    };

    static constexpr int TableSize = 256;
#if 1
    struct Item
    {
      uint32_t m_code;
      int32_t m_size;
    };
    alignas(32) Item m_items[TableSize] = {};

    void setCode(int index, uint32_t code, int32_t size)
    {
      m_items[index].m_code = code;
      m_items[index].m_size = size;
    }

    uint32_t getCode(int index) const
    {
      return m_items[index].m_code;
    }

    int32_t getSize(int index) const
    {
      return m_items[index].m_size;
    }
#else
    uint32_t m_codeTable[TableSize] = {};
    int8_t m_sizeTable[TableSize] = {};

    void setCode(int index, uint32_t code, int8_t size)
    {
      m_codeTable[index] = code;
      m_sizeTable[index] = size;
    }

    uint32_t getCode(int index) const
    {
      return m_codeTable[index];
    }

    int8_t getSize(int index) const
    {
      return m_sizeTable[index];
    }
#endif
    bool m_isValid = false;

    LookupTable();
    LookupTable(const HuffmanTable& huffmanTable, Type type);
  };

protected:
  template <int SimdLength>
  static int64_t encodeBlocks(const int16_t* block, const int* mcuComponents, int mcuBlockCount,
    const HuffmanEncoder* componentEncoders, const int* componentEncoderIndices, int* componentDc, int mcuCount,
    uint64_t* output, int64_t outputOffset, int64_t outputSizeInItems);

private:
  LookupTable m_dcTable;
  LookupTable m_acTable;

  static int m_simdLength;
  static int m_byteSimdLength;
};

}
