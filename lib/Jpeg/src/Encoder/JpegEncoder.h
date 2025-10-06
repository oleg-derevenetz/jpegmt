#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "EncoderBuffer/JpegEncoderBuffer.h"
#include "HuffmanEncoder/JpegHuffmanEncoder.h"
#include "JpegQuantizer.h"

namespace Jpeg
{

struct QuantizationTable;
struct EncodingOptions;

class Encoder
{
public:
  struct BitBuffer
  {
    int64_t m_bitCount = 0;
    uint64_t* m_bits = nullptr;
    int64_t m_wordsAllocated = 0;
    std::shared_ptr<uint64_t> m_buffer;

    bool reserve(int64_t bitCount);

    static BitBuffer merge(const std::vector<BitBuffer>& buffers, double reserveFraction);
  };
  typedef EncoderBuffer::McuIndex McuIndex;

  Encoder(const std::vector<QuantizationTable>& quantizationTables, const std::vector<int>& componentQuantizationTableIndices,
    const std::vector<HuffmanEncoder>& huffmanEncoders, const std::vector<int>& componentHuffmanEncoderIndices);

  int quantize(EncoderBuffer& encoderBuffer, const ImageMetaData& imageMetaData, const uint8_t* pixels, const McuIndex* mcuIndices, int16_t (*dst)[Dct::BlockSize2], int count, const EncodingOptions& options) const;
  void optimizeDummyBlocks(const EncoderBuffer::MetaData& encoderBufferMetaData, const Size& imageSize, const Size& imageSizeInMcu, const McuIndex* mcuIndices, int16_t (*blocks)[Dct::BlockSize2], int count) const;
  bool encode(const EncoderBuffer::MetaData& encoderBufferMetaData, BitBuffer& dst, std::vector<int>& prevComponentDc, const int16_t (*blocks)[Dct::BlockSize2], int count) const;

  static bool reserveEstimatedBitCount(BitBuffer& bitBuffer, const EncoderBuffer::MetaData& encoderBufferMetaData, int64_t blockCount, int quality);

private:
  std::vector<QuantizationTable> m_quantizationTables;
  std::vector<int> m_componentQuantizationTableIndices;
  std::vector<HuffmanEncoder> m_huffmanEncoders;
  std::vector<int> m_componentHuffmanEncoderIndices;
  alignas(32) std::vector<Quantizer> m_quantizers;
  int m_storingOrder[Dct::BlockSize2];
};

}
