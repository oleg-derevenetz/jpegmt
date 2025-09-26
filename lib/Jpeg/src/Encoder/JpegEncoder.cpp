#include "JpegEncoder.h"

#include <cmath>
#include <cstring>

#include <Jpeg/JpegImageMetaData.h>

#include "JpegForwardDct.h"
#include "JpegQuantizer.h"
#include "JpegHuffmanEncoder.h"

namespace Jpeg
{

Encoder::Encoder(const std::vector<QuantizationTable>& quantizationTables, const std::vector<int>& componentQuantizationTableIndices,
  const std::vector<HuffmanEncoder>& huffmanEncoders, const std::vector<int>& componentHuffmanEncoderIndices) :
  m_quantizationTables(quantizationTables), m_componentQuantizationTableIndices(componentQuantizationTableIndices),
  m_huffmanEncoders(huffmanEncoders), m_componentHuffmanEncoderIndices(componentHuffmanEncoderIndices)
{
  for (size_t i = 0; i < m_quantizationTables.size(); i++)
    m_quantizers.push_back(Quantizer(m_quantizationTables.at(i)));

  for(int i = 0; i < Dct::BlockSize2; i++)
    m_storingOrder[Dct::m_zigzagOrder[i]] = i;
}

int Encoder::quantize(EncoderBuffer& encoderBuffer, const ImageMetaData& imageMetaData, const uint8_t* pixels, const McuIndex* mcuIndices, int16_t (*dst)[Dct::BlockSize2], int count, const EncodingOptions& options) const
{
//      Q_ASSERT(count == EncoderBuffer::SimdLength || i == endBufferIndex);
  encoderBuffer.importImageBlocks(imageMetaData, pixels, mcuIndices, count, options);
  ForwardDct::perform(encoderBuffer, m_quantizers.data(), m_componentQuantizationTableIndices.data());
  encoderBuffer.exportQuantizedBlocks(dst, count);

  return count;
}

void Encoder::optimizeDummyBlocks(const EncoderBuffer::MetaData& encoderBufferMetaData, const Size& imageSize, const Size& imageSizeInMcu, const McuIndex* mcuIndices, int16_t (*blocks)[Dct::BlockSize2], int count) const
{
  int xblocks = 1 + (imageSize.m_width - 1) / Dct::BlockSize;
  int yblocks = 1 + (imageSize.m_height - 1) / Dct::BlockSize;

  if (xblocks % encoderBufferMetaData.m_hScale > 0)
  {
    for(int n = 0; n < count; n++)
    {
      if (mcuIndices[n].m_x != imageSizeInMcu.m_width - 1)
        continue;

      for(size_t c = 0; c < encoderBufferMetaData.m_components.size(); c++)
      {
        const EncoderBuffer::MetaData::Component& component = encoderBufferMetaData.m_components.at(c);
        int hblocks = component.m_info.m_hBlocks;
        int xDummyBlocks = (hblocks - xblocks % hblocks) % hblocks;

        if (xDummyBlocks == 0)
          continue;

        for(int i = 0; i < component.m_info.m_vBlocks; i++)
        {
          int16_t (*iblocks)[Dct::BlockSize2] = &blocks[n * encoderBufferMetaData.m_mcuBlockCount + component.m_blockOffset + i * hblocks];

          for (int j = hblocks - xDummyBlocks; j < hblocks; j++)
          {
            memset(iblocks[j], 0, sizeof(iblocks[j]));
            iblocks[j][0] = iblocks[j - 1][0];
          }
        }
      }
    }
  }

  if (yblocks % encoderBufferMetaData.m_vScale > 0)
  {
    for (int n = 0; n < count; n++)
    {
      if (mcuIndices[n].m_y != imageSizeInMcu.m_height - 1)
        continue;

      for (size_t c = 0; c < encoderBufferMetaData.m_components.size(); c++)
      {
        const EncoderBuffer::MetaData::Component& component = encoderBufferMetaData.m_components.at(c);
        int hblocks = component.m_info.m_hBlocks;
        int vblocks = component.m_info.m_vBlocks;
        int yDummyBlocks = (vblocks - yblocks % vblocks) % vblocks;

        if (yDummyBlocks == 0)
          continue;

        for (int i = vblocks - yDummyBlocks; i < vblocks; i++)
        {
          int16_t (*iblocks)[Dct::BlockSize2] = &blocks[n * encoderBufferMetaData.m_mcuBlockCount + component.m_blockOffset + i * hblocks];

          for (int j = 0; j < hblocks; j++)
          {
            memset(iblocks[j], 0, sizeof(iblocks[j]));
            iblocks[j][0] = iblocks[j - 1][0];
          }
        }
      }
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

template<int BlockCount>
FORCE_INLINE void encodeComponent(const EncoderBuffer::MetaData::Component& component, const HuffmanEncoder& huffmanEncoder, int& prevComponentDc, const int16_t(*blocks)[Dct::BlockSize2], Encoder::BitBuffer& compressionBuffer)
{
  for (int j = 0; j < BlockCount; j++)
  {
    const int16_t* block = blocks[component.m_blockOffset + j];
    assert(compressionBuffer.m_wordsAllocated * sizeof(compressionBuffer.m_bits[0]) * 8 >= (size_t)compressionBuffer.m_bitCount);
    compressionBuffer.m_bitCount = huffmanEncoder.encode(block, prevComponentDc, compressionBuffer.m_bits, compressionBuffer.m_bitCount, compressionBuffer.m_wordsAllocated);
    assert(compressionBuffer.m_wordsAllocated * sizeof(compressionBuffer.m_bits[0]) * 8 >= (size_t)compressionBuffer.m_bitCount);
    prevComponentDc = block[0];
  }
}

bool Encoder::encode(const EncoderBuffer::MetaData& encoderBufferMetaData, BitBuffer& compressionBuffer, std::vector<int>& prevComponentDc, const int16_t (*blocks)[Dct::BlockSize2], int count) const
{
  int* prevDc = prevComponentDc.data();

  constexpr int countPerCheck = 8;
  constexpr int maxPossibleBitsPerBlock = 1024;
#if 1
  const int* mcuComponents = encoderBufferMetaData.m_mcuComponents.data();
  const int* encoderIndices = m_componentHuffmanEncoderIndices.data();
  const HuffmanEncoder* encoders = m_huffmanEncoders.data();

  for (int i = 0; i < count; i += countPerCheck, blocks += countPerCheck * encoderBufferMetaData.m_mcuBlockCount)
  {
    int mcuCount = std::min(countPerCheck, count - i);
    if (!compressionBuffer.reserve(mcuCount * encoderBufferMetaData.m_mcuBlockCount * maxPossibleBitsPerBlock))
      return false;

    compressionBuffer.m_bitCount = HuffmanEncoder::encode(blocks[0], mcuComponents, encoderBufferMetaData.m_mcuBlockCount, encoders, encoderIndices, prevDc, mcuCount,
      compressionBuffer.m_bits, compressionBuffer.m_bitCount, compressionBuffer.m_wordsAllocated);
  }
#else
  const std::vector<EncoderBuffer::MetaData::Component>& components = encoderBufferMetaData.m_components;
  int checkCount = count / countPerCheck;
  if (!compressionBuffer.reserve(count * encoderBufferMetaData.m_mcuBlockCount * maxPossibleBitsPerBlock))
    return false;

#if 1
  if (components.size() == 1 && components.at(0).m_blockCount == 1)
  {
    const EncoderBuffer::MetaData::Component& component = components.at(0);
    const HuffmanEncoder& huffmanEncoder = m_huffmanEncoders.at(m_componentHuffmanEncoderIndices.at(0));

    for (int i = 0; i < checkCount; i++)
    {
      for(int j = 0; j < countPerCheck; j++)
        encodeComponent<1>(component, huffmanEncoder, prevDc[0], blocks + (i * countPerCheck + j) * encoderBufferMetaData.m_mcuBlockCount, compressionBuffer);
      if (!compressionBuffer.reserve(countPerCheck * encoderBufferMetaData.m_mcuBlockCount * maxPossibleBitsPerBlock))
        return false;
    }
    for (int i = checkCount * countPerCheck; i < count; i++)
      encodeComponent<1>(component, huffmanEncoder, prevDc[0], blocks + i * encoderBufferMetaData.m_mcuBlockCount, compressionBuffer);

    return true;
  }
  if (components.size() == 3)
  {
    const EncoderBuffer::MetaData::Component& component0 = components.at(0);
    const EncoderBuffer::MetaData::Component& component1 = components.at(1);
    const EncoderBuffer::MetaData::Component& component2 = components.at(2);
    const HuffmanEncoder& huffmanEncoder0 = m_huffmanEncoders.at(m_componentHuffmanEncoderIndices.at(0));
    const HuffmanEncoder& huffmanEncoder1 = m_huffmanEncoders.at(m_componentHuffmanEncoderIndices.at(1));
    const HuffmanEncoder& huffmanEncoder2 = m_huffmanEncoders.at(m_componentHuffmanEncoderIndices.at(2));

    if (components.at(0).m_blockCount == 4 && components.at(1).m_blockCount == 1 && components.at(2).m_blockCount == 1)
    {
      for (int i = 0; i < checkCount; i++)
      {
        for (int j = 0; j < countPerCheck; j++)
        {
          encodeComponent<4>(component0, huffmanEncoder0, prevDc[0], blocks + (i * countPerCheck + j) * encoderBufferMetaData.m_mcuBlockCount, compressionBuffer);
          encodeComponent<1>(component1, huffmanEncoder1, prevDc[1], blocks + (i * countPerCheck + j) * encoderBufferMetaData.m_mcuBlockCount, compressionBuffer);
          encodeComponent<1>(component2, huffmanEncoder2, prevDc[2], blocks + (i * countPerCheck + j) * encoderBufferMetaData.m_mcuBlockCount, compressionBuffer);
        }
        if (!compressionBuffer.reserve(countPerCheck * encoderBufferMetaData.m_mcuBlockCount * maxPossibleBitsPerBlock))
          return false;
      }

      for (int i = checkCount * countPerCheck; i < count; i++)
      {
        encodeComponent<4>(component0, huffmanEncoder0, prevDc[0], blocks + i * encoderBufferMetaData.m_mcuBlockCount, compressionBuffer);
        encodeComponent<1>(component1, huffmanEncoder1, prevDc[1], blocks + i * encoderBufferMetaData.m_mcuBlockCount, compressionBuffer);
        encodeComponent<1>(component2, huffmanEncoder2, prevDc[2], blocks + i * encoderBufferMetaData.m_mcuBlockCount, compressionBuffer);
      }

      return true;
    }
  }
  Q_ASSERT(false);
#endif

  for (int i = 0; i < count; i++)
  {
    for (int c = 0; c < components.size(); c++)
    {
      const EncoderBuffer::MetaData::Component& component = components.at(c);
      const HuffmanEncoder& huffmanEncoder = m_huffmanEncoders.at(m_componentHuffmanEncoderIndices.at(c));

      for (int j = 0; j < component.m_blockCount; j++)
      {
        const int16_t* block = blocks[component.m_blockOffset + j];
        compressionBuffer.m_bitCount = huffmanEncoder.encode(block, prevDc[c], compressionBuffer.m_bits, compressionBuffer.m_bitCount, compressionBuffer.m_wordsAllocated);
        prevDc[c] = block[0];
      }
    }
    if (!compressionBuffer.reserve(countPerCheck * encoderBufferMetaData.m_mcuBlockCount * maxPossibleBitsPerBlock))
      return false;
  }
#endif

  return true;
}

bool Encoder::reserveEstimatedBitCount(BitBuffer& bitBuffer, const EncoderBuffer::MetaData& encoderBufferMetaData, int64_t blockCount, int quality)
{
  double estimatedBitsPerBlock = 32;
  if (quality >= 50)
    estimatedBitsPerBlock *= pow(2.0, (quality - 50) / 10.0);
  return bitBuffer.reserve(blockCount * encoderBufferMetaData.m_mcuBlockCount * (int)estimatedBitsPerBlock);
}

Encoder::BitBuffer Encoder::BitBuffer::merge(const std::vector<BitBuffer>& buffers, double reserveFraction)
{
  if (buffers.size() == 0)
    return BitBuffer();
  if (buffers.size() == 1)
    return buffers.at(0);

  int64_t totalBits = 0;
  for (size_t i = 0; i < buffers.size(); i++)
    totalBits += buffers.at(i).m_bitCount;

  totalBits += (int64_t)(totalBits * reserveFraction);

  BitBuffer merged = buffers.at(0);
  if (!merged.reserve(totalBits)) // NOTE: results.at(0) may be used if already has enough bits
    return BitBuffer();

  constexpr int wordBits = sizeof(merged.m_bits[0]) * 8;
  for (size_t i = 1; i < buffers.size(); i++)
  {
    int64_t dstw = merged.m_bitCount / wordBits;
    uint64_t* src = buffers.at(i).m_bits;
    int64_t srcBitCount = buffers.at(i).m_bitCount;
    int64_t srcwcnt = 1 + (srcBitCount - 1) / wordBits;
    int shift = merged.m_bitCount % wordBits;

    if (shift)
    {
      uint64_t reminder = merged.m_bits[dstw];

      for (int64_t j = 0; j < srcwcnt - 1; j++)
      {
        merged.m_bits[dstw++] = reminder | (src[j] >> shift);
        reminder = src[j] << (wordBits - shift);
      }

      int64_t srcReminderBits = srcBitCount % wordBits ? srcBitCount % wordBits : wordBits;
      if (wordBits - shift < srcReminderBits)
      {
        merged.m_bits[dstw++] = reminder | (src[srcwcnt - 1] >> shift);
        merged.m_bits[dstw] = src[srcwcnt - 1] << (wordBits - shift);
      }
      else
        merged.m_bits[dstw] = reminder | (src[srcwcnt - 1] >> shift);

      assert(dstw == (merged.m_bitCount + srcBitCount - 1) / wordBits);
    }
    else
      memcpy(merged.m_bits + dstw, src, srcwcnt * sizeof(merged.m_bits[0]));

    merged.m_bitCount += srcBitCount;
  }

  return merged;
}

bool Encoder::BitBuffer::reserve(int64_t bitCount)
{
  assert(m_wordsAllocated >= 0 && m_bitCount >= 0 && bitCount >= 0);
  assert(m_wordsAllocated * sizeof(m_bits[0]) * 8 >= (size_t)m_bitCount);
  if (m_wordsAllocated * sizeof(m_bits[0]) * 8 >= (size_t)(m_bitCount + bitCount))
    return true;

  int64_t bitsRequired = std::max<int64_t>(m_bitCount + bitCount, (int64_t)(m_bitCount * 1.5));
  int64_t wordsRequired = 1 + (bitsRequired - 1) / (sizeof(m_bits[0]) * 8);
  uint64_t* words = HuffmanEncoder::allocBitBuffer(wordsRequired);
  if (!words)
    return false;

//  assert(!m_bits || !m_bitCount);
  if (m_bits && m_bitCount)
    memcpy(words, m_bits, (1 + (m_bitCount - 1) / (sizeof(words[0]) * 8)) * sizeof(words[0]));
  m_buffer = std::shared_ptr<uint64_t>(words, HuffmanEncoder::freeBitBuffer);
  m_bits = words;
  m_wordsAllocated = wordsRequired;
  return true;
}

}
