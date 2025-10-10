#include <Jpeg/JpegWriter.h>

#include <algorithm>
#include <mutex>

#include <Helper/Platform/Cpu/simd.h>

#include <Jpeg/JpegImageMetaData.h>
#include <Jpeg/JpegThreadPool.h>

#include "Encoder/Common.h"
#include "Encoder/JpegEncoder.h"
#include "JpegFileFormat.h"

static bool isGrayscaleImage(const Jpeg::ImageMetaData& imageMetaData)
{
  return imageMetaData.m_format == Jpeg::ImageMetaData::Grayscale8;
}

namespace Jpeg
{

Writer::Writer(OutputStream* stream, ThreadPool* threadPool) :
  m_stream(stream), m_threadPool(threadPool),
  m_luminanceQuantizationTable(QuantizationTable::defaultLuminanceTable().scaled(qualityToScale(m_quality))),
  m_chrominanceQuantizationTable(QuantizationTable::defaultChrominanceTable().scaled(qualityToScale(m_quality))),
  m_luminanceDcHuffmanTable(HuffmanTable::defaultLuminanceDcTable()),
  m_luminanceAcHuffmanTable(HuffmanTable::defaultLuminanceAcTable()),
  m_chrominanceDcHuffmanTable(HuffmanTable::defaultChrominanceDcTable()),
  m_chrominanceAcHuffmanTable(HuffmanTable::defaultChrominanceAcTable())
{
}

bool Writer::setQuality(int quality)
{
  if (quality < 1 || quality > 100)
    return false;

  m_quality = quality;
  m_luminanceQuantizationTable = QuantizationTable::defaultLuminanceTable().scaled(qualityToScale(m_quality));
  m_chrominanceQuantizationTable = QuantizationTable::defaultChrominanceTable().scaled(qualityToScale(m_quality));

  return true;
}

EncoderBufferItemType Writer::getEncoderBufferItemType(const EncodingOptions& options)
{
  return options.m_encoderBufferItemType;
}

int Writer::getEncoderBufferSimdLength(const EncodingOptions& options)
{
  return EncoderBuffer::MetaData::getSimdLength(options);
}

int Writer::getHuffmanEncoderSimdLength(const EncodingOptions& options)
{
  return HuffmanEncoderOptions(options.m_huffmanEncoderMaxSimdLength, options.m_byteStuffingMaxSimdLength).m_encoderSimdLength;
}

int Writer::getByteStuffingSimdLength(const EncodingOptions& options)
{
  return HuffmanEncoderOptions(options.m_huffmanEncoderMaxSimdLength, options.m_byteStuffingMaxSimdLength).m_byteStuffingSimdLength;
}

bool Writer::write(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncodingOptions& options)
{
  if (!m_stream)
    return false;

  if (!writeHeader(imageMetaData))
    return false;
  if (!writeFrameHeader(imageMetaData))
    return false;
  if (!writeScanHeader(imageMetaData))
    return false;
  if (!compressAndWrite(imageMetaData, pixels, options))
    return false;
  if (!writeFileTrailer())
    return false;
  return true;
}

bool Writer::writeHeader(const ImageMetaData& imageMetaData)
{
  int64_t bytesWritten = FileFormat::writeMarker(m_stream, FileFormat::SOI);
  if (bytesWritten <= 0)
    return false;
  m_bytesWritten += bytesWritten;

  bytesWritten = FileFormat::writeMarker(m_stream, FileFormat::APP0, Jfif::fromImage(imageMetaData).toByteSequence());
  if (bytesWritten <= 0)
    return false;
  m_bytesWritten += bytesWritten;

  bytesWritten = FileFormat::writeQuantizationTable(m_stream, m_luminanceQuantizationTableIndex, m_luminanceQuantizationTable);
  if (bytesWritten <= 0)
    return false;
  m_bytesWritten += bytesWritten;

  if (!isGrayscaleImage(imageMetaData))
  {
    bytesWritten = FileFormat::writeQuantizationTable(m_stream, m_chrominanceQuantizationTableIndex, m_chrominanceQuantizationTable);
    if (bytesWritten <= 0)
      return false;
    m_bytesWritten += bytesWritten;
  }

  return true;
}

bool Writer::writeFrameHeader(const ImageMetaData& imageMetaData)
{
  std::vector<Component> components = imageComponents(imageMetaData);
  std::vector<FrameHeader::ComponentInfo> componentInfo;

  for (size_t i = 0; i < components.size(); i++)
  {
    const Component& c = components.at(i);
    componentInfo.push_back(FrameHeader::ComponentInfo(c.m_id, c.m_hSamplingFactor, c.m_vSamplingFactor, c.m_quantizationTableIndex));
  }

  FrameHeader frameHeader(8, imageMetaData.m_size.m_width, imageMetaData.m_size.m_height, componentInfo);
  int64_t bytesWritten = FileFormat::writeMarker(m_stream, FileFormat::SOF0, frameHeader.toByteSequence());
  if (bytesWritten <= 0)
    return false;
  m_bytesWritten += bytesWritten;

  bytesWritten = FileFormat::writeHuffmanTable(m_stream, m_luminanceHuffmanTableIndex, false, m_luminanceDcHuffmanTable);
  if (bytesWritten <= 0)
    return false;
  m_bytesWritten += bytesWritten;

  bytesWritten = FileFormat::writeHuffmanTable(m_stream, m_luminanceHuffmanTableIndex, true, m_luminanceAcHuffmanTable);
  if (bytesWritten <= 0)
    return false;
  m_bytesWritten += bytesWritten;

  if (!isGrayscaleImage(imageMetaData))
  {
    bytesWritten = FileFormat::writeHuffmanTable(m_stream, m_chrominanceHuffmanTableIndex, false, m_chrominanceDcHuffmanTable);
    if (bytesWritten <= 0)
      return false;
    m_bytesWritten += bytesWritten;

    bytesWritten = FileFormat::writeHuffmanTable(m_stream, m_chrominanceHuffmanTableIndex, true, m_chrominanceAcHuffmanTable);
    if (bytesWritten <= 0)
      return false;
    m_bytesWritten += bytesWritten;
  }

  return true;
}

bool Writer::writeScanHeader(const ImageMetaData& imageMetaData)
{
  std::vector<Component> components = imageComponents(imageMetaData);
  std::vector<ScanHeader::ComponentInfo> componentInfo;

  for (size_t i = 0; i < components.size(); i++)
  {
    const Component& c = components.at(i);
    componentInfo.push_back(ScanHeader::ComponentInfo(c.m_id, c.m_huffmanTableIndex, c.m_huffmanTableIndex));
  }

  ScanHeader scanHeader(componentInfo);
  int64_t bytesWritten = FileFormat::writeMarker(m_stream, FileFormat::SOS, scanHeader.toByteSequence());
  if (bytesWritten <= 0)
    return false;
  m_bytesWritten += bytesWritten;

  return true;
}

namespace
{
  struct AllocQuantizationBufferCallable : public ReturnValueCallable<int16_t (*)[Dct::BlockSize2], nullptr>
  {
    template <typename T, int SimdLength>
    static int16_t (*perform(int count))[Dct::BlockSize2]
    {
      return (int16_t (*)[Dct::BlockSize2])EncoderBuffer::allocSimdBuffer<T, SimdLength>(count);
    }
  };

  struct ReleaseQuantizationBufferCallable : public NoReturnValueCallable
  {
    template <typename T, int SimdLength>
    static void perform(int16_t (*buffer)[Dct::BlockSize2])
    {
      return EncoderBuffer::releaseSimdBuffer<T, SimdLength>(buffer);
    }
  };
}

int16_t (*allocQuantizationBuffer(const EncoderBuffer::MetaData& bufferMetaData, const HuffmanEncoderOptions& huffmanEncoderOptions, int count))[Dct::BlockSize2]
{
  int simdLength;
  switch(bufferMetaData.m_itemType)
  {
  case EncoderBufferItemType::Int16:
    simdLength = std::max(bufferMetaData.m_simdLength, huffmanEncoderOptions.m_encoderSimdLength);
    return SimdFunctionChooser<AllocQuantizationBufferCallable>::perform<int16_t>(simdLength, count);
  case EncoderBufferItemType::Int32:
    simdLength = std::max(bufferMetaData.m_simdLength, huffmanEncoderOptions.m_encoderSimdLength / 2);
    return SimdFunctionChooser<AllocQuantizationBufferCallable>::perform<int32_t>(bufferMetaData.m_simdLength, count);
  }

  assert(false);
  return nullptr;
}

void releaseQuantizationBuffer(const EncoderBuffer::MetaData& bufferMetaData, const HuffmanEncoderOptions& huffmanEncoderOptions, int16_t (*buffer)[Dct::BlockSize2])
{
  int simdLength;
  switch (bufferMetaData.m_itemType)
  {
  case EncoderBufferItemType::Int16:
    simdLength = std::max(bufferMetaData.m_simdLength, huffmanEncoderOptions.m_encoderSimdLength);
    return SimdFunctionChooser<ReleaseQuantizationBufferCallable>::perform<int16_t>(simdLength, buffer);
  case EncoderBufferItemType::Int32:
    simdLength = std::max(bufferMetaData.m_simdLength, huffmanEncoderOptions.m_encoderSimdLength / 2);
    return SimdFunctionChooser<ReleaseQuantizationBufferCallable>::perform<int32_t>(simdLength, buffer);
  }
  assert(false);
}

bool Writer::compressAndWrite(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncodingOptions& options)
{
  std::vector<Component> components = imageComponents(imageMetaData);
  std::vector<EncoderBuffer::MetaData::ComponentInfo> componentInfo;
  std::vector<int> componentQuantizationTableIndices;
  std::vector<int> componentHuffmanEncoderIndices;
  for(size_t i = 0; i < components.size(); i++)
  {
    const Component& component = components.at(i);
    EncoderBuffer::MetaData::ComponentInfo info;
    info.m_hBlocks = component.m_hSamplingFactor;
    info.m_vBlocks = component.m_vSamplingFactor;
    switch(component.m_id)
    {
    case 1:
      info.m_type = EncoderBuffer::MetaData::Y;
      componentQuantizationTableIndices.push_back(0);
      componentHuffmanEncoderIndices.push_back(0);
      break;
    case 2:
      info.m_type = EncoderBuffer::MetaData::Cb;
      componentQuantizationTableIndices.push_back(1);
      componentHuffmanEncoderIndices.push_back(1);
      break;
    case 3:
      info.m_type = EncoderBuffer::MetaData::Cr;
      componentQuantizationTableIndices.push_back(1);
      componentHuffmanEncoderIndices.push_back(1);
      break;
    default:
      assert(false);
      return false;
    }
    componentInfo.push_back(info);
  }

  EncoderBuffer::MetaData encoderBufferMetaData(componentInfo, options);
  HuffmanEncoderOptions huffmanEncoderOptions(options.m_huffmanEncoderMaxSimdLength, options.m_byteStuffingMaxSimdLength);
  int bufferSimdBlocks = 1;
#ifndef TRANSPOSED_SIMD_BUFFER
  bufferSimdBlocks = encoderBufferMetaData.computeImageSizeInMcu(imageMetaData.m_size).m_width;
#else
  if (!m_threadPool || m_threadPool->getMaxThreadCount() <= 1)
    bufferSimdBlocks = 1 + ((encoderBufferMetaData.computeImageSizeInMcu(imageMetaData.m_size).m_width - 1) >> encoderBufferMetaData.m_simdLengthLog2);
#endif
  int bufferCount = encoderBufferMetaData.computeImageBufferCount(imageMetaData.m_size, bufferSimdBlocks);
  alignas(32) Encoder encoder(
    std::vector<QuantizationTable>{m_luminanceQuantizationTable, m_chrominanceQuantizationTable}, componentQuantizationTableIndices,
    std::vector<HuffmanEncoder>{{m_luminanceDcHuffmanTable, m_luminanceAcHuffmanTable}, {m_chrominanceDcHuffmanTable, m_chrominanceAcHuffmanTable}},
    componentHuffmanEncoderIndices);

  int nThreads = m_threadPool ? m_threadPool->computeThreadCount(bufferCount) : 1;
  std::vector<Encoder::BitBuffer> compressedParts(nThreads);
  int blocksPerBuffer = bufferSimdBlocks * encoderBufferMetaData.m_simdLength * encoderBufferMetaData.getMcuBlockCount();
  int16_t (*quantizationBuffer)[Dct::BlockSize2] = allocQuantizationBuffer(encoderBufferMetaData, huffmanEncoderOptions, blocksPerBuffer * nThreads * 2);
  struct LastBufferInfo
  {
    std::mutex mutex;
    bool computed = false;
  };
  LastBufferInfo* threadLastBufferInfo = new LastBufferInfo[nThreads];

  ThreadPool::WorkerFunction worker = [this, imageMetaData, pixels, options, huffmanEncoderOptions, &encoder /* by reference to keep alignment */,
    encoderBufferMetaData, bufferSimdBlocks, quantizationBuffer, blocksPerBuffer, threadLastBufferInfo, &compressedParts](int threadIndex, int64_t i0, int64_t i1)
    {
      EncoderBuffer encoderBuffer(encoderBufferMetaData, bufferSimdBlocks);
      std::vector<int> lastDc(encoderBufferMetaData.m_components.size(), 0);
      int16_t (*quantized)[Dct::BlockSize2] = quantizationBuffer + 2 * threadIndex * blocksPerBuffer;
#ifndef TRANSPOSED_SIMD_BUFFER
      int bufferMcuCount = bufferSimdBlocks;
#else
      int bufferMcuCount = encoderBufferMetaData.m_simdLength * bufferSimdBlocks;
#endif
      std::vector<EncoderBuffer::McuIndex> mcuIndicesVector(bufferMcuCount);
      EncoderBuffer::McuIndex* mcuIndices = mcuIndicesVector.data();
      const Size& imageSize = imageMetaData.m_size;
      Size imageSizeInMcu = encoderBufferMetaData.computeImageSizeInMcu(imageSize);

      if (threadIndex > 0)
      {
        LastBufferInfo& lastBufferInfo = threadLastBufferInfo[threadIndex - 1];
        int16_t (*lastBufferQuantized)[Dct::BlockSize2] = quantized - blocksPerBuffer;

        lastBufferInfo.mutex.lock();
        if (!lastBufferInfo.computed)
        {
          int count = encoderBuffer.getBufferMcuIndices(imageSizeInMcu, (int)i0 - 1, mcuIndices);
          encoder.quantize(encoderBuffer, imageMetaData, pixels, mcuIndices, lastBufferQuantized, count, options);
          encoder.optimizeDummyBlocks(encoderBufferMetaData, imageSize, imageSizeInMcu, mcuIndices, lastBufferQuantized, count);
          lastBufferInfo.computed = true;
        }
        lastBufferInfo.mutex.unlock();

        int16_t (*lastQuantizedMcu)[Dct::BlockSize2] = lastBufferQuantized + (bufferMcuCount - 1) * encoderBufferMetaData.getMcuBlockCount();
        for (int i = 0; i < (int)encoderBufferMetaData.m_components.size(); i++)
        {
          EncoderBuffer::MetaData::Component c = encoderBufferMetaData.getComponent(i);
          lastDc[i] = lastQuantizedMcu[c.m_blockOffset + c.m_blockCount - 1][0];
        }
      }

      Encoder::BitBuffer& compressionBuffer = compressedParts[threadIndex];
      Encoder::reserveEstimatedBitCount(compressionBuffer, encoderBufferMetaData, (i1 - i0 + 1) * bufferMcuCount, m_quality, huffmanEncoderOptions);

      for (int i = (int)i0; i < i1; i++)
      {
        int count = encoderBuffer.getBufferMcuIndices(imageSizeInMcu, i, mcuIndices);
        assert(count == bufferMcuCount);
        encoder.quantize(encoderBuffer, imageMetaData, pixels, mcuIndices, quantized, count, options);
        encoder.optimizeDummyBlocks(encoderBufferMetaData, imageSize, imageSizeInMcu, mcuIndices, quantized, count);
        encoder.encode(encoderBufferMetaData, compressionBuffer, lastDc, quantized, count, huffmanEncoderOptions);
      }

      LastBufferInfo& lastBufferInfo = threadLastBufferInfo[threadIndex];
      int16_t (*lastBufferQuantized)[Dct::BlockSize2] = quantized + blocksPerBuffer;
      lastBufferInfo.mutex.lock();
      if (!lastBufferInfo.computed)
      {
        int count = encoderBuffer.getBufferMcuIndices(imageSizeInMcu, (int)i1, mcuIndices);
        encoder.quantize(encoderBuffer, imageMetaData, pixels, mcuIndices, lastBufferQuantized, count, options);
        encoder.optimizeDummyBlocks(encoderBufferMetaData, imageSize, imageSizeInMcu, mcuIndices, lastBufferQuantized, count);
        lastBufferInfo.computed = true;
      }
      lastBufferInfo.mutex.unlock();
      encoder.encode(encoderBufferMetaData, compressionBuffer, lastDc, lastBufferQuantized, encoderBuffer.getBufferMcuIndices(imageSizeInMcu, (int)i1, nullptr), huffmanEncoderOptions);
    };

  if (m_threadPool && nThreads > 1)
    m_threadPool->executeParallel(worker, bufferCount);
  else
    worker(0, 0, bufferCount - 1);

  delete[] threadLastBufferInfo;
  threadLastBufferInfo = nullptr;
  releaseQuantizationBuffer(encoderBufferMetaData, huffmanEncoderOptions, quantizationBuffer);
  quantizationBuffer = nullptr;

  Encoder::BitBuffer compressed = Encoder::BitBuffer::merge(compressedParts, 0.1, huffmanEncoderOptions);
  if (!compressed.m_bits)
    return false;

  compressed.m_bitCount = HuffmanEncoder::padToByteBoundary(compressed.m_bits, compressed.m_bitCount);
  int64_t bytesToAdd = HuffmanEncoder::byteStuffingByteCount(compressed.m_bits, compressed.m_bitCount, huffmanEncoderOptions);
  if (!compressed.reserve(bytesToAdd * 8, huffmanEncoderOptions))
    return false;

  compressed.m_bitCount = HuffmanEncoder::byteStuffing(compressed.m_bits, compressed.m_bitCount, bytesToAdd, huffmanEncoderOptions);

  assert(compressed.m_bitCount % 8 == 0);
  int64_t written = m_stream->writeJpegBytes((char*)compressed.m_bits, compressed.m_bitCount / 8);

  compressedParts.clear();
  compressed = Encoder::BitBuffer();

  assert(written > 0);
  return written > 0;
}

bool Writer::writeFileTrailer()
{
  return FileFormat::writeMarker(m_stream, FileFormat::EOI);
}

std::vector<Writer::Component> Writer::imageComponents(const ImageMetaData& imageMetaData) const
{
  if (isGrayscaleImage(imageMetaData))
    return std::vector<Component>{{1, 1, 1, 0, 0}};

  return std::vector<Component>{{1, 2, 2, 0, 0}, {2, 1, 1, 1, 1}, {3, 1, 1, 1, 1}};
}

int Writer::qualityToScale(int quality)
{
  /* Safety limit on quality factor.  Convert 0 to 1 to avoid zero divide. */
  if (quality <= 0)
    quality = 1;
  if (quality > 100)
    quality = 100;

  /* The basic table is used as-is (scaling 100) for a quality of 50.
   * Qualities 50..100 are converted to scaling percentage 200 - 2*Q;
   * note that at Q=100 the scaling is 0, which will cause jpeg_add_quant_table
   * to make all the table entries 1 (hence, minimum quantization loss).
   * Qualities 1..50 are converted to scaling percentage 5000/Q.
   */
  return quality < 50 ? 5000 / quality : 200 - quality * 2;
}

Writer::Component::Component()
{
}

Writer::Component::Component(int id, int hSamplingFactor, int vSamplingFactor, int huffmanTableIndex, int quantizationTableIndex) :
  m_id(id), m_hSamplingFactor(hSamplingFactor), m_vSamplingFactor(vSamplingFactor), m_huffmanTableIndex(huffmanTableIndex), m_quantizationTableIndex(quantizationTableIndex)
{
}

}
