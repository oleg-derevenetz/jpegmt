#pragma once

#include <cstdint>
#include <vector>

#include "JpegDCT.h"

namespace Jpeg
{

struct ImageMetaData;
class ThreadPool;

class OutputStream
{
public:
  virtual ~OutputStream() = default;

  virtual int64_t writeJpegBytes(const char* bytes, int64_t count) = 0;
};

struct EncodingOptions
{
  enum EncoderBufferItemType
  {
    Int16,
    Int32
  };

  bool m_averageInRgbSpace = true;
  EncoderBufferItemType m_encoderBufferItemType = Int16;
  int m_encoderBufferMaxSimdLength = 0x7fffffff;
  int m_huffmanEncoderMaxSimdLength = 0x7fffffff;
  int m_byteStuffingMaxSimdLength = 0x7fffffff;
};

class Writer
{
public:
  Writer(OutputStream* stream, ThreadPool* threadPool = nullptr);

  bool setQuality(int quality);

  EncodingOptions::EncoderBufferItemType getEncoderBufferItemType(const EncodingOptions& options) const;
  int getEncoderBufferSimdLength(const EncodingOptions& options) const;
  int getHuffmanEncoderSimdLength(const EncodingOptions& options) const;
  int getByteStuffingSimdLength(const EncodingOptions& options) const;

  bool write(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncodingOptions& options = EncodingOptions());

  bool writeHeader(const ImageMetaData& imageMetaData);
  bool writeFrameHeader(const ImageMetaData& imageMetaData);
  bool writeScanHeader(const ImageMetaData& imageMetaData);
  bool compressAndWrite(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncodingOptions& options = EncodingOptions());
  bool writeFileTrailer();

protected:
  static int qualityToScale(int quality);

  struct Component
  {
    int m_id = -1;
    int m_hSamplingFactor = 0;
    int m_vSamplingFactor = 0;
    int m_huffmanTableIndex = -1;
    int m_quantizationTableIndex = -1;

    Component();
    Component(int id, int hSamplingFactor, int vSamplingFactor, int huffmanTableIndex, int quantizationTableIndex);
  };
  std::vector<Component> imageComponents(const ImageMetaData& imageMetaData) const;

private:
  OutputStream* m_stream;
  ThreadPool* m_threadPool;
  int m_quality = 75;
  int64_t m_bytesWritten = 0;
  QuantizationTable m_luminanceQuantizationTable;
  QuantizationTable m_chrominanceQuantizationTable;
  HuffmanTable m_luminanceDcHuffmanTable;
  HuffmanTable m_luminanceAcHuffmanTable;
  HuffmanTable m_chrominanceDcHuffmanTable;
  HuffmanTable m_chrominanceAcHuffmanTable;
  int m_luminanceQuantizationTableIndex = 0;
  int m_chrominanceQuantizationTableIndex = 1;
  int m_luminanceHuffmanTableIndex = 0;
  int m_chrominanceHuffmanTableIndex = 1;
};

}
