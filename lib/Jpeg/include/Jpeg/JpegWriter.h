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
  bool m_averageInRgbSpace = true;
};

class Writer
{
public:
  struct ProfileData
  {
    int64_t headersTime = 0;
    int64_t encodingTime = 0;
    int64_t postEncodingTime = 0;
    int64_t streamWriteTime = 0;
    int64_t memoryReleaseTime = 0;
    int64_t otherCompressionTime = 0;

    int64_t timeSum() const;

    virtual void startTimer() = 0;
    virtual int64_t elapsed() const = 0;

    static void startTimer(ProfileData* data);
    static void updateHeadersTime(ProfileData* data, bool restartTimer);
    static void updateEncodingTime(ProfileData* data, bool restartTimer);
    static void updatePostEncodingTime(ProfileData* data, bool restartTimer);
    static void updateStreamWriteTime(ProfileData* data, bool restartTimer);
    static void updateMemoryReleaseTime(ProfileData* data, bool restartTimer);
    static void updateOtherCompressionTime(ProfileData* data, bool restartTimer);
  };

  Writer(OutputStream* stream, ThreadPool* threadPool = nullptr);

  bool setQuality(int quality);

  bool write(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncodingOptions& options = EncodingOptions(), ProfileData* profileData = nullptr);

  bool writeHeader(const ImageMetaData& imageMetaData);
  bool writeFrameHeader(const ImageMetaData& imageMetaData);
  bool writeScanHeader(const ImageMetaData& imageMetaData);
  bool compressAndWrite(const ImageMetaData& imageMetaData, const uint8_t* pixels, const EncodingOptions& options = EncodingOptions(), ProfileData* profileData = nullptr);
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
