#pragma once

#include <cstdint>

namespace Jpeg
{
  struct ImageMetaData;
  class OutputStream;
}

class JpegLibWritter
{
public:
  JpegLibWritter(Jpeg::OutputStream* stream) : m_stream(stream)
  {
  }

  bool setQuality(int quality);

  bool write(const Jpeg::ImageMetaData& imageMetaData, const uint8_t* pixels);

private:
  Jpeg::OutputStream* m_stream;
  int m_quality = 75;
};
