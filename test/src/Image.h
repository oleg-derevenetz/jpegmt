#pragma once

#include <memory>
#include <string>

#include <Jpeg/JpegImageMetaData.h>

struct Image
{
  Jpeg::ImageMetaData m_metaData;
  std::shared_ptr<char> m_pixels;
  int m_alignment = 1;

  enum Format
  {
    Unknown,
    Jpeg,
    Png,
  };

  bool isValid() const;
  bool allocPixels(int alignment);
  char* scanline(int y);
  const char* scanline(int y) const;

  Image copy() const;
  Image converted(Jpeg::ImageMetaData::Format format) const;

  static Image create(int width, int height, Jpeg::ImageMetaData::Format format, int alignment = 32);
  static Image load(const std::string& fileName, int alignment = 32);
  static Image load(std::filebuf& file, Format format = Unknown, int alignment = 32);
};
