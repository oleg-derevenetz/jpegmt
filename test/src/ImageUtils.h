#pragma once

struct Image;

namespace Jpeg
{
  struct ImageMetaData;
}

Image makeDebugImage();
Image makeDebugImageRgb();

Image compareImages(const Image& src, const Image& check, const Image& test);

