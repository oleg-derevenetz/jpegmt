#include "ImageUtils.h"

#include <cassert>

#include <Jpeg/JpegImageMetaData.h>
#include <Jpeg/Rgb.h>

#include "Image.h"

Image makeDebugImage()
{
  int size = 8 * 250;
  Image image = Image::create(size, size, Jpeg::ImageMetaData::Grayscale8);
  uint8_t* pixels = (uint8_t*)image.m_pixels.get();

  int xblocks = image.m_metaData.m_size.m_width / 8;
  int yblocks = image.m_metaData.m_size.m_height / 8;

  uint8_t value = 0;
  for (int y = 0; y < yblocks; y++)
  {
    for (int x = 0; x < xblocks; x++)
    {
      uint8_t* block = pixels + y * 8 * image.m_metaData.m_size.m_width + x * 8;

      for (int i = 0; i < 8; i++)
      {
        for (int j = 0; j < 8; j++)
          block[i * image.m_metaData.m_size.m_width + j] = value;
      }

      value++;
    }
  }

  return image;
}

Image makeDebugImageRgb()
{
  int size = 16 * 10;
  Image image = Image::create(size, size, Jpeg::ImageMetaData::Bgra32);
  uint32_t* pixels = (uint32_t*)image.m_pixels.get();

  int xblocks = image.m_metaData.m_size.m_width / 16;
  int yblocks = image.m_metaData.m_size.m_height / 16;

  uint8_t value = 0;
  for (int y = 0; y < yblocks; y++)
  {
    for (int x = 0; x < xblocks; x++)
    {
      uint32_t* block = pixels + y * 16 * image.m_metaData.m_size.m_width + x * 16;
      uint32_t value32 = Jpeg::Rgb<Jpeg::ImageMetaData::Bgra32>::rgb(value, value, value);

      for (int i = 0; i < 16; i++)
      {
        for (int j = 0; j < 16; j++)
          block[i * image.m_metaData.m_size.m_width + j] = value32;
      }

      value++;
    }
  }

  return image;
}

Image compareImages(const Image& src, const Image& check, const Image& test)
{
  using namespace Jpeg;
  ImageMetaData::Format format = src.m_metaData.m_format;
  Size size = src.m_metaData.m_size;
  assert(size == check.m_metaData.m_size && size == test.m_metaData.m_size && check.m_metaData.m_format == test.m_metaData.m_format);

  int totalDifferentPixels = 0;
  int equalErrorPixels = 0;
  int lessErrorPixels = 0;
  int greaterErrorPixels = 0;
  double testErrorSum = 0, checkErrorSum = 0;
  double maxTestError = 0, maxCheckError = 0;

  Image diffImage;

  if (format == ImageMetaData::Grayscale8)
  {
    diffImage = Image::create(size.m_width, size.m_height, ImageMetaData::Grayscale8);

    for (int y = 0; y < size.m_height; y++)
    {
      const uint8_t* srcLine = (const uint8_t*)src.scanline(y);
      const uint8_t* checkLine = (const uint8_t*)check.scanline(y);
      const uint8_t* testLine = (const uint8_t*)test.scanline(y);
      uint8_t* diffLine = (uint8_t*)diffImage.scanline(y);

      for (int x = 0; x < size.m_width; x++)
      {
        if (checkLine[x] == testLine[x])
        {
          diffLine[x] = 0;
          continue;
        }

        int checkErr = checkLine[x] - srcLine[x];
        int testErr = testLine[x] - srcLine[x];

        if (testErr > checkErr)
        {
          greaterErrorPixels++;
          maxTestError = std::max<double>(maxTestError, testErr);
          testErrorSum += testErr;
        }
//        else if (testDR < checkDR || testDG < checkDG || testDB < checkDB)
        else if (testErr < checkErr)
        {
          lessErrorPixels++;
          maxCheckError = std::max<double>(maxCheckError, checkErr);
          checkErrorSum += checkErr;
        }
        else
          equalErrorPixels++;

        totalDifferentPixels++;
        diffLine[x] = std::abs(testLine[x] - checkLine[x]) * 10;
      }
    }
  }
  else
  {
    constexpr ImageMetaData::Format rgbFormat = ImageMetaData::Rgba32;
    diffImage = Image::create(size.m_width, size.m_height, rgbFormat);

    assert(format == ImageMetaData::Rgba32 || format == ImageMetaData::Bgra32);

    Image srcImage = src, checkImage = check, testImage = test;
    if (format != rgbFormat)
      srcImage = src.converted(rgbFormat);
    if (testImage.m_metaData.m_format != rgbFormat)
      testImage = test.converted(rgbFormat);
    if (checkImage.m_metaData.m_format != rgbFormat)
      checkImage = check.converted(rgbFormat);
    for (int y = 0; y < size.m_height; y++)
    {
      const uint32_t* srcLine = (const uint32_t*)(srcImage.scanline(y));
      const uint32_t* checkLine = (const uint32_t*)(checkImage.scanline(y));
      const uint32_t* testLine = (const uint32_t*)(testImage.scanline(y));
      uint32_t* diffLine = (uint32_t*)(diffImage.scanline(y));

      for (int x = 0; x < size.m_width; x++)
      {
        if (checkLine[x] == testLine[x])
        {
          diffLine[x] = Rgb<rgbFormat>::rgb(0, 0, 0);
          continue;
        }

        int srcR = Rgb<rgbFormat>::red(srcLine[x]), srcG = Rgb<rgbFormat>::green(srcLine[x]), srcB = Rgb<rgbFormat>::blue(srcLine[x]);
        int checkDR = Rgb<rgbFormat>::red(checkLine[x]) - srcR, checkDG = Rgb<rgbFormat>::green(checkLine[x]) - srcG, checkDB = Rgb<rgbFormat>::blue(checkLine[x]) - srcB;
        int testDR = Rgb<rgbFormat>::red(testLine[x]) - srcR, testDG = Rgb<rgbFormat>::green(testLine[x]) - srcG, testDB = Rgb<rgbFormat>::blue(testLine[x]) - srcB;
        int rw = 2 * 256 + srcR, gw = 4 * 256, bw = 3 * 256 - srcR, dw = 256;
        int testErr2 = rw * testDR * testDR + gw * testDG * testDG + bw * testDB * testDB;
        int checkErr2 = rw * checkDR * checkDR + gw * checkDG * checkDG + bw * checkDB * checkDB;
//        if (testDR > checkDR || testDG > checkDG || testDB > checkDB)
        if (testErr2 > checkErr2)
        {
          greaterErrorPixels++;
          double testErr = sqrt(testErr2 / dw) - sqrt(checkErr2 / dw);
          maxTestError = std::max(maxTestError, testErr);
          testErrorSum += testErr;
        }
//        else if (testDR < checkDR || testDG < checkDG || testDB < checkDB)
        else if (testErr2 < checkErr2)
        {
          lessErrorPixels++;
          double checkErr = sqrt(checkErr2 / dw) - sqrt(testErr2 / dw);
          maxCheckError = std::max(maxCheckError, checkErr);
          checkErrorSum += checkErr;
        }
        else
          equalErrorPixels++;

        totalDifferentPixels++;
        diffLine[x] = Rgb<rgbFormat>::rgb(
          std::abs(Rgb<rgbFormat>::red(testLine[x]) - Rgb<rgbFormat>::red(checkLine[x])) * 10,
          std::abs(Rgb<rgbFormat>::green(testLine[x]) - Rgb<rgbFormat>::green(checkLine[x])) * 10,
          std::abs(Rgb<rgbFormat>::blue(testLine[x]) - Rgb<rgbFormat>::blue(checkLine[x])) * 10);
      }
    }
  }

  if (totalDifferentPixels)
  {
    printf("total pixels: %d\n", size.m_width * size.m_height);
    printf("total different pixels: %d\n", totalDifferentPixels);
    printf("equal error pixels: %d\n", equalErrorPixels);
    printf("greater error pixels: %d\n", greaterErrorPixels);
    printf("less error pixels: %d\n", lessErrorPixels);
    printf("libjpeg error max/sum: %lf/%lf\n", maxCheckError, checkErrorSum);
    printf("jpegmt error max/sum: %lf/%lf\n", maxTestError, testErrorSum);
    return diffImage;
  }

  return Image();
}
