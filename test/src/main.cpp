#include <algorithm>
#include <cassert>
#include <fstream>

#include <Helper/ThreadPool.h>

#include <Jpeg/JpegImageMetaData.h>
#include <Jpeg/JpegThreadPool.h>
#include <Jpeg/JpegWriter.h>

#include "ElapsedTimer.h"
#include "Image.h"
#include "ImageUtils.h"
#ifdef WITH_LIBJPEG
#include "JpegLibWrapper.h"
#endif

static int threadCount = -1;
#ifndef NDEBUG
static int testPassCount = 1;
#else
static int testPassCount = 10;
#endif
static int quality = 75;
static std::string srcImagePath;
static Jpeg::EncodingOptions encodingOptions;

class JpegThreadPool : public Jpeg::ThreadPool
{
public:
  JpegThreadPool(int maxThreadCount = -1) : m_threadPool(maxThreadCount) {};

protected:
  int getMaxThreadCount() const override
  {
    return m_threadPool.getThreadCount();
  }

  void executeWorkers(const WorkerFunction& f, const std::vector< std::pair<int64_t, int64_t> >& ranges) override
  {
    for(size_t i = 0; i < ranges.size(); i++)
    {
      m_threadPool.addJob([i, ranges, f]()
        {
          f((int)i, ranges.at(i).first, ranges.at(i).second);
        });
    }
    m_threadPool.waitJobs();
  }

protected:
  Helper::ThreadPool m_threadPool;
};

static std::unique_ptr<JpegThreadPool> threadPool;

class JpegMemoryOutputStream : public Jpeg::OutputStream
{
public:
  JpegMemoryOutputStream(size_t bytesToReserve = 0)
  {
    if (bytesToReserve > 0)
      m_buffer.reserve(bytesToReserve);
  };

  int64_t writeJpegBytes(const char* bytes, int64_t count) override
  {
    m_buffer.insert(m_buffer.end(), bytes, bytes + count);
    return count;
  }

protected:
  std::vector<char> m_buffer;
};

class JpegFileOutputStream : public Jpeg::OutputStream
{
public:
  JpegFileOutputStream(const std::string& path) : m_path(path)
  {
  };

  bool open()
  {
    return m_file.open(m_path, std::ios::binary | std::ios::out) != nullptr;
  }

  void close()
  {
    m_file.close();
  }

  int64_t writeJpegBytes(const char* bytes, int64_t count) override
  {
    return m_file.sputn(bytes, count);
  }

protected:
  std::filebuf m_file;
  std::string m_path;
};
/*
void test0(const QImage& image)
{
  QBuffer buffer;
  for (int i = 0; i < TestPassCount; i++)
  {
    QImageWriter writer(&buffer, "jpg");
    writer.write(image);
  }
}
*/
#ifdef WITH_LIBJPEG
void test_libjpeg(const Image& image)
{
  const Jpeg::ImageMetaData& imageMetaData = image.m_metaData;
  JpegMemoryOutputStream buffer;
  for (int i = 0; i < testPassCount; i++)
  {
    JpegLibWritter writer(&buffer);
    writer.setQuality(quality);

    writer.write(imageMetaData, (const uint8_t*)image.scanline(0));
  }
}
#endif

void test_jpegmt(const Image& image)
{
  const Jpeg::ImageMetaData& imageMetaData = image.m_metaData;
  JpegMemoryOutputStream buffer;
  for (int i = 0; i < testPassCount; i++)
  {
    Jpeg::Writer writer(&buffer, threadPool.get());
    writer.setQuality(quality);

    writer.write(imageMetaData, (const uint8_t*)image.scanline(0), encodingOptions);
  }
}

static int usage()
{
  printf(
    "usage:\n"
    "jpegtest [options] <path to the test image>\n"
    "options:\n"
    "\t-t | --threads <count>     - number of threads to use\n"
    "\t-n | --passes <count>      - number of test pasess\n"
    "\t-q | --quality <value>     - jpeg quality [1..100]\n"
    "\t--force-int32              - force using int32 simd\n"
    "\t--max-simd-bits            - max width form simd in bits\n"
    "\t--[no-]optimize-averaging  - enable/disable optimization of component averaging\n"
  );

  return 1;
}

static bool getOptionIntValue(int argc, char* argv[], int index, int& value)
{
  if (index + 1 >= argc)
  {
    printf("value required for '%s' option\n", argv[index]);
    return false;
  }

  value = atoi(argv[index + 1]);
  return true;
}

static bool parseArgs(int argc, char* argv[])
{
  int maxSimdBits = 0x7fffffff;

  for(int i = 1; i < argc; i++)
  {
    if (*argv[i] == '-')
    {
      std::string arg(argv[i]);
      if (arg == "-t" || arg == "--threads")
      {
        if (!getOptionIntValue(argc, argv, i++, threadCount))
          return false;
      }
      else if (arg == "-n" || arg == "--passes")
      {
        if (!getOptionIntValue(argc, argv, i++, testPassCount))
          return false;
      }
      else if (arg == "-q" || arg == "--quality")
      {
        if (!getOptionIntValue(argc, argv, i++, quality))
          return false;
      }
      else if (arg == "--force-int32")
        encodingOptions.m_encoderBufferItemType = Jpeg::EncodingOptions::Int32;
      else if (arg == "--max-simd-bits")
      {
        if (!getOptionIntValue(argc, argv, i++, maxSimdBits))
          return false;
      }
      else if (arg == "--optimize-averaging")
        encodingOptions.m_averageInRgbSpace = true;
      else if (arg == "--no-optimize-averaging")
        encodingOptions.m_averageInRgbSpace = false;
      else
      {
        printf("unknown option: %s\n", argv[i]);
        return false;
      }
    }
    else
      srcImagePath = argv[i];
  }

  int itemBits = encodingOptions.m_encoderBufferItemType == Jpeg::EncodingOptions::Int32 ? 32 : 16;
  encodingOptions.m_encoderBufferMaxSimdLength = std::max(maxSimdBits / itemBits, 1);
  encodingOptions.m_huffmanEncoderMaxSimdLength = std::max(maxSimdBits / 16, 1);
  encodingOptions.m_byteStuffingMaxSimdLength = std::max(maxSimdBits / 8, 1);

  if (srcImagePath.empty())
  {
    printf("no test image specified\n");
    return false;
  }

  return true;
}

int main(int argc, char *argv[])
{
  if (!parseArgs(argc, argv))
    return usage();

#if 1
  Image image = Image::load(srcImagePath);
  if (!image.isValid())
  {
    printf("failed to read image from file: %s\n", srcImagePath.data());
    return 1;
  }
//  image = image.scaled(QSize(2000, 2000), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
//  image = image.converted(Jpeg::ImageMetaData::Grayscale8);
#else
  image = makeDebugImageRgb();
#endif

  threadPool = std::make_unique<JpegThreadPool>(threadCount);

#ifdef WITH_LIBJPEG
  JpegFileOutputStream check("_check.jpg");
  if (check.open())
  {
    const Jpeg::ImageMetaData& imageMetaData = image.m_metaData;
    JpegLibWritter jpegWriter(&check);
    jpegWriter.setQuality(quality);

    bool result = jpegWriter.write(imageMetaData, (const uint8_t*)image.scanline(0));
    assert(result);
    (void)result;

    check.close();
  }
#endif

  JpegFileOutputStream f("_test.jpg");
  if (f.open())
  {
    const Jpeg::ImageMetaData& imageMetaData = image.m_metaData;
    Jpeg::Writer jpegWriter(&f, threadPool.get());
    jpegWriter.setQuality(quality);

    Jpeg::EncodingOptions::EncoderBufferItemType itemType = jpegWriter.getEncoderBufferItemType(encodingOptions);
    printf("encoder buffer  simd type: %sx%d\n", itemType == Jpeg::EncodingOptions::Int32 ? "int32" : "int16", jpegWriter.getEncoderBufferSimdLength(encodingOptions));
    printf("huffman encoder simd type: int16x%d\n", jpegWriter.getHuffmanEncoderSimdLength(encodingOptions));
    printf("byte stuffing   simd type: int8x%d\n", jpegWriter.getByteStuffingSimdLength(encodingOptions));

    bool result = jpegWriter.write(imageMetaData, (const uint8_t*)image.scanline(0), encodingOptions);
    assert(result);
    (void)result;

    f.close();
  }

#ifdef WITH_LIBJPEG
  Image checkImage = checkImage.load("_check.jpg");
  Image testImage = testImage.load("_test.jpg");
  if (checkImage.isValid() && testImage.isValid())
  {
    Image diffImage = compareImages(image, checkImage, testImage);
    if (diffImage.isValid())
    {
//      diffImage.save("diff.bmp");
    }
  }
  else
  {
    printf("failed to load images for check!\n");
    assert(false);
    return 1;
  }
#endif

#if 1
  ElapsedTimer timer;
/*
  timer.start();
  test0(image);
  printf("libjpeg time: %.3lf msec\n", timer.elapsed() / 1e6);
*/
  double totalPixels = (double)image.m_metaData.m_size.m_width * image.m_metaData.m_size.m_height * testPassCount;
  double unitFactor = 1e3;
#ifdef WITH_LIBJPEG
  timer.start();
  test_libjpeg(image);
  printf("libjpeg time: %.3lf msec (%.3lf MPixels/sec)\n", timer.elapsed() / 1e6, unitFactor * totalPixels / timer.elapsed());
#endif
  timer.start();
  test_jpegmt(image);
  printf("jpegmt time: %.3lf msec (%.3lf MPixels/sec)\n", timer.elapsed() / 1e6, unitFactor * totalPixels / timer.elapsed());
#endif

  return 0;
}
