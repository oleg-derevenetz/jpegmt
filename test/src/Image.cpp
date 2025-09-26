#include "Image.h"

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <fstream>

#include <Helper/Platform/Cpu/cpu.h>
#include <Helper/Platform/os.h>

#ifdef WITH_LIBJPEG
#include <jpeglib.h>
#endif
#ifdef WITH_LIBPNG
#include <png.h>
#endif

#include <Jpeg/Rgb.h>

Image Image::create(int width, int height, Jpeg::ImageMetaData::Format format, int alignment)
{
  Image image;
  Jpeg::ImageMetaData& metaData = image.m_metaData;
  metaData.m_size = Jpeg::Size{width, height};
  metaData.m_format = format;
  switch (format)
  {
  case Jpeg::ImageMetaData::Grayscale8:
    metaData.m_scanlineBytes = width;
    break;
  case Jpeg::ImageMetaData::Rgba32:
  case Jpeg::ImageMetaData::Bgra32:
    metaData.m_scanlineBytes = width * 4;
    break;
  case Jpeg::ImageMetaData::Invalid:
    return Image();
  }

  image.allocPixels(alignment);
  return image;
}

bool Image::isValid() const
{
  return m_pixels != nullptr;
}

#ifndef PLATFORM_OS_WINDOWS
static void* _aligned_malloc(size_t size, size_t alignment)
{
  return aligned_alloc(alignment, size);
}

static void _aligned_free(void* p)
{
  free(p);
}
#endif

bool Image::allocPixels(int alignment)
{
  if (!m_metaData.isValid())
    return false;

  assert(m_metaData.m_scanlineBytes >= m_metaData.m_size.m_width);
  m_pixels = std::shared_ptr<char>((char*)_aligned_malloc(m_metaData.m_scanlineBytes * m_metaData.m_size.m_height, alignment), _aligned_free);
  m_alignment = alignment;
  return m_pixels != nullptr;
}

char* Image::scanline(int y)
{
  return m_pixels.get() + m_metaData.m_scanlineBytes * y;
}

const char* Image::scanline(int y) const
{
  return m_pixels.get() + m_metaData.m_scanlineBytes * y;
}

Image Image::copy() const
{
  Image image{m_metaData, nullptr};
  image.allocPixels(m_alignment);
  memcpy(image.m_pixels.get(), m_pixels.get(), m_metaData.m_scanlineBytes * m_metaData.m_size.m_height);
  return image;
}

template<Jpeg::ImageMetaData::Format format>
static Image& grayscaleToRgb32(Image& dst, const Image& src)
{
  assert(src.m_metaData.m_format == Jpeg::ImageMetaData::Grayscale8 && format == dst.m_metaData.m_format);
  for (int y = 0; y < src.m_metaData.m_size.m_height; y++)
  {
    const uint8_t* srcLine = (const uint8_t*)src.scanline(y);
    uint32_t* dstLine = (uint32_t*)dst.scanline(y);

    for (int x = 0; x < src.m_metaData.m_size.m_width; x++)
      dstLine[x] = Jpeg::Rgb<format>::rgb(srcLine[x], srcLine[x], srcLine[x]);
  }
  return dst;
}

template<Jpeg::ImageMetaData::Format format>
static Image& rgb32ToGrayscale(Image& dst, const Image& src)
{
  using namespace Jpeg;
  assert(format == src.m_metaData.m_format && dst.m_metaData.m_format == Jpeg::ImageMetaData::Grayscale8);
  for(int y = 0; y < src.m_metaData.m_size.m_height; y++)
  {
    const uint32_t* srcLine = (const uint32_t*)src.scanline(y);
    uint8_t* dstLine = (uint8_t*)dst.scanline(y);

    for (int x = 0; x < src.m_metaData.m_size.m_width; x++)
      dstLine[x] = Rgb8ToYcc::rgbToY<format>(srcLine[x]);
  }
  return dst;
}

template<Jpeg::ImageMetaData::Format dstFormat, Jpeg::ImageMetaData::Format srcFormat>
static Image& rgb32ToRgb32(Image& dst, const Image& src)
{
  using namespace Jpeg;
  assert(src.m_metaData.m_format == srcFormat && dst.m_metaData.m_format == dstFormat);
  for (int y = 0; y < src.m_metaData.m_size.m_height; y++)
  {
    const uint32_t* srcLine = (const uint32_t*)src.scanline(y);
    uint32_t* dstLine = (uint32_t*)dst.scanline(y);

    for (int x = 0; x < src.m_metaData.m_size.m_width; x++)
      dstLine[x] = Rgb<dstFormat>::rgb(Rgb<srcFormat>::red(srcLine[x]), Rgb<srcFormat>::green(srcLine[x]), Rgb<srcFormat>::blue(srcLine[x]));
  }
  return dst;
}

Image Image::converted(Jpeg::ImageMetaData::Format format) const
{
  if (format == Jpeg::ImageMetaData::Invalid)
    return Image();
  if (format == m_metaData.m_format)
    return copy();

  Image image = Image::create(m_metaData.m_size.m_width, m_metaData.m_size.m_height, format, m_alignment);
  image.allocPixels(m_alignment);

  switch(m_metaData.m_format)
  {
  case Jpeg::ImageMetaData::Grayscale8:
    switch(format)
    {
    case Jpeg::ImageMetaData::Rgba32:
      return grayscaleToRgb32<Jpeg::ImageMetaData::Rgba32>(image, *this);
    case Jpeg::ImageMetaData::Bgra32:
      return grayscaleToRgb32<Jpeg::ImageMetaData::Bgra32>(image, *this);
    case Jpeg::ImageMetaData::Grayscale8:
    case Jpeg::ImageMetaData::Invalid:
      break;
    }
    break;
  case Jpeg::ImageMetaData::Rgba32:
    switch (format)
    {
    case Jpeg::ImageMetaData::Grayscale8:
      return rgb32ToGrayscale<Jpeg::ImageMetaData::Rgba32>(image, *this);
    case Jpeg::ImageMetaData::Bgra32:
      return rgb32ToRgb32<Jpeg::ImageMetaData::Bgra32, Jpeg::ImageMetaData::Rgba32>(image, *this);
    case Jpeg::ImageMetaData::Rgba32:
    case Jpeg::ImageMetaData::Invalid:
      break;
    }
    break;
  case Jpeg::ImageMetaData::Bgra32:
    switch (format)
    {
    case Jpeg::ImageMetaData::Grayscale8:
      return rgb32ToGrayscale<Jpeg::ImageMetaData::Bgra32>(image, *this);
    case Jpeg::ImageMetaData::Rgba32:
      return rgb32ToRgb32<Jpeg::ImageMetaData::Rgba32, Jpeg::ImageMetaData::Bgra32>(image, *this);
    case Jpeg::ImageMetaData::Bgra32:
    case Jpeg::ImageMetaData::Invalid:
      break;
    }
    break;
  case Jpeg::ImageMetaData::Invalid:
    break;
  }

  return Image();
}

static Image::Format assumeFileFormat(const std::string& fileName)
{
  size_t dotPos = fileName.rfind('.');
  if (dotPos == std::string::npos)
    return Image::Unknown;

  size_t pos = dotPos + 1;
  size_t extensionLength = fileName.length() - pos;
  if (fileName.compare(pos, extensionLength, "jpg") == 0 || fileName.compare(pos, extensionLength, "jpeg") == 0)
    return Image::Jpeg;
  if (fileName.compare(pos, extensionLength, "png") == 0)
    return Image::Png;

  return Image::Unknown;
}

template <size_t signatureSize>
bool checkSignature(std::filebuf& file, const char* signature)
{
  assert(signatureSize == strlen(signature));
  char signatureBuf[signatureSize];
  return file.sgetn(signatureBuf, signatureSize) == signatureSize && memcmp(signatureBuf, signature, signatureSize) == 0;
}

static bool isJpegFormat(std::filebuf& file)
{
  return checkSignature<2>(file, "\xff\xd8");
}

static bool isPngFormat(std::filebuf& file)
{
  return checkSignature<8>(file, "\x89\x50\x4E\x47\x0D\x0A\x1A\x0A");
}

static Image::Format detectFileFormat(std::filebuf& file)
{
  file.pubseekpos(0);
  if (isJpegFormat(file))
    return Image::Jpeg;
  if (isPngFormat(file))
    return Image::Png;

  return Image::Unknown;
}

static bool checkFileFormat(std::filebuf& file, Image::Format format)
{
  switch(format)
  {
  case Image::Jpeg:
    return isJpegFormat(file);
  case Image::Png:
    return isPngFormat(file);
  case Image::Unknown:
    return false;
  }

  assert(false);
  return false;
}

#ifdef WITH_LIBJPEG
namespace
{

struct JpegSourceManager : public jpeg_source_mgr
{
  static constexpr int BufferSize = 4096;

  std::filebuf* m_file;
  JOCTET m_buffer[BufferSize];

public:
  JpegSourceManager(std::filebuf* file);
};

static void initJpegSource(j_decompress_ptr)
{
}

static boolean fillJpegInputBuffer(j_decompress_ptr cinfo)
{
  JpegSourceManager* src = (JpegSourceManager*)cinfo->src;
  src->next_input_byte = src->m_buffer;
  int64_t num_read = src->m_file->sgetn((char*)src->m_buffer, JpegSourceManager::BufferSize);
  if (num_read <= 0)
  {
    // Insert a fake EOI marker - as per jpeglib recommendation
    src->next_input_byte = src->m_buffer;
    src->m_buffer[0] = (JOCTET)0xFF;
    src->m_buffer[1] = (JOCTET)JPEG_EOI;
    src->bytes_in_buffer = 2;
  }
  else
    src->bytes_in_buffer = num_read;

  return TRUE;
}

static void skipJpegInputData(j_decompress_ptr cinfo, long num_bytes)
{
  JpegSourceManager* src = (JpegSourceManager*)cinfo->src;

  if (num_bytes <= (long)src->bytes_in_buffer)
  {
    src->next_input_byte += (size_t)num_bytes;
    src->bytes_in_buffer -= (size_t)num_bytes;
  }
  else
  {
    src->m_file->pubseekoff(num_bytes, std::ios::cur, std::ios::in);
    src->bytes_in_buffer = 0;
  }
}

static void termJpegSource(j_decompress_ptr cinfo)
{
  JpegSourceManager* src = (JpegSourceManager*)cinfo->src;
  src->m_file->pubseekoff(src->bytes_in_buffer, std::ios::end, std::ios::in);
}

JpegSourceManager::JpegSourceManager(std::filebuf* file) : m_file(file)
{
  bytes_in_buffer = 0;
  next_input_byte = m_buffer;
  init_source = initJpegSource;
  fill_input_buffer = fillJpegInputBuffer;
  skip_input_data = skipJpegInputData;
  resync_to_restart = jpeg_resync_to_restart;
  term_source = termJpegSource;
}

}

static int alignedScanlineSize(int scanlineSize, int alignment)
{
  return scanlineSize > 0 ? (1 + (scanlineSize - 1) / alignment) * alignment : scanlineSize;
}
#endif

static Image loadJpegImage(std::filebuf& file, int rowAlignment)
{
#ifndef WITH_LIBJPEG
  (void)file;
  (void)rowAlignment;
  printf("reading jpeg format is not supported\n");
  return Image();
#else
  JpegSourceManager sourceManager(&file);
  jpeg_decompress_struct info;
  jpeg_error_mgr jerr;
  Image image;

  info.err = jpeg_std_error(&jerr);

  jpeg_create_decompress(&info);
  info.src = &sourceManager;

  jpeg_save_markers(&info, JPEG_COM, 0xFFFF);
  jpeg_save_markers(&info, JPEG_APP0 + 1, 0xFFFF); // Exif uses APP1 marker
  jpeg_save_markers(&info, JPEG_APP0 + 2, 0xFFFF); // ICC uses APP2 marker

  jpeg_read_header(&info, TRUE);
  jpeg_calc_output_dimensions(&info);

  Jpeg::ImageMetaData& metaData = image.m_metaData;
  metaData.m_size = Jpeg::Size{(int)info.output_width, (int)info.output_height};

  if (info.output_components == 1)
  {
    metaData.m_format = Jpeg::ImageMetaData::Grayscale8;
    metaData.m_scanlineBytes = alignedScanlineSize(info.output_width, rowAlignment);
  }
  else if (info.output_components == 3 || info.output_components == 4)
  {
    metaData.m_format = Jpeg::ImageMetaData::Rgba32;
    metaData.m_scanlineBytes = alignedScanlineSize(info.output_width * 4, rowAlignment);
  }
  else
  {
    assert(false);
    return Image();
  }

  image.allocPixels(rowAlignment);
  jpeg_start_decompress(&info);

  if (metaData.m_format == Jpeg::ImageMetaData::Grayscale8)
  {
    for (; info.output_scanline < info.output_height;)
    {
      char* row = image.scanline(info.output_scanline);
      jpeg_read_scanlines(&info, (unsigned char**)&row, 1);
    }
  }
  else
  {
    JSAMPARRAY rows = (info.mem->alloc_sarray)((j_common_ptr)&info, JPOOL_IMAGE, info.output_width * info.output_components, 1);

    for (; info.output_scanline < info.output_height;)
    {
      unsigned char (*dst)[4] = (unsigned char (*)[4])(image.m_pixels.get() + info.output_scanline * metaData.m_scanlineBytes);

      jpeg_read_scanlines(&info, rows, 1);

      if (info.output_components == 3)
      {
        unsigned char (*src)[3] = (unsigned char (*)[3])rows[0];
        for(int x = 0; x < metaData.m_size.m_width; x++)
        {
          dst[x][0] = src[x][0];
          dst[x][1] = src[x][1];
          dst[x][2] = src[x][2];
          dst[x][3] = 0xff;
        }
      }
      else if (info.out_color_space == JCS_CMYK)
      {
        // Convert CMYK->RGB.
        unsigned char (*src)[4] = (unsigned char (*)[4])rows[0];
        for (int x = 0; x < metaData.m_size.m_width; x++)
        {
          int k = src[x][3];

          dst[x][0] = (k * src[x][0]) / 255;
          dst[x][1] = (k * src[x][1]) / 255;
          dst[x][2] = (k * src[x][2]) / 255;
          dst[x][3] = 0xff;
        }
      }
    }
  }

  jpeg_finish_decompress(&info);
  jpeg_destroy_decompress(&info);

  return image;
#endif
}

#ifdef WITH_LIBPNG
static void pngReadFunction(png_structp png_ptr, png_bytep data, size_t count)
{
  std::filebuf& file = *(std::filebuf*)png_get_io_ptr(png_ptr);
  file.sgetn((char*)data, count);
}
#endif

static Image loadPngImage(std::filebuf& file, int rowAlignment)
{
#ifndef WITH_LIBPNG
  (void)file;
  (void)rowAlignment;
  printf("reading png format is not supported\n");
  return Image();
#else
  png_structp png_ptr;
  png_infop info_ptr;
  png_uint_32 width, height;
  int bit_depth, color_type, interlace_type;
  Image image;

  png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if (!png_ptr)
    return Image();

  info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr)
  {
    png_destroy_read_struct(&png_ptr, nullptr, nullptr);
    return Image();
  }

  if (setjmp(png_jmpbuf(png_ptr)))
  {
    /* Free all of the memory associated with the png_ptr and info_ptr. */
    png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
    /* If we get here, we had a problem reading the file. */
    return Image();
  }

  png_set_read_fn(png_ptr, (void*)&file, pngReadFunction);

  png_read_info(png_ptr, info_ptr);
  png_get_IHDR(png_ptr, info_ptr, &width, &height, &bit_depth, &color_type, &interlace_type, nullptr, nullptr);

  Jpeg::ImageMetaData& metaData = image.m_metaData;
  metaData.m_size = Jpeg::Size{(int)width, (int)height};

  if (bit_depth == 16)
    png_set_strip_16(png_ptr);
  png_set_packing(png_ptr);

  metaData.m_format = Jpeg::ImageMetaData::Bgra32;
  if (color_type == PNG_COLOR_TYPE_GRAY)
  {
    if (bit_depth < 8)
      png_set_expand_gray_1_2_4_to_8(png_ptr);
    metaData.m_format = Jpeg::ImageMetaData::Grayscale8;
  }
  if (color_type == PNG_COLOR_TYPE_PALETTE)
    png_set_palette_to_rgb(png_ptr);

  if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS) != 0)
    png_set_tRNS_to_alpha(png_ptr);

  double screen_gamma = PNG_DEFAULT_sRGB;
  int intent;

  if (png_get_sRGB(png_ptr, info_ptr, &intent) != 0)
    png_set_gamma(png_ptr, screen_gamma, PNG_DEFAULT_sRGB);
  else
  {
    double image_gamma;
    if (png_get_gAMA(png_ptr, info_ptr, &image_gamma) != 0)
      png_set_gamma(png_ptr, screen_gamma, image_gamma);
    else
      png_set_gamma(png_ptr, screen_gamma, 0.45455);
  }

  if ((color_type & PNG_COLOR_MASK_COLOR) != 0)
  {
    png_set_bgr(png_ptr);
    if (!png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS))
      png_set_filler(png_ptr, 0xffff, PNG_FILLER_AFTER);
  }

  png_read_update_info(png_ptr, info_ptr);

  metaData.m_scanlineBytes = alignedScanlineSize((int)png_get_rowbytes(png_ptr, info_ptr), rowAlignment);
  image.allocPixels(rowAlignment);

  png_bytep* row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * height);
  for (png_uint_32 row = 0; row < height; row++)
    row_pointers[row] = (png_bytep)image.scanline(row);

  png_read_image(png_ptr, row_pointers);
  png_read_end(png_ptr, info_ptr);

  free(row_pointers);
  png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);

  return image;
#endif
}

Image Image::load(const std::string& fileName, int alignment)
{
  Image::Format format = assumeFileFormat(fileName);
  std::filebuf file;

  if (!file.open(fileName, std::ios::in | std::ios::binary))
    return Image();

  if (format != Unknown && !checkFileFormat(file, format))
    format = Unknown;

  return load(file, format, alignment);
}

Image Image::load(std::filebuf& file, Format format, int alignment)
{
  if (file.pubseekpos(0, std::ios::in) == -1)
  {
    assert(false);
    return Image();
  }

  if (format == Unknown)
    format = detectFileFormat(file);
  if (format == Unknown)
    return Image();

  file.pubseekpos(0);
  switch (format)
  {
  case Image::Jpeg:
    return loadJpegImage(file, alignment);
  case Image::Png:
    return loadPngImage(file, alignment);
  case Image::Unknown:
    return Image();
  }

  assert(false);
  return Image();
}
