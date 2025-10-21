#include "JpegLibWrapper.h"

#include <cmath>
#include <cstdio>

#include <Jpeg/JpegImageMetaData.h>
#include <Jpeg/JpegWriter.h>

#include <jpeglib.h>

struct JpegLibOutputStreamManager : public jpeg_destination_mgr
{
  static constexpr int BufferSize = 4096;

  Jpeg::OutputStream* m_stream = nullptr;
  JOCTET m_buffer[BufferSize];

public:
  JpegLibOutputStreamManager(Jpeg::OutputStream* stream);
};

extern "C" {

  static void jpeglib_init_destination(j_compress_ptr)
  {
  }

  static boolean jpeglib_empty_output_buffer(j_compress_ptr cinfo)
  {
    JpegLibOutputStreamManager* mgr = (JpegLibOutputStreamManager*)cinfo->dest;

    int64_t bytesWritten = mgr->m_stream->writeJpegBytes((char*)mgr->m_buffer, JpegLibOutputStreamManager::BufferSize);
    (void)(bytesWritten);
    //  if (bytesWritten < 0)
    //    (*cinfo->err->error_exit)((j_common_ptr)cinfo);

    mgr->next_output_byte = mgr->m_buffer;
    mgr->free_in_buffer = JpegLibOutputStreamManager::BufferSize;

    return TRUE;
  }

  static void jpeglib_term_destination(j_compress_ptr cinfo)
  {
    JpegLibOutputStreamManager* mgr = (JpegLibOutputStreamManager*)cinfo->dest;
    int64_t rest = JpegLibOutputStreamManager::BufferSize - mgr->free_in_buffer;

    int64_t bytesWritten = mgr->m_stream->writeJpegBytes((char*)mgr->m_buffer, rest);
    (void)(bytesWritten);
    //  if (bytesWritten == -1)
    //    (*cinfo->err->error_exit)((j_common_ptr)cinfo);
  }

}

inline JpegLibOutputStreamManager::JpegLibOutputStreamManager(Jpeg::OutputStream* stream) : m_stream(stream)
{
  jpeg_destination_mgr::init_destination = jpeglib_init_destination;
  jpeg_destination_mgr::empty_output_buffer = jpeglib_empty_output_buffer;
  jpeg_destination_mgr::term_destination = jpeglib_term_destination;
  next_output_byte = m_buffer;
  free_in_buffer = BufferSize;
}

bool JpegLibWritter::setQuality(int quality)
{
  if (quality < 1 || quality > 100)
    return false;

  m_quality = quality;
  return true;
}

bool JpegLibWritter::write(const Jpeg::ImageMetaData& imageMetaData, const uint8_t* pixels)
{
  JpegLibOutputStreamManager streamManager(m_stream);
  jpeg_compress_struct cinfo;
  jpeg_error_mgr jerr;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  cinfo.dest = &streamManager;

  cinfo.image_width = imageMetaData.m_size.m_width;
  cinfo.image_height = imageMetaData.m_size.m_height;

  switch (imageMetaData.m_format)
  {
  case Jpeg::ImageMetaData::Grayscale8:
    cinfo.input_components = 1;
    cinfo.in_color_space = JCS_GRAYSCALE;
    break;
  case Jpeg::ImageMetaData::Rgba32:
    cinfo.input_components = 4;
    cinfo.in_color_space = JCS_EXT_RGBA;
    break;
  case Jpeg::ImageMetaData::Bgra32:
    cinfo.input_components = 4;
    cinfo.in_color_space = JCS_EXT_BGRA;
    break;
  default:
    return false;
  }

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, m_quality, false);

  double dpcx = imageMetaData.m_dotsPerMeterHorizontal / 100.0;
  double dpcy = imageMetaData.m_dotsPerMeterVertical / 100.0;
  double dpix = dpcx * 2.54;
  double dpiy = dpcy * 2.54;

  double diffInch = std::abs(dpix - round(dpix)) + std::abs(dpiy - round(dpiy));
  double diffCm = (std::abs(dpcx - round(dpcx)) + std::abs(dpcy - round(dpcy))) * 2.54;
  if (diffInch < diffCm) {
    cinfo.density_unit = 1; // dots/inch
    cinfo.X_density = (uint16_t)round(imageMetaData.m_dotsPerMeterHorizontal * 2.54 / 100.);
    cinfo.Y_density = (uint16_t)round(imageMetaData.m_dotsPerMeterVertical * 2.54 / 100.);
  }
  else {
    cinfo.density_unit = 2; // dots/cm
    cinfo.X_density = (imageMetaData.m_dotsPerMeterHorizontal + 50) / 100;
    cinfo.Y_density = (imageMetaData.m_dotsPerMeterVertical + 50) / 100;
  }

  jpeg_start_compress(&cinfo, TRUE);

  for (JSAMPROW row = (unsigned char*)pixels; (int)cinfo.next_scanline < imageMetaData.m_size.m_height; row += imageMetaData.m_scanlineBytes)
    jpeg_write_scanlines(&cinfo, &row, 1);

  jpeg_finish_compress(&cinfo);

  return true;
}

