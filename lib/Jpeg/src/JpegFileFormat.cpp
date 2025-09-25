#include "JpegFileFormat.h"

#include <cassert>
#include <cmath>

#include <Jpeg/JpegDCT.h>
#include <Jpeg/JpegImageMetaData.h>
#include <Jpeg/JpegWriter.h>

namespace Jpeg
{

int64_t FileFormat::writeMarker(OutputStream* stream, Marker marker)
{
  uint8_t bytes[2] = {0xFF, marker};
  if (stream->writeJpegBytes((char*)bytes, sizeof(bytes)) != sizeof(bytes))
    return 0;

  return sizeof(bytes);
}

int64_t FileFormat::writeMarker(OutputStream* stream, Marker marker, const ByteSequence& data)
{
  size_t length = data.size() + 2;
  assert(length <= 0xffffu);

  uint8_t bytes[4] = {0xFF, marker, (uint8_t)((length >> 8) & 0xff), (uint8_t)(length & 0xff)};
  if (stream->writeJpegBytes((char*)bytes, sizeof(bytes)) != sizeof(bytes))
    return 0;
  if (stream->writeJpegBytes((char*)data.data(), data.size()) != data.size())
    return 0;

  return sizeof(bytes) + data.size();
}

int64_t FileFormat::writeHuffmanTable(OutputStream* stream, int index, bool isAc, const HuffmanTable& table)
{
  int valuesCount = table.valuesCount();
  uint16_t length = 16 + valuesCount + 2 + 1;
  uint8_t header[] = {0xFF, DHT, (uint8_t)((length >> 8) & 0xff), (uint8_t)(length & 0xff), (uint8_t)(index + (isAc ? 16 : 0))};

  if (stream->writeJpegBytes((char*)header, sizeof(header)) != sizeof(header))
    return 0;
  if (stream->writeJpegBytes((char*)table.m_bits + 1, 16) != 16)
    return 0;
  if (stream->writeJpegBytes((char*)table.m_values, valuesCount) != valuesCount)
    return 0;

  return sizeof(header) + 16 + valuesCount;
}

int64_t FileFormat::writeQuantizationTable(OutputStream* stream, int index, const QuantizationTable& table)
{
  bool is16bit = table.maxValue() > 255;
  uint16_t dataSize = Dct::BlockSize2 * (is16bit ? 2 : 1);
  uint16_t length = dataSize + 2 + 1;
  uint8_t header[] = {0xFF, DQT, (uint8_t)((length >> 8) & 0xff), (uint8_t)(length & 0xff), (uint8_t)(index + (is16bit ? 16 : 0))};

  if (stream->writeJpegBytes((char*)header, sizeof(header)) != sizeof(header))
    return 0;

  uint8_t data[Dct::BlockSize2 * 2], *p = data;
  for(int i = 0; i < Dct::BlockSize2; i++)
  {
    uint16_t value = table.m_table[Dct::m_zigzagOrder[i]];
    if (is16bit)
      *p++ = (value >> 8) & 0xff;
    *p++ = value & 0xff;
  }

  if (stream->writeJpegBytes((char*)data, dataSize) != dataSize)
    return 0;

  return sizeof(header) + dataSize;
}

FileFormat::ByteSequence Jfif::toByteSequence() const
{
  return FileFormat::ByteSequence{
    'J', 'F', 'I', 'F', 0,
    MajorVersion, MinorVersion,
    m_densityUnit,
    (uint8_t)((m_xDensity >> 8) & 0xff), (uint8_t)(m_xDensity & 0xff),
    (uint8_t)((m_yDensity >> 8) & 0xff), (uint8_t)(m_yDensity & 0xff),
    0, 0
    };
}

Jfif Jfif::fromImage(const ImageMetaData& imageMetaData)
{
  Jfif jfif;

  constexpr double cmPerInch = 2.54;
  double xDotsPerInch = imageMetaData.m_dotsPerMeterHorizontal * cmPerInch / 100.;
  double yDotsPerInch = imageMetaData.m_dotsPerMeterVertical * cmPerInch / 100.;
  double diffInch = std::abs(xDotsPerInch - round(xDotsPerInch)) + std::abs(yDotsPerInch - round(yDotsPerInch));
  double diffCm = (std::abs(imageMetaData.m_dotsPerMeterHorizontal / 100. - round(imageMetaData.m_dotsPerMeterHorizontal / 100.))
    + std::abs(imageMetaData.m_dotsPerMeterVertical / 100. - round(imageMetaData.m_dotsPerMeterVertical / 100.))) * cmPerInch;
  if (diffInch < diffCm) {
    jfif.m_densityUnit = DotsPerInch;
    jfif.m_xDensity = (uint16_t)lround(xDotsPerInch);
    jfif.m_yDensity = (uint16_t)lround(yDotsPerInch);
  }
  else {
    jfif.m_densityUnit = DotsPerCentimeter;
    jfif.m_xDensity = (imageMetaData.m_dotsPerMeterHorizontal + 50) / 100;
    jfif.m_yDensity = (imageMetaData.m_dotsPerMeterVertical + 50) / 100;
  }

  return jfif;
}

FrameHeader::ComponentInfo::ComponentInfo(uint8_t id, uint8_t hSamplingFactor, uint8_t vSamplingFactor, uint8_t quantizationTableIndex) :
  m_id(id), m_hSamplingFactor(hSamplingFactor), m_vSamplingFactor(vSamplingFactor), m_quantizationTableIndex(quantizationTableIndex)
{
}

FrameHeader::FrameHeader(uint8_t precision, uint16_t imageWidth, uint16_t imageHeight, const std::vector<ComponentInfo>& components) :
  m_precision(precision), m_imageWidth(imageWidth), m_imageHeight(imageHeight), m_components(components)
{
}

FileFormat::ByteSequence FrameHeader::toByteSequence() const
{
  FileFormat::ByteSequence data{
    m_precision,
    (uint8_t)((m_imageHeight >> 8) & 0xff), (uint8_t)(m_imageHeight & 0xff),
    (uint8_t)((m_imageWidth >> 8) & 0xff), (uint8_t)(m_imageWidth & 0xff),
    (uint8_t)m_components.size()
    };

  for(int i = 0; i < m_components.size(); i++)
  {
    const ComponentInfo& c = m_components.at(i);
    data.insert(data.end(), {c.m_id, (uint8_t)(c.m_hSamplingFactor * 16 + c.m_vSamplingFactor), c.m_quantizationTableIndex});
  }

  return data;
}

ScanHeader::ComponentInfo::ComponentInfo(uint8_t id, uint8_t dcHuffmanTableIndex, uint8_t acHuffmanTableIndex) :
  m_id(id), m_dcHuffmanTableIndex(dcHuffmanTableIndex), m_acHuffmanTableIndex(acHuffmanTableIndex)
{
}

ScanHeader::ScanHeader(const std::vector<ComponentInfo>& components) : m_components(components)
{
}

FileFormat::ByteSequence ScanHeader::toByteSequence() const
{
  FileFormat::ByteSequence data{(uint8_t)m_components.size()};

  for (int i = 0; i < m_components.size(); i++)
  {
    const ComponentInfo& c = m_components.at(i);
    data.insert(data.end(), {c.m_id, (uint8_t)(c.m_dcHuffmanTableIndex * 16 + c.m_acHuffmanTableIndex)});
  }
  data.insert(data.end(), {m_ss, m_se, (uint8_t)(m_ah * 16 + m_al)});

  return data;
}

}
