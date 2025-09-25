#pragma once

#include <cstdint>
#include <vector>

namespace Jpeg
{

struct ImageMetaData;
struct HuffmanTable;
class OutputStream;
struct QuantizationTable;

class FileFormat
{
public:
  enum Marker : uint8_t
  {
    SOI = 0xD8, // Start of image
    SOF0 = 0xC0, // Start of frame (baseline)
    SOF2 = 0xC2, // Start of frame (progressive)
    DHT = 0xC4, // Define Huffman tables(s)
    DQT = 0xDB, // Define quantization tables(s)
    DRI = 0xDD, // Define restart interval
    SOS = 0xDA, // Start of scan
    RST0 = 0xD0, // Restart (+0..7)
    APP0 = 0xE0, // Application specific
    COM = 0xFE, // Comment
    EOI = 0xD9, // End of image
  };

  typedef std::vector<uint8_t> ByteSequence;

  static int64_t writeMarker(OutputStream* stream, Marker marker);
  static int64_t writeMarker(OutputStream* stream, Marker marker, const ByteSequence& data);
  static int64_t writeHuffmanTable(OutputStream* stream, int index, bool isAc, const HuffmanTable& table);
  static int64_t writeQuantizationTable(OutputStream* stream, int index, const QuantizationTable& table);
};

struct Jfif
{
  enum DensityUnit : uint8_t
  {
    DotsPerInch = 1,
    DotsPerCentimeter = 2,
  };

  DensityUnit m_densityUnit = DotsPerInch;
  uint16_t m_xDensity = 96;
  uint16_t m_yDensity = 96;

  FileFormat::ByteSequence toByteSequence() const;
  static Jfif fromImage(const ImageMetaData& imageMetaData);

  static constexpr uint8_t MajorVersion = 1;
  static constexpr uint8_t MinorVersion = 1;
};

struct FrameHeader
{
  struct ComponentInfo
  {
    uint8_t m_id = 0;
    uint8_t m_hSamplingFactor = 1;
    uint8_t m_vSamplingFactor = 1;
    uint8_t m_quantizationTableIndex = 0;

    ComponentInfo(uint8_t id, uint8_t hSamplingFactor, uint8_t vSamplingFactor, uint8_t quantizationTableIndex);
  };

  uint8_t m_precision = 8;
  uint16_t m_imageWidth = 0;
  uint16_t m_imageHeight = 0;
  std::vector<ComponentInfo> m_components;

  FrameHeader(uint8_t precision, uint16_t imageWidth, uint16_t imageHeight, const std::vector<ComponentInfo>& components);

  FileFormat::ByteSequence toByteSequence() const;
};

struct ScanHeader
{
  struct ComponentInfo
  {
    uint8_t m_id = 0;
    uint8_t m_dcHuffmanTableIndex = 0;
    uint8_t m_acHuffmanTableIndex = 0;

    ComponentInfo(uint8_t id, uint8_t dcHuffmanTableIndex, uint8_t acHuffmanTableIndex);
  };

  std::vector<ComponentInfo> m_components;
  uint8_t m_ss = 0;
  uint8_t m_se = 63;
  uint8_t m_ah = 0;
  uint8_t m_al = 0;

  ScanHeader(const std::vector<ComponentInfo>& components);

  FileFormat::ByteSequence toByteSequence() const;
};

}
