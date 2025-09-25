#pragma once

#include <cstdint>

namespace Jpeg
{

class Dct
{
public:

  static constexpr int BlockSizeLog2 = 3;
  static constexpr int BlockSize = 8;
  static constexpr int BlockSize2 = BlockSize * BlockSize;
  static int m_zigzagOrder[BlockSize2];
};

struct HuffmanTable
{
  uint8_t m_bits[17] = {};
  uint8_t m_values[256];
  int m_maxValue = 0;

  HuffmanTable();
  HuffmanTable(const uint8_t bits[17], const uint8_t* values, int maxValue);

  int valuesCount() const;

  static HuffmanTable defaultLuminanceDcTable();
  static HuffmanTable defaultLuminanceAcTable();
  static HuffmanTable defaultChrominanceDcTable();
  static HuffmanTable defaultChrominanceAcTable();
};

struct QuantizationTable
{
  uint16_t m_table[Dct::BlockSize2] = {};

  QuantizationTable();
  QuantizationTable(const uint16_t table[Dct::BlockSize2]);

  bool isValid() const;
  uint16_t maxValue() const;
  QuantizationTable scaled(int scale, int valueLimit = 255 /* baseline range limit */) const;

  static QuantizationTable defaultLuminanceTable();
  static QuantizationTable defaultChrominanceTable();
};

}
