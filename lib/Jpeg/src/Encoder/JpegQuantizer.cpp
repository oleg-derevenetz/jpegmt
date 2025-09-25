#include "JpegQuantizer.h"

#include <Helper/Platform/Cpu/intrinsics.h>

namespace Jpeg
{

template<typename T, typename T2>
T computeReciprocal(uint16_t divisor, int16_t& correction, int16_t& shift)
{
  if (divisor == 1)
  {
    correction = 0;
    shift = -(int16_t)(sizeof(T) * 8);
    return 1;
  }

  int b = Platform::Cpu::mostSignificantSetBit<uint32_t>(divisor);
  int r = sizeof(T) * 8 + b;

  T2 fq = ((T2)1 << r) / divisor;
  T2 fr = ((T2)1 << r) % divisor;

  T c = divisor / 2;                      /* for rounding */

  if (fr == 0)
  {                        /* divisor is power of two */
    /* fq will be one bit too large to fit in DCTELEM, so adjust */
    fq >>= 1;
    r--;
  }
  else if (fr <= (divisor / 2U))    /* fractional part is < 0.5 */
    c++;
  else                              /* fractional part is > 0.5 */
    fq++;

  correction = c; /* correction + roundfactor */
  shift = (int16_t)(r - sizeof(T) * 8);
  return (T)fq;
}

Quantizer::Quantizer(const QuantizationTable& quantizationTable)
{
  for(int i = 0; i < Dct::BlockSize2; i++)
  {
    Divisor& item = m_divisors[i];
    uint16_t divisor = quantizationTable.m_table[i] << 3;
    int16_t shift;

    if (quantizationTable.m_table[i] != 1)
      m_isUnit = false;

    item.m_reciprocal = computeReciprocal<uint32_t, uint64_t>(divisor, item.m_correction, item.m_shift);
    m_reciprocal16[i] = computeReciprocal<uint16_t, uint32_t>(divisor, m_correction16[i], shift);
    m_scale16[i] = 1 << (sizeof(int16_t) * 8 - shift);

    for(int j = 0; j < 16; j++)
    {
      m_int16x16table[i][0][j] = m_reciprocal16[i];
      m_int16x16table[i][1][j] = m_correction16[i];
      m_int16x16table[i][2][j] = m_scale16[i];
    }

    for (int j = 0; j < 8; j++)
    {
      m_uint32x8table[i][0][j] = item.m_reciprocal;
      m_uint32x8table[i][1][j] = item.m_correction;
    }

    if (shift <= 0)
      m_haveNonPositiveShift = true;

//    m_storingOrder[Dct::m_zigzagOrder[i]] = i;
  }
}

#if 0
static int32_t conditionalNegate(int32_t value, bool negate)
{
  return negate ? -value : value;
}

void Quantizer::perform(const EncoderBuffer& buffer, int c, int16_t (*dst)[Dct::BlockSize2], int count) const
{
  constexpr int SimdSize = EncoderBuffer::SimdLength;
  int componentBlocks = buffer.m_componentBlockCount;

  const EncoderBuffer::Component& component = buffer.m_components.at(c);

  if (component.m_bufferCount >= SimdSize)
  {
    int simdBuffersPerMcu = component.m_bufferCount / SimdSize;
    for (int i = 0; i < count; i++)
    {
      for (int j = 0; j < simdBuffersPerMcu; j++)
      {
        int srcOffset = component.m_bufferOffset + i * simdBuffersPerMcu + j;
        int dstOffset = component.m_bufferOffset + i * componentBlocks + j * SimdSize;
        const int32_t (*values)[SimdSize] = (int32_t (*)[SimdSize])buffer.m_componentBuffers[srcOffset];

        for (int k = 0; k < Dct::BlockSize2; k++)
        {
          const Divisor& divisor = m_divisors[k];

          for (int n = 0; n < SimdSize; n++)
          {
            int32_t temp = values[k][n];
            bool negate = temp < 0;

            temp = conditionalNegate(temp, negate);
            uint64_t product = (uint64_t)(temp + divisor.m_correction) * divisor.m_reciprocal;
            product >>= divisor.m_shift + sizeof(int32_t) * 8;
            temp = conditionalNegate((int32_t)product, negate);

            dst[dstOffset + n][k] = (int16_t)temp;
          }
        }
      }
    }
  }
  else
  {
    int simdBuffers = 1 + (count * component.m_bufferCount - 1) / SimdSize;
    int mcuPerSimdBuffers = SimdSize / component.m_bufferCount;
#if 0
    for (int i = 0; i < simdBuffers; i++)
    {
      int srcOffset = component.m_bufferOffset + i;
      const int32_t (*values)[SimdSize] = (int32_t (*)[SimdSize])buffer.m_componentBuffers[srcOffset];

      int mcuCount = qMin(mcuPerSimdBuffers, count * component.m_bufferCount - i * mcuPerSimdBuffers);
      for (int j = 0; j < mcuCount; j++)
      {
        int dstOffset = component.m_bufferOffset + (i * mcuPerSimdBuffers + j) * componentBlocks;

        for (int k = 0; k < Dct::BlockSize2; k++)
        {
          const Divisor& divisor = m_divisors[k];

          for (int n = 0; n < component.m_bufferCount; n++)
          {
            int32_t temp = values[k][j*component.m_bufferCount + n];
            bool negate = temp < 0;

            temp = conditionalNegate(temp, negate);
            uint64_t product = (uint64_t)(temp + divisor.m_correction) * divisor.m_reciprocal;
            product >>= divisor.m_shift + sizeof(int32_t) * 8;
            temp = conditionalNegate((int32_t)product, negate);

            dst[dstOffset + n][k] = (int16_t)temp;
          }
        }
      }
    }
#else
    int dstOffsets[SimdSize];
    for (int i = 0; i < mcuPerSimdBuffers; i++)
    {
      for (int j = 0; j < component.m_bufferCount; j++)
        dstOffsets[i * component.m_bufferCount + j] = i * componentBlocks + j;
    }

    for (int i = 0; i < simdBuffers; i++)
    {
      int srcOffset = component.m_bufferOffset + i;
      int dstOffset = component.m_bufferOffset + i * mcuPerSimdBuffers * componentBlocks;
      const int32_t (*values)[SimdSize] = (int32_t(*)[SimdSize])buffer.m_componentBuffers[srcOffset];
      int16_t (*dsti)[Dct::BlockSize2] = dst + dstOffset;

#if 1
      for (int k = 0; k < Dct::BlockSize2; k++)
      {
#if 0
        const Divisor& divisor = m_divisors[k];

        for (int n = 0; n < SimdSize; n++)
        {
          int32_t temp = values[k][n];
          bool negate = temp < 0;

          temp = conditionalNegate(temp, negate);
          uint64_t product = (uint64_t)(temp + divisor.m_correction) * divisor.m_reciprocal;
          product >>= divisor.m_shift + sizeof(int32_t) * 8;
          temp = conditionalNegate((int32_t)product, negate);

          dsti[dstOffsets[n]][k] = (int16_t)temp;
        }
#else
/*
        using namespace Platform::Cpu;
        typedef SIMD<int32_t, SimdSize> SimdHelper;

        SimdType<SimdSize> value = SimdHelper::load(values[k]);
        SimdType<SimdSize> hi, zero = SimdHelper::populate(0);
        SimdHelper::ConditionType negate = value < zero;
        value = SimdHelper::select(zero, value, negate) - SimdHelper::select(value, zero, negate);
        SimdHelper::mulUnsigned(value + SimdHelper::populate(divisor.m_correction), SimdHelper::populate(divisor.m_reciprocal), hi);
        value = SimdHelper::logicalRightShift(hi, divisor.m_shift);
        value = SimdHelper::select(zero, value, negate) - SimdHelper::select(value, zero, negate);

        alignas(16) int32_t temp[SimdSize];
        SimdHelper::store(temp, value);
*/
        for (int n = 0; n < SimdSize; n++)
          dsti[dstOffsets[n]][m_storingOrder[k]] = (int16_t)values[k][n];
#endif
      }
#else
      for (int m = 0; m < Dct::BlockSize2 / SimdSize; m++)
      {
        alignas(16) int32_t matrix[SimdSize][SimdSize];

        for (int p = 0; p < SimdSize; p++)
        {
          int k = m * SimdSize + p;
          const Divisor& divisor = m_divisors[k];

          for (int n = 0; n < SimdSize; n++)
          {
            int32_t temp = values[k][n];
            bool negate = temp < 0;

            temp = conditionalNegate(temp, negate);
            uint64_t product = (uint64_t)(temp + divisor.m_correction) * divisor.m_reciprocal;
            product >>= divisor.m_shift + sizeof(int32_t) * 8;
            temp = conditionalNegate((int32_t)product, negate);

            matrix[p][n] = temp;
          }
        }

        typedef Platform::Cpu::SIMD<int32_t, 8> SimdHelper;
        SimdHelper::transpose((typename SimdHelper::Type*)matrix[0]);
        for (int n = 0; n < SimdSize; n++)
        {
          for (int k = 0; k < SimdSize; k++)
            dsti[dstOffsets[n]][m * SimdSize + k] = (int16_t)matrix[n][k];
        }
      }
#endif
    }
#endif
  }
}
#endif

}
