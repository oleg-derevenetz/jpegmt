#include "JpegHuffmanEncoder.h"

#include <Helper/Platform/Cpu/cpu.h>
#include <Helper/Platform/Cpu/intrinsics.h>
#include <Helper/Platform/Cpu/simd.h>

#include <Jpeg/JpegDCT.h>

#include "../Common.h"
#include "JpegHuffmanEncoderUtils.h"

#if defined(DEBUG) || defined(_DEBUG)
static int64_t totalBlockCount = 0;
static int64_t zeroAcBlockCount = 0;
static int64_t totalRunCount = 0;
#endif

namespace
{

struct BitCountTable
{
  uint8_t m_table[65536];

  BitCountTable()
  {
    auto zeroEntry = m_table + 32768;

    zeroEntry[0] = zeroEntry[-1] = 0;
    for (int i = 1; i < 32768; i++)
      zeroEntry[i] = zeroEntry[-i - 1] = Platform::Cpu::mostSignificantSetBit<uint32_t>((uint32_t)i) + 1;
  }

  int8_t valueBits(int value) const
  {
    return m_table[32768 + value];
  }
};

BitCountTable bitCountTable;

}

namespace Jpeg
{

namespace
{

template <bool nativeByteOrder>
struct RegisterBitsBuffer
{
  uint64_t* m_output;
  int64_t m_outputOffset;
  int m_freeBits;
  uint64_t m_bufferedBits;

  constexpr static int WordBits = 64;

  FORCE_INLINE RegisterBitsBuffer(uint64_t* output, int64_t outputOffset) :
    m_output(output), m_outputOffset(bitWord<uint64_t>(outputOffset)),
    m_freeBits(WordBits - wordBitNum<uint64_t>(outputOffset)), m_bufferedBits((nativeByteOrder ? output[m_outputOffset] : fromBigEndian(output[m_outputOffset])) >> m_freeBits)
  {
  }

  FORCE_INLINE void putBits(int64_t value, int32_t valueBits)
  {
    m_freeBits -= valueBits;
    if (m_freeBits >= 0)
    {
      m_bufferedBits = (m_bufferedBits << valueBits) | value;
      return;
    }

    uint64_t bufferedBits = (m_bufferedBits << (valueBits + m_freeBits)) | (value >> -m_freeBits);
    m_output[m_outputOffset++] = nativeByteOrder ? bufferedBits : toBigEndian(bufferedBits);
    m_freeBits += sizeof(m_bufferedBits) * 8;
    m_bufferedBits = value;
  }

  FORCE_INLINE void putCode(int32_t value, uint8_t valueBits, const HuffmanEncoder::LookupTable& lookupTable, uint8_t index)
  {
    int32_t code = lookupTable.getCode(index);
    int32_t codeBits = lookupTable.getSize(index);

//    value &= (((int32_t)1) << valueBits) - 1;
    value &= maskTable[valueBits];
    putBits(code | value, codeBits);
  }

  FORCE_INLINE void putCodePair(int32_t value0, int32_t value1, uint8_t value0Bits, uint8_t value1Bits,
    const HuffmanEncoder::LookupTable& lookupTable, uint8_t index0, uint8_t index1)
  {
    int32_t code0 = lookupTable.getCode(index0);
    int32_t code0Bits = lookupTable.getSize(index0);
    int32_t code1 = lookupTable.getCode(index1);
    int32_t code1Bits = lookupTable.getSize(index1);

//    value0 &= (((int32_t)1) << value0Bits) - 1;
    value0 &= maskTable[value0Bits];
//    value1 &= (((int32_t)1) << value1Bits) - 1;
    value1 &= maskTable[value1Bits];
    putBits(((int64_t)(value0 | code0) << (value1Bits + code1Bits)) | (value1 | code1), code0Bits + code1Bits);
  }

  FORCE_INLINE int64_t flush()
  {
    if (m_freeBits < 64)
      m_output[m_outputOffset] = nativeByteOrder ? (m_bufferedBits << m_freeBits) : toBigEndian(m_bufferedBits << m_freeBits);
    return (m_outputOffset + 1) * WordBits - m_freeBits;
  }
};

static constexpr int indexBitMask(int index, int w)
{
  return index > w * 16 && index <= (w + 1) * 16 ? 1 << (index - w * 16 - 1) : 0;
}

template <typename T, int w, int... I>
T makeAcBitMaskBits()
{
  return T::create((indexBitMask(I, w))...);
}

template <typename T>
struct AcBitMask
{
  T bits_0_15, bits_16_31, bits_32_47, bits_48_63;
};

template <typename T, int... I>
AcBitMask<T> makeAcBitMask()
{
  return AcBitMask<T>{makeAcBitMaskBits<T, 0, I...>(), makeAcBitMaskBits<T, 1, I...>(), makeAcBitMaskBits<T, 2, I...>(), makeAcBitMaskBits<T, 3, I...>()};
}

template <typename T, int bitWords>
void updateMaskBits(const AcBitMask<T>& bitMask, T acMask, T& bits0_15, T& bits16_31, T& bits32_47, T& bits48_63)
{
  using namespace Platform::Cpu;

  if (bitWords & 1)
    bits0_15 |= bitMask.bits_0_15.andNot(acMask);
  if (bitWords & 2)
    bits16_31 |= bitMask.bits_16_31.andNot(acMask);
  if (bitWords & 4)
    bits32_47 |= bitMask.bits_32_47.andNot(acMask);
  if (bitWords & 8)
    bits48_63 |= bitMask.bits_48_63.andNot(acMask);
}

}

template <int SimdLength> static uint64_t getAcMask(const int16_t* block, int16_t* dst);

static int32_t maskTable[16] = {
  0x0000, 0x0001, 0x0003, 0x0007,
  0x000f, 0x001f, 0x003f, 0x007f,
  0x00ff, 0x01ff, 0x03ff, 0x07ff,
  0x0fff, 0x1fff, 0x3fff, 0x7fff,
};

static int32_t acMask(uint8_t acbits)
{
  //  return (1 << acbits) - 1;
  return maskTable[acbits];
}

namespace
{
  struct AcCode
  {
    uint32_t code;
    int32_t size;
  };
}

static AcCode getAcHuffmanCode(const HuffmanEncoder::LookupTable& acTable, int16_t ac, uint8_t cntZeros)
{
  uint8_t acbits = bitCountTable.valueBits(ac);
  uint8_t index = 16 * cntZeros + acbits;

  return AcCode{acTable.getCode(index) | (ac & acMask(acbits)), acTable.getSize(index)};
}

//#define ZIGZAG_ORDER_AC

static int getAcIndex(int8_t bitIndex)
{
#ifdef ZIGZAG_ORDER_AC
  return bitIndex;
#else
  return Dct::m_zigzagOrder[bitIndex + 1];
#endif
}

template <int SimdLength>
int64_t HuffmanEncoder::encodeBlocks(const int16_t* block, const int* mcuComponents, int mcuBlockCount,
  const HuffmanEncoder* componentEncoders, const int* componentEncoderIndices, int* componentDc, int mcuCount,
  uint64_t* output, int64_t outputOffset, int64_t outputSizeInItems)
{
  (void)(outputSizeInItems); // TODO: check
  RegisterBitsBuffer<true> bitsBuffer(output, outputOffset);

  for (int mcuIndex = 0; mcuIndex < mcuCount; mcuIndex++)
  {
    for (int blockIndex = 0; blockIndex < mcuBlockCount; blockIndex++, block += Dct::BlockSize2)
    {
      int component = mcuComponents[blockIndex];
      const HuffmanEncoder& encoder = componentEncoders[componentEncoderIndices[component]];
      const LookupTable& dcTable = encoder.m_dcTable;
      const LookupTable& acTable = encoder.m_acTable;

      /* Encode the DC coefficient difference per section F.1.2.1 */
      int dc = block[0] - componentDc[component];
      componentDc[component] = block[0];
      dc += (dc >> (8 * sizeof(int32_t) - 1));
      uint8_t nbits = bitCountTable.valueBits(dc);

      bitsBuffer.putCode(dc, nbits, dcTable, nbits);

      /* Encode the AC coefficients per section F.1.2.2 */

      alignas(32) int16_t ac[64];
#ifdef ZIGZAG_ORDER_AC
      uint64_t mask = toZigzagOrder(block, ac);
#else
      uint64_t mask = getAcMask<SimdLength>(block, ac);
#endif
#if 0
      for (int i = 0; i < 63; i++)
      {
        Q_ASSERT(ac[i + 1] == block[i + 1] + (block[i + 1] >> 15));
        Q_ASSERT((bool)ac[Dct::m_zigzagOrder[i + 1]] == (bool)(mask & (1ull << i)));
        //        if ((bool)zigzag[i] != (bool)(mask & (1ull << i)))
        //          mask = toZigzagOrder(block, zigzag);
      }
      Q_ASSERT((mask & (1ull << 63)) == 0);
#endif
#if defined(DEBUG) || defined(_DEBUG)
      totalBlockCount++;
      if (!mask)
        zeroAcBlockCount++;
      totalRunCount += mask ? Platform::Cpu::popcnt<uint64_t>(mask) : 0;
#endif
      if (!mask)
      {
        bitsBuffer.putBits(acTable.getCode(0), acTable.getSize(0));
        continue;
      }

      //      alignas(32) int16_t aclen[64];
      //      getAcBitLength(ac, aclen);
#if 1
      int cntRuns = Platform::Cpu::popcnt<uint64_t>(mask);
      int8_t currRun = -1;
      //      int maxCodeSize = 0;
      AcCode acCodes[64];

      // TODO: solve msvc issue with tzcnt false output dependency and enable that branch
      if (0 && Platform::Cpu::mostSignificantSetBit<uint64_t>(mask) + 1 - cntRuns < 16)
      {
        for (int i = 0; i < cntRuns; i++) // 7 runs average for 75% quality, 53 runs average for 100% quality
        {
          int8_t runLength = Platform::Cpu::leastSignificantSetBit<uint64_t>(mask) + 1;
          mask >>= runLength;
          currRun += runLength;
          acCodes[i] = getAcHuffmanCode(acTable, ac[getAcIndex(currRun)], runLength - 1);
        }
        int i = 0;
        for (; i + 1 < cntRuns; i += 2)
          bitsBuffer.putBits(((int64_t)acCodes[i].code << acCodes[i + 1].size) | acCodes[i + 1].code, acCodes[i].size + acCodes[i + 1].size);
        if (i < cntRuns)
          bitsBuffer.putBits(acCodes[i].code, acCodes[i].size);
      }
      else
      {
        alignas(32) uint8_t runZeros[64]; // = {};
        int maxRun = 0;

        for (int i = 0; mask; i++) // 7 runs average for 75% quality, 53 runs average for 100% quality
        {
          int runLength = Platform::Cpu::leastSignificantSetBit<uint64_t>(mask) + 1;
          mask >>= runLength;
          currRun += runLength;
          runZeros[i] = runLength - 1;
          acCodes[i] = getAcHuffmanCode(acTable, ac[getAcIndex(currRun)], (runLength - 1) & 0xf);
          //          if (acCodes[i].size > maxCodeSize)
          //            maxCodeSize = acCodes[i].size;
          maxRun |= runLength - 1;
#if 0
          int nonzeroRunLength = Platform::Cpu::leastSignificantSetBit<uint64_t>(~mask);
          mask >>= nonzeroRunLength;
          for (int j = 0; j < nonzeroRunLength; j++)
          {
            runZeros[++i] = 0;
            acCodes[i] = getAcHuffmanCode(acTable, ac[getAcIndex(++currRun)], 0);
          }
#endif
        }
        //      if(!_mm256_movemask_epi8(_mm256_cmpgt_epi8(_mm256_or_si256(_mm256_load_si256((__m256i*)runZeros), _mm256_load_si256((__m256i*)runZeros + 1)), _mm256_set1_epi8(15))))
        if (maxRun < 16)
        {
          int i = 0;
#if 0
          if (maxCodeSize <= 16)
          {
            for (; i + 3 < cntRuns; i += 4)
            {
              int32_t size23 = acCodes[i + 2].size + acCodes[i + 3].size;
              bitsBuffer.putBits(((int64_t)((acCodes[i].code << acCodes[i + 1].size) | acCodes[i + 1].code) << size23) |
                (acCodes[i + 2].code << acCodes[i + 3].size) | acCodes[i + 3].code, acCodes[i].size + acCodes[i + 1].size + size23);
            }
            if (i + 1 < cntRuns)
            {
              bitsBuffer.putBits((acCodes[i].code << acCodes[i + 1].size) | acCodes[i + 1].code, acCodes[i].size + acCodes[i + 1].size);
              i += 2;
            }
            if (i < cntRuns)
              bitsBuffer.putBits(acCodes[i].code, acCodes[i].size);
          }
          else
#endif
          {
#if 1
            for (; i + 1 < cntRuns; i += 2)
              bitsBuffer.putBits(((int64_t)acCodes[i].code << acCodes[i + 1].size) | acCodes[i + 1].code, acCodes[i].size + acCodes[i + 1].size);
            if (i < cntRuns)
#else
            for (; i < cntRuns; i++)
#endif
              bitsBuffer.putBits(acCodes[i].code, acCodes[i].size);
          }
        }
        else
        {
          for (int i = 0; i < cntRuns; i++)
          {
            for (int cntZero = runZeros[i]; cntZero >= 16; cntZero -= 16)
              bitsBuffer.putBits(acTable.getCode(0xf0), acTable.getSize(0xf0));
            bitsBuffer.putBits(acCodes[i].code, acCodes[i].size);
          }
        }
      }

      /* If the last coef(s) were zero, emit an end-of-block code */
      if (currRun < 62)
        bitsBuffer.putBits(acTable.getCode(0), acTable.getSize(0));
#elif 1
      int cntRuns = Platform::Cpu::popcnt<uint64_t>(mask);
      int currRun = -1;

      for (int i = 0; i < cntRuns; i++) // 7 runs average
      {
        int runLength = Platform::Cpu::leastSignificantSetBit<uint64_t>(mask) + 1;
        currRun += runLength;
        mask >>= runLength;

        for (; runLength > 16; runLength -= 16)
          bitsBuffer.putBits(acTable.getCode(0xf0), acTable.getSize(0xf0));
        AcCode acCode = getAcHuffmanCode(acTable, ac[getAcIndex(currRun)], (runLength - 1) & 0xf);
        bitsBuffer.putBits(acCode.code, acCode.size);
      }
      if (currRun < 62)
        bitsBuffer.putBits(acTable.getCode(0), acTable.getSize(0));
#else
      int16_t* curr = zigzag;
      do
      {
        uint8_t cntZero = Platform::Cpu::leastSignificantSetBit<uint64_t>(mask);
        int ac = curr[cntZero];
        uint8_t acbits = bitCountTable.valueBits(ac);

        curr += cntZero + 1;
        mask >>= cntZero + 1;

        /* if run length > 15, must emit special run-length-16 codes (0xF0) */
        for (; cntZero >= 16; cntZero -= 16)
          bitsBuffer.putBits(acTable.getCode(0xf0), acTable.getSize(0xf0));

        /* Emit Huffman symbol for run length / number of bits */
        bitsBuffer.putCode(ac, acbits, acTable, 16 * cntZero + acbits);
      } while (mask);
      /* If the last coef(s) were zero, emit an end-of-block code */
      if (curr < zigzag + 63)
        bitsBuffer.putBits(acTable.getCode(0), acTable.getSize(0));
#endif
    }
  }

  return bitsBuffer.flush();
}

}
