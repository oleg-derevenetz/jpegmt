#include "JpegHuffmanEncoder.h"

#include <cassert>

#include <Helper/Platform/Cpu/cpu.h>
#include <Helper/Platform/Cpu/intrinsics.h>
#include <Helper/Platform/Cpu/simd.h>

#include <Jpeg/JpegDCT.h>

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

#if defined(DEBUG) || defined(_DEBUG)
int64_t totalBlockCount = 0;
int64_t zeroAcBlockCount = 0;
int64_t totalRunCount = 0;
#endif

namespace Jpeg
{

int HuffmanEncoder::m_simdLength = Platform::Cpu::SimdDetector<int16_t>::maxSimdLength();
int HuffmanEncoder::m_byteSimdLength = Platform::Cpu::SimdDetector<int8_t>::maxSimdLength();

HuffmanEncoder::LookupTable::LookupTable()
{
}

HuffmanEncoder::LookupTable::LookupTable(const HuffmanTable& huffmanTable, Type type)
{
  /* Note that huffsize[] and huffcode[] are filled in code-length order,
   * paralleling the order of the symbols themselves in htbl->huffval[].
   */

  /* Figure C.1: make table of Huffman code length for each symbol */

  int p = 0;
  char huffsize[257];
  for (int len = 1; len <= 16; len++)
  {
    int i = (int)huffmanTable.m_bits[len];
    assert(i >= 0 && p + i <= 256);
    if (i < 0 || p + i > 256)   /* protect against table overrun */
      return;
    while (i--)
      huffsize[p++] = (char)len;
  }
  huffsize[p] = 0;
  int lastp = p;

  /* Figure C.2: generate the codes themselves */
  /* We also validate that the counts represent a legal Huffman code tree. */

  uint32_t code = 0;
  uint32_t huffcode[257];
  for (int p = 0, si = huffsize[0]; huffsize[p]; code <<= 1, si++)
  {
    for (;(int)huffsize[p] == si; p++, code++)
      huffcode[p] = code;

    /* code is now 1 more than the last code used for codelength si; but
     * it must still fit in si bits, since no code is allowed to be all ones.
     */
    assert(code < (1u << si));
    if (code >= (1u << si))
      return;
  }

  /* Figure C.3: generate encoding tables */
  /* These are code and size indexed by symbol value */

  /* This is also a convenient place to check for out-of-range
   * and duplicated VAL entries.  We allow 0..255 for AC symbols
   * but only 0..15 for DC.  (We could constrain them further
   * based on data depth and mode, but this seems enough.)
   */

  for (int p = 0; p < lastp; p++)
  {
    int i = huffmanTable.m_values[p];
    assert(i >= 0 && i <= huffmanTable.m_maxValue && !getSize(i));
    if (i < 0 || i > huffmanTable.m_maxValue || getSize(i))
      return;

    int valueBits = type == Ac ? i & 0xf : i;
    setCode(i, huffcode[p] << valueBits, huffsize[p] + valueBits);
  }

  m_isValid = true;
}

HuffmanEncoder::HuffmanEncoder(const HuffmanTable& dcTable, const HuffmanTable& acTable) :
  m_dcTable(dcTable, LookupTable::Dc), m_acTable(acTable, LookupTable::Ac)
{
}

#ifndef FORCE_INLINE
#ifdef PLATFORM_COMPILER_MSVC
#define FORCE_INLINE __forceinline
#elif defined(PLATFORM_COMPILER_GNU)
#define FORCE_INLINE __attribute__((always_inline))
#else
#define FORCE_INLINE
#endif
#endif

FORCE_INLINE static inline uint64_t toBigEndian(uint64_t source)
{
  return Platform::Cpu::byteOrder == Platform::Cpu::BigEndian ? source :
      ((source & 0x00000000000000ffULL) << 56)
    | ((source & 0x000000000000ff00ULL) << 40)
    | ((source & 0x0000000000ff0000ULL) << 24)
    | ((source & 0x00000000ff000000ULL) << 8)
    | ((source & 0x000000ff00000000ULL) >> 8)
    | ((source & 0x0000ff0000000000ULL) >> 24)
    | ((source & 0x00ff000000000000ULL) >> 40)
    | ((source & 0xff00000000000000ULL) >> 56);
}

FORCE_INLINE static inline uint64_t fromBigEndian(uint64_t source)
{
  return toBigEndian(source);
}

constexpr static int getPowerOf2(int n)
{
  return n == 0 || (n > 1 && (n & 1)) ? -1 : (n == 1 ? 0 : (getPowerOf2(n >> 1) == -1 ? -1 : getPowerOf2(n >> 1) + 1));
}

template <typename T>
constexpr static int64_t bitWord(int64_t bitIndex)
{
  constexpr int WordBitsPowerOf2 = getPowerOf2(sizeof(T) * 8);
  return bitIndex >> WordBitsPowerOf2;
}
template <typename T>
constexpr static int wordBitNum(int64_t bitIndex)
{
  constexpr int WordBitsPowerOf2 = getPowerOf2(sizeof(T) * 8);
  return bitIndex & ((1LL << WordBitsPowerOf2) - 1);
}

static int32_t maskTable[16] = {
  0x0000, 0x0001, 0x0003, 0x0007,
  0x000f, 0x001f, 0x003f, 0x007f,
  0x00ff, 0x01ff, 0x03ff, 0x07ff,
  0x0fff, 0x1fff, 0x3fff, 0x7fff,
};

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

int64_t HuffmanEncoder::encode(const int16_t* block, int prevDcValue, uint64_t* output, int64_t outputOffset, int64_t outputSizeInItems) const
{
  int mcuComponents = 0, encoderIndex = 0;
  return encode(block, &mcuComponents, 1, this, &encoderIndex, &prevDcValue, 1, output, outputOffset, outputSizeInItems);
}

#if 0
typedef uint64_t BlockWord;
constexpr static BlockWord FilledWord = 0xffffffffffffffffULL;

#if 0
template <typename ItemType, int startIndex, int blockWordItems = sizeof(BlockWord) / sizeof(ItemType)>
FORCE_INLINE static int encodeAcWord(BlockWord word, int r, const HuffmanEncoder::LookupTable& acTable, RegisterBitsBuffer& bitsBuffer)
{
  constexpr int itemBits = sizeof(ItemType) * 8;

  if (word == 0)
    return r + 16 * (blockWordItems - startIndex);

  if (blockWordItems == 1)
  {
    if (startIndex == 0)
    {
      int32_t ac = (ItemType)(word & ((1 << itemBits) - 1));
      /* Branch-less absolute value, bitwise complement, etc., same as above */
      int32_t sign = ac >> (CHAR_BIT * sizeof(int32_t) - 1);
      int32_t nbits = (ac + sign) ^ sign;
      nbits = Platform::Cpu::mostSignificantSetBit<uint32_t>((uint32_t)nbits) + 1;

      /* if run length > 15, must emit special run-length-16 codes (0xF0) */
      for (; r >= 16 * 16; r -= 16 * 16)
        bitsBuffer.putBits(acTable.m_codeTable[0xf0], acTable.m_sizeTable[0xf0]);

      /* Emit Huffman symbol for run length / number of bits */
      bitsBuffer.putCode(ac + sign, nbits, acTable, r + nbits);
    }
    return 0;
  }

  constexpr int loIndex = Platform::Cpu::byteOrder == Platform::Cpu::BigEndian ? 1 : 0;
  constexpr int hiwIndex = Platform::Cpu::byteOrder == Platform::Cpu::LowEndian ? 1 : 0;
  BlockWord lo = ((word >> (loIndex * (itemBits * blockWordItems / 2))) & (((BlockWord)1 << (itemBits * blockWordItems / 2)) - 1));
  BlockWord hi = ((word >> (hiIndex * (itemBits * blockWordItems / 2))) & (((BlockWord)1 << (itemBits * blockWordItems / 2)) - 1));

  r = encodeAcWord<ItemType, startIndex, blockWordItems / 2>(lo, r, acTable, bitsBuffer);
  r = encodeAcWord<ItemType, 0, blockWordItems / 2>(hi, r, acTable, bitsBuffer);

  return r;
}
#else
template <typename ItemType, int startIndex>
FORCE_INLINE static int encodeAcWord(BlockWord word, int r, const HuffmanEncoder::LookupTable& acTable, RegisterBitsBuffer<true>& bitsBuffer)
{
  constexpr int itemBits = sizeof(ItemType) * 8;
  constexpr BlockWord itemMask = ((BlockWord)1 << itemBits) - 1;
  constexpr int blockWordItems = sizeof(BlockWord) / sizeof(ItemType);

  if (word == 0)
    return r + 16 * (blockWordItems - startIndex);

  int startItemIndex = startIndex;
  for (; word;)
  {
    int nonZeroItemIndex = Platform::Cpu::leastSignificantSetBit<BlockWord>(word) / itemBits;
    int32_t ac = (ItemType)((word >> (nonZeroItemIndex * itemBits)) & itemMask);
    /* Branch-less absolute value, bitwise complement, etc., same as above */
    int32_t sign = ac >> (CHAR_BIT * sizeof(int32_t) - 1);
    int32_t nbits = (ac + sign) ^ sign;
    nbits = Platform::Cpu::mostSignificantSetBit<uint32_t>((uint32_t)nbits) + 1;

    /* if run length > 15, must emit special run-length-16 codes (0xF0) */
    for (r += 16 * (nonZeroItemIndex - startItemIndex); r >= 16 * 16; r -= 16 * 16)
      bitsBuffer.putBits(acTable.m_items[0xf0].m_code, acTable.m_items[0xf0].m_size);

    /* Emit Huffman symbol for run length / number of bits */
    bitsBuffer.putCode(ac + sign, nbits, acTable, r + nbits);
    r = 0;

    startItemIndex = nonZeroItemIndex + 1;
    word &= ~(itemMask << (nonZeroItemIndex * itemBits));
  }

  return r + 16 * (blockWordItems - startItemIndex);
}
#endif

static constexpr uint64_t i0mask = 0xffff;
static constexpr uint64_t i1mask = 0xffff0000;
static constexpr uint64_t i2mask = 0xffff00000000ull;
static constexpr uint64_t i3mask = 0xffff000000000000ull;

template <int i, int i0, int i1, int i2, int i3>
static __m256i subItemMask(__m256i mask)
{
  mask = _mm256_shufflehi_epi16(_mm256_shufflelo_epi16(mask, _MM_SHUFFLE(i, i, i, i)), _MM_SHUFFLE(i, i, i, i));
  return _mm256_and_si256(_mm256_setr_epi64x((uint64_t)1 << i0, (uint64_t)1 << i1, (uint64_t)1 << i2, (uint64_t)1 << i3), mask);
}

template <int i0, int i1, int i2, int i3, int i4, int i5, int i6, int i7, int i8, int i9, int i10, int i11, int i12, int i13, int i14, int i15>
static __m256i simdMask(__m256i word)
{
  __m256i mask = _mm256_xor_si256(_mm256_cmpeq_epi16(word, _mm256_setzero_si256()), _mm256_set1_epi32(0xffffffff));
  return _mm256_or_si256(subItemMask<0, i0, i4, i8, i12>(mask), _mm256_or_si256(subItemMask<1, i1, i5, i9, i13>(mask),
    _mm256_or_si256(subItemMask<2, i2, i6, i10, i14>(mask), subItemMask<3, i3, i7, i11, i15>(mask))));
}

template <int i0, int i1, int i2, int i3>
static uint64_t wordMask(uint64_t word)
{
  //  return ((word & 0xffff) ? 1 : 0) | ((word & 0xffff0000) ? 2 : 0) | ((word & 0xffff00000000ull) ? 4 : 0) | ((word & 0xffff000000000000ull) ? 8 : 0);
  return ((word & i0mask) ? (uint64_t)1 << i0 : 0) | ((word & i1mask) ? (uint64_t)1 << i1 : 0) | ((word & i2mask) ? (uint64_t)1 << i2 : 0) | ((word & i3mask) ? (uint64_t)1 << i3 : 0);
}

int64_t HuffmanEncoder::encode(const int16_t* block, const int* mcuComponents, int mcuBlockCount,
  const HuffmanEncoder* componentEncoders, const int* componentEncoderIndices, int* componentDc, int mcuCount,
  uint64_t* output, int64_t outputOffset, int64_t outputSizeInItems)
{
  Q_UNUSED(outputSizeInItems); // TODO: check
  /* Encode the DC coefficient difference per section F.1.2.1 */
  RegisterBitsBuffer<true> bitsBuffer(output, outputOffset);

  for (int mcuIndex = 0; mcuIndex < mcuCount; mcuIndex++)
  {
    for (int blockIndex = 0; blockIndex < mcuBlockCount; blockIndex++, block += Dct::BlockSize2)
    {
      int component = mcuComponents[blockIndex];
      const HuffmanEncoder& encoder = componentEncoders[componentEncoderIndices[component]];
      const LookupTable& dcTable = encoder.m_dcTable;
      const LookupTable& acTable = encoder.m_acTable;

      int dc = block[0] - componentDc[component];
      componentDc[component] = block[0];

      /* This is a well-known technique for obtaining the absolute value without a
       * branch.  It is derived from an assembly language technique presented in
       * "How to Optimize for the Pentium Processors", Copyright (c) 1996, 1997 by
       * Agner Fog.  This code assumes we are on a two's complement machine.
       */
      int32_t sign = dc >> (CHAR_BIT * sizeof(int32_t) - 1);

      /* Find the number of bits needed for the magnitude of the coefficient */
      int32_t nbits = (dc + sign) ^ sign;
      nbits = nbits ? Platform::Cpu::mostSignificantSetBit<uint32_t>((uint32_t)nbits) + 1 : 0;

      /* Emit the Huffman-coded symbol for the number of bits.
       * Emit that number of bits of the value, if positive,
       * or the complement of its magnitude, if negative.
       */
      bitsBuffer.putCode(dc + sign, nbits, dcTable, nbits);

      /* Encode the AC coefficients per section F.1.2.2 */

      int r = 0;                  /* r = run length of zeros */
#if 0
      for (int k = 1; k < Dct::BlockSize2; k++)
      {
        int32_t ac = block[k];
        if (ac == 0)
          r += 16;
        else
        {
          /* Branch-less absolute value, bitwise complement, etc., same as above */
          int32_t sign = ac >> (CHAR_BIT * sizeof(int32_t) - 1);
          int32_t nbits = (ac + sign) ^ sign;
          nbits = Platform::Cpu::mostSignificantSetBit<uint32_t>((uint32_t)nbits) + 1;

          /* if run length > 15, must emit special run-length-16 codes (0xF0) */
          for (; r >= 16 * 16; r -= 16 * 16)
            bitsBuffer.putBits(m_acTable.m_codeTable[0xf0], m_acTable.m_sizeTable[0xf0]);

          /* Emit Huffman symbol for run length / number of bits */
          r += nbits;
          bitsBuffer.putCode(ac + sign, nbits, m_acTable, r);
          r = 0;
        }
      }
#else
      const BlockWord* wblock = (BlockWord*)block;
#if defined(JPEG_HUFFMAN_ENCODER_ZIGZAG_INPUT)
      typedef int16_t ItemType;
      constexpr BlockWord firstWordMask = FilledWord << (8 * sizeof(ItemType));
      r = encodeAcWord<ItemType, 1>(wblock[0] & firstWordMask, r, m_acTable, bitsBuffer);
      r = encodeAcWord<ItemType, 0>(wblock[1], r, m_acTable, bitsBuffer);
      r = encodeAcWord<ItemType, 0>(wblock[2], r, m_acTable, bitsBuffer);
      r = encodeAcWord<ItemType, 0>(wblock[3], r, m_acTable, bitsBuffer);
      r = encodeAcWord<ItemType, 0>(wblock[4], r, m_acTable, bitsBuffer);
      r = encodeAcWord<ItemType, 0>(wblock[5], r, m_acTable, bitsBuffer);
      r = encodeAcWord<ItemType, 0>(wblock[6], r, m_acTable, bitsBuffer);
      r = encodeAcWord<ItemType, 0>(wblock[7], r, m_acTable, bitsBuffer);
      r = encodeAcWord<ItemType, 0>(wblock[8], r, m_acTable, bitsBuffer);
      r = encodeAcWord<ItemType, 0>(wblock[9], r, m_acTable, bitsBuffer);
      r = encodeAcWord<ItemType, 0>(wblock[10], r, m_acTable, bitsBuffer);
      r = encodeAcWord<ItemType, 0>(wblock[11], r, m_acTable, bitsBuffer);
      r = encodeAcWord<ItemType, 0>(wblock[12], r, m_acTable, bitsBuffer);
      r = encodeAcWord<ItemType, 0>(wblock[13], r, m_acTable, bitsBuffer);
      r = encodeAcWord<ItemType, 0>(wblock[14], r, m_acTable, bitsBuffer);
      r = encodeAcWord<ItemType, 0>(wblock[15], r, m_acTable, bitsBuffer);
#else
#if 0
      uint64_t mask = wordMask<0, 1, 5, 6>(wblock[0]) & ~(uint64_t)1;
      mask |= wordMask<14, 15, 27, 28>(wblock[1]);
      mask |= wordMask<2, 4, 7, 13>(wblock[2]);
      mask |= wordMask<16, 26, 29, 42>(wblock[3]);
      mask |= wordMask<3, 8, 12, 17>(wblock[4]);
      mask |= wordMask<25, 30, 41, 43>(wblock[5]);
      mask |= wordMask<9, 11, 18, 24>(wblock[6]);
      mask |= wordMask<31, 40, 44, 53>(wblock[7]);
      mask |= wordMask<10, 19, 23, 32>(wblock[8]);
      mask |= wordMask<39, 45, 52, 54>(wblock[9]);
      mask |= wordMask<20, 22, 33, 38>(wblock[10]);
      mask |= wordMask<46, 51, 55, 60>(wblock[11]);
      mask |= wordMask<21, 34, 37, 47>(wblock[12]);
      mask |= wordMask<50, 56, 59, 61>(wblock[13]);
      mask |= wordMask<35, 36, 48, 49>(wblock[14]);
      mask |= wordMask<57, 58, 62, 63>(wblock[15]);
#else
      const __m256i* simdBlock = (__m256i*)wblock;
      __m256i smask = _mm256_or_si256(simdMask<0, 1, 5, 6, 14, 15, 27, 28, 2, 4, 7, 13, 16, 26, 29, 42>(simdBlock[0]),
        _mm256_or_si256(simdMask<3, 8, 12, 17, 25, 30, 41, 43, 9, 11, 18, 24, 31, 40, 44, 53>(simdBlock[1]),
          _mm256_or_si256(simdMask<10, 19, 23, 32, 39, 45, 52, 54, 20, 22, 33, 38, 46, 51, 55, 60>(simdBlock[2]),
            simdMask<21, 34, 37, 47, 50, 56, 59, 61, 35, 36, 48, 49, 57, 58, 62, 63>(simdBlock[3])
          )));
      alignas(32) uint64_t smaskx4[4];
      _mm256_store_si256((__m256i*)smaskx4, smask);
      uint64_t mask = (smaskx4[0] | smaskx4[1] | smaskx4[2] | smaskx4[3]) & ~(uint64_t)1;
#endif

      int startIndex = 1;
      for (; mask;)
      {
        int nonZeroIndex = Platform::Cpu::leastSignificantSetBit<BlockWord>(mask);
        int32_t ac = block[Dct::m_zigzagOrder[nonZeroIndex]];
        /* Branch-less absolute value, bitwise complement, etc., same as above */
        int32_t sign = ac >> (CHAR_BIT * sizeof(int32_t) - 1);
        int32_t acbits = Platform::Cpu::mostSignificantSetBit<uint32_t>((uint32_t)((ac + sign) ^ sign)) + 1;

        /* if run length > 15, must emit special run-length-16 codes (0xF0) */
        int r = 16 * (nonZeroIndex - startIndex);
        for (; r >= 16 * 16; r -= 16 * 16)
          bitsBuffer.putBits(acTable.getCode(0xf0), acTable.getSize(0xf0));

        /* Emit Huffman symbol for run length / number of bits */
        bitsBuffer.putCode(ac + sign, acbits, acTable, r + acbits);

        startIndex = nonZeroIndex + 1;
        mask &= ~((uint64_t)1 << nonZeroIndex);
      }
      r = 64 - startIndex;
#endif
#endif

      /* If the last coef(s) were zero, emit an end-of-block code */
      if (r > 0)
        bitsBuffer.putBits(acTable.getCode(0), acTable.getSize(0));
    }
  }

  return bitsBuffer.flush();
}
#else
#if !defined(DEBUG) && !defined(_DEBUG)
FORCE_INLINE
#endif
static inline uint64_t toZigzagOrder(const int16_t* block, int16_t* zigzag)
{
  using namespace Platform::Cpu;
  typedef SIMD<int16_t, 8> SimdHelper;

  int16x8_t zero = SimdHelper::populate(0);
  int16x8_t d0 = int16x8_t::load(block);                       // d0 = xx 01 02 03 04 05 06 07
  int16x8_t d1 = int16x8_t::load(block + 8);                   // d1 = 08 09 10 11 12 13 14 15
  int16x8_t w0 = int16x8_t{int32x4_t::shuffle<0, 4, 1, 5>(d0, d1)}; // w0 = xx 01 08 09 02 03 10 11
  w0.shuffle<1, 2, 0, 3, 4, 5, 6, 7>();                             // w0 = 01 08 xx 09 02 03 10 11
  w0.insert<2, 7>(block[16], block[17]);                            // w0 = 01 08 16 09 02 03 10 17
  w0.onesComplement().store(zigzag);

  int16x8_t w3 = int16x8_t{int32x4_t::shuffle<2, 6, 3, 7>(d0, d1)}; // w3 = 04 05 12 13 06 07 14 15
  int16x8_t w1 = int16x8_t{int64x2_t::shuffle<0, 2>(d1, w3)};              // w1 = 08 09 10 11 04 05 12 13

  int16x8_t d3 = int16x8_t{int64x2_t::loadLowWord((int64_t*)(block + 24))}; // d3 = 24 25 26 27 xx xx xx xx
  int16x8_t w2 = d3.shuffled<0, 2, 1, 3, 4, 5, 6, 7>();                            // w2 = 24 26 25 27 xx xx xx xx
  w1.shiftWordsUp<1>();                                              // w1 = xx 08 09 10 11 04 05 12 
  w1 = int16x8_t{int64x2_t::shuffle<0, 3>(w2, w1)};                  // w1 = 24 26 25 27 11 04 05 12
  w1.insert<1, 3>(block[32], block[18]);                             // w1 = 24 32 25 18 11 04 05 12
  w1.onesComplement().store(zigzag + 8);

  int16x8_t d6 = int16x8_t::load(block + 48);                        // d6 = 48 49 50 51 52 53 54 55
  w2 = int16x8_t{int64x2_t::shuffle<0, 2>(w2, d6)};                  // w2 = 24 26 25 27 48 49 50 51
  w3.shiftWordsDown<2>();                                            // w3 = 12 13 06 07 14 15 xx xx

  w3.insert<0, 5, 6, 7>(block[20], block[21], block[28], block[35]); // w3 = 20 13 06 07 14 21 28 35
  w3.onesComplement().store(zigzag + 24);

  w2.insert<0, 2, 3, 5, 6, 7>(block[19], block[33], block[40], block[41], block[34], block[27]); // w2 = 19 26 33 40 48 41 34 27
  w2.onesComplement().store(zigzag + 16);

  uint64_t mask = SimdHelper::conditionBitMask(w0 == zero, w1 == zero) | (SimdHelper::conditionBitMask(w2 == zero, w3 == zero) << 16);

  int16x8_t d7 = int16x8_t::load(block + 56);                     // d7 = 56 57 58 59 60 61 62 63
  w3 = int16x8_t{int32x4_t::shuffle<2, 6, 3, 7>(d6, d7)};         // w3 = 52 53 60 61 54 55 62 63
  w3.shiftWordsDown<1>();                                         // w3 = 53 60 61 54 55 62 63 zz  
  w3.shuffle<1, 2, 3, 0, 4, 5, 6, 7>();                           // w3 = 60 61 54 53 55 62 63 zz
  w3.insert<3>(block[47]);                                        // w3 = 60 61 54 47 55 62 63 zz
  w3.onesComplement().store(zigzag + 56);

  w1 = int16x8_t{int64x2_t::loadLowWord<false>((int64_t*)(block + 44))};    // w1 = 44 45 46 47 xx xx xx xx
  int16x8_t w5 = int16x8_t{int32x4_t::shuffle<0, 4, 1, 5>(d6, d7)};          // w5 = 48 49 56 57 50 51 58 59
  int16x8_t w4 = int16x8_t{int64x2_t::loadLowWord((int64_t*)(block + 36))}; // w4 = 36 37 38 39 xx xx xx xx

  w4 = w1 = int16x8_t{int32x4_t::shuffle<0, 4, 1, 5>(w4, w1)};                    // w1 = 36 37 44 45 38 39 46 47
  w4.shiftWordsDown<1>();                                                         // w4 = 37 44 45 38 39 46 47 zz
  w4.shuffle<0, 1, 2, 3, 7, 4, 5, 7>();                                           // w4 = 37 44 45 38 zz 39 46 zz
  w4.insert<0, 1, 4, 7>(block[59], block[52], block[31], block[53]);              // w4 = 59 52 45 38 31 39 46 53
  w4.onesComplement().store(zigzag + 48);

  w1 = int16x8_t{int64x2_t::shuffle<0, 3>(w1, w5)}.shiftWordsUp<1>();        // w1 = 36 37 44 45 50 51 58 59 -> zz 36 37 44 45 50 51 58
  w1 = int16x8_t{int64x2_t::loadLowWord(int32x4_t{w1}.shuffled<0, 2, 1, 3>(), (int64_t*)(block + 20))}; // w1 = zz 36 45 50 37 44 51 58 -> 20 21 22 23 37 44 51 58
  w1.shuffle<2, 0, 3, 1, 4, 5, 6, 7>();                                      // w1 = 22 20 23 21 37 44 51 58
  w1.insert<1, 3>(block[15], block[30]);                                     // w1 = 22 15 23 30 37 44 51 58
  w1.onesComplement().store(zigzag + 40);

  w5.insert<0, 5, 6, 7>(block[42], block[43], block[36], block[29]); // w5 = 42 49 56 57 50 43 36 29
  w5.onesComplement().store(zigzag + 32);

  return ~(mask | (SimdHelper::conditionBitMask(w5 == zero, w1 == zero) << 32) | (SimdHelper::conditionBitMask(w4 == zero, w3 == zero) << 48));
}
#if 0
static void getAcBitLength(const int16_t* block, int16_t* length)
{
  using namespace Platform::Cpu;

  for(int i = 0; i < 4; i++)
  {
    int16x16_t w = int16x16_t::load<true>(block + i * 16);
    int16x16_cond_t isNegative = w < int16x16_t::zero();
    int16x16_t wabs = w ^ int16x16_t{isNegative};
    int16x16_t len = int16x16_t::zero();
    int16x16_t mask = int16x16_t::populate((int16_t)0xff00);
    int16x16_t maskLength = int16x16_t::populate(8);
    int16x16_cond_t maskCond = (wabs & mask) == int16x16_t::zero();

    len += maskLength.andNot(int16x16_t{maskCond});
    wabs = int16x16_t::select(wabs, wabs >> 8, maskCond);

    mask = mask >> 4;
    maskLength = maskLength >> 1;
    maskCond = (wabs & mask) == int16x16_t::zero();
    len += maskLength.andNot(int16x16_t{maskCond});
    wabs = int16x16_t::select(wabs, wabs >> 4, maskCond);

    mask = mask >> 2;
    maskLength = maskLength >> 1;
    maskCond = (wabs & mask) == int16x16_t::zero();
    len += maskLength.andNot(int16x16_t{maskCond});
    wabs = int16x16_t::select(wabs, wabs >> 2, maskCond);

    mask = mask >> 1;
    maskLength = maskLength >> 1;
    maskCond = (wabs & mask) == int16x16_t::zero();
    len += maskLength.andNot(int16x16_t{maskCond});
    wabs = int16x16_t::select(wabs, wabs >> 1, maskCond);

    mask = mask >> 1;
    maskCond = (wabs & mask) == int16x16_t::zero();
    len += maskLength.andNot(int16x16_t{maskCond});

    len.store(length + i * 16);
  }
}
#endif
namespace
{
struct AcCode
{
  uint32_t code;
  int32_t size;
};

}

static constexpr int indexBitMask(int index, int w)
{
  return index > w * 16 && index <= (w + 1) * 16 ? 1 << (index - w * 16 - 1) : 0;
}

template <typename T, int w, int... I>
T makeAcBitMaskBits()
{
  return T::create((indexBitMask(I, w))...);
}

static uint64_t reduce(Platform::Cpu::int16x8_t mask)
{
  Platform::Cpu::int64x2_t mask64x2{mask};
  uint64_t mask64 = (mask64x2 | mask64x2.shuffled<1, 1>()).get<0>();
  mask64 |= mask64 >> 32;
  return (mask64 | (mask64 >> 16)) & 0xffff;
}

static uint64_t reduce(Platform::Cpu::int16x16_t mask)
{
  return reduce(mask.lowPart() | mask.highPart());
}

#if 0
static Platform::Cpu::int64x4_t reduce4x4(Platform::Cpu::int16x16_t mask)
{
  using namespace Platform::Cpu;
  mask |= int16x16_t{uint64x4_t{mask} >> 32};
  return int64x4_t{mask | int16x16_t{uint64x4_t{mask} >> 16}} & int64x4_t::populate(0xffff);
}
#endif

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

template <int SimdLength> static uint64_t getAcMask(const int16_t* block, int16_t* dst);

typedef AcBitMask<Platform::Cpu::int16x16_t> AcBitMaskx16;

static AcBitMaskx16 acBitMaskx16_0 = makeAcBitMask<Platform::Cpu::int16x16_t, 0, 1, 5, 6, 14, 15, 27, 28, 2, 4, 7, 13, 16, 26, 29, 42>();
static AcBitMaskx16 acBitMaskx16_1 = makeAcBitMask<Platform::Cpu::int16x16_t, 3, 8, 12, 17, 25, 30, 41, 43, 9, 11, 18, 24, 31, 40, 44, 53>();
static AcBitMaskx16 acBitMaskx16_2 = makeAcBitMask<Platform::Cpu::int16x16_t, 10, 19, 23, 32, 39, 45, 52, 54, 20, 22, 33, 38, 46, 51, 55, 60>();
static AcBitMaskx16 acBitMaskx16_3 = makeAcBitMask<Platform::Cpu::int16x16_t, 21, 34, 37, 47, 50, 56, 59, 61, 35, 36, 48, 49, 57, 58, 62, 63>();

template<> 
FORCE_INLINE inline uint64_t getAcMask<16>(const int16_t* block, int16_t* dst)
{
  using namespace Platform::Cpu;
  int16x16_t bits0_15 = int16x16_t::zero(), bits16_31 = int16x16_t::zero(), bits32_47 = int16x16_t::zero(), bits48_63 = int16x16_t::zero();
  int16x16_t w0 = int16x16_t::load<true>(block);
  int16x16_t w1 = int16x16_t::load<true>(block + 16);
  int16x16_t w2 = int16x16_t::load<true>(block + 32);
  int16x16_t w3 = int16x16_t::load<true>(block + 48);
  int16x16_t zero = int16x16_t::zero();

  updateMaskBits<int16x16_t, 0x7>(acBitMaskx16_0, (w0 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w0.onesComplement().store(dst);
  updateMaskBits<int16x16_t, 0xf>(acBitMaskx16_1, (w1 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w1.onesComplement().store(dst + 16);
  updateMaskBits<int16x16_t, 0xf>(acBitMaskx16_2, (w2 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w2.onesComplement().store(dst + 32);
  updateMaskBits<int16x16_t, 0xe>(acBitMaskx16_3, (w3 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w3.onesComplement().store(dst + 48);

#if 0
  int64x4_t bits = reduce4x4(bits0_15) | (reduce4x4(bits16_31) << 16) | (reduce4x4(bits32_47) << 32) | (reduce4x4(bits48_63) << 48);
  int64x2_t bits128 = int64x2_t{bits.lowPart() | bits.highPart()};
  return (bits128 | bits128.shuffled<1, 1>()).get<0>();
#else
  return reduce(bits0_15) | (reduce(bits16_31) << 16) | (reduce(bits32_47) << 32) | (reduce(bits48_63) << 48);
#endif
}

typedef AcBitMask<Platform::Cpu::int16x8_t> AcBitMaskx8;

static AcBitMaskx8 acBitMaskx8_0 = makeAcBitMask<Platform::Cpu::int16x8_t, 0, 1, 5, 6, 14, 15, 27, 28>();
static AcBitMaskx8 acBitMaskx8_1 = makeAcBitMask<Platform::Cpu::int16x8_t, 2, 4, 7, 13, 16, 26, 29, 42>();
static AcBitMaskx8 acBitMaskx8_2 = makeAcBitMask<Platform::Cpu::int16x8_t, 3, 8, 12, 17, 25, 30, 41, 43>();
static AcBitMaskx8 acBitMaskx8_3 = makeAcBitMask<Platform::Cpu::int16x8_t, 9, 11, 18, 24, 31, 40, 44, 53>();
static AcBitMaskx8 acBitMaskx8_4 = makeAcBitMask<Platform::Cpu::int16x8_t, 10, 19, 23, 32, 39, 45, 52, 54>();
static AcBitMaskx8 acBitMaskx8_5 = makeAcBitMask<Platform::Cpu::int16x8_t, 20, 22, 33, 38, 46, 51, 55, 60>();
static AcBitMaskx8 acBitMaskx8_6 = makeAcBitMask<Platform::Cpu::int16x8_t, 21, 34, 37, 47, 50, 56, 59, 61>();
static AcBitMaskx8 acBitMaskx8_7 = makeAcBitMask<Platform::Cpu::int16x8_t, 35, 36, 48, 49, 57, 58, 62, 63>();

template<>
FORCE_INLINE inline uint64_t getAcMask<8>(const int16_t* block, int16_t* dst)
{
  using namespace Platform::Cpu;
  int16x8_t bits0_15 = int16x8_t::zero(), bits16_31 = int16x8_t::zero(), bits32_47 = int16x8_t::zero(), bits48_63 = int16x8_t::zero();
  int16x8_t w0 = int16x8_t::load<true>(block);
  int16x8_t w1 = int16x8_t::load<true>(block + 8);
  int16x8_t w2 = int16x8_t::load<true>(block + 16);
  int16x8_t w3 = int16x8_t::load<true>(block + 24);
  int16x8_t w4 = int16x8_t::load<true>(block + 32);
  int16x8_t w5 = int16x8_t::load<true>(block + 40);
  int16x8_t w6 = int16x8_t::load<true>(block + 48);
  int16x8_t w7 = int16x8_t::load<true>(block + 56);
  int16x8_t zero = int16x8_t::zero();

  updateMaskBits<int16x8_t, 0x3>(acBitMaskx8_0, (w0 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w0.onesComplement().store(dst);
  updateMaskBits<int16x8_t, 0x7>(acBitMaskx8_1, (w1 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w1.onesComplement().store(dst + 8);
  updateMaskBits<int16x8_t, 0x7>(acBitMaskx8_2, (w2 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w2.onesComplement().store(dst + 16);
  updateMaskBits<int16x8_t, 0xf>(acBitMaskx8_3, (w3 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w3.onesComplement().store(dst + 24);
  updateMaskBits<int16x8_t, 0xf>(acBitMaskx8_4, (w4 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w4.onesComplement().store(dst + 32);
  updateMaskBits<int16x8_t, 0xe>(acBitMaskx8_5, (w5 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w5.onesComplement().store(dst + 40);
  updateMaskBits<int16x8_t, 0xe>(acBitMaskx8_6, (w6 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w6.onesComplement().store(dst + 48);
  updateMaskBits<int16x8_t, 0xc>(acBitMaskx8_7, (w7 == zero).mask(), bits0_15, bits16_31, bits32_47, bits48_63);
  w7.onesComplement().store(dst + 56);

  return reduce(bits0_15) | (reduce(bits16_31) << 16) | (reduce(bits32_47) << 32) | (reduce(bits48_63) << 48);
}

template <int i0, int i1, int i2, int i3>
static uint64_t wordMask(uint64_t word)
{
  // i0 > 0 ? i0 - 1 : 0 - to prevent incorrect msvc warning
  return ((word & 0xffff) && i0 > 0 ? 1ull << (i0 > 0 ? i0 - 1 : 0) : 0) | ((word & 0xffff0000) ? 1ull << (i1 - 1) : 0) |
         ((word & 0xffff00000000ull) ? 1ull << (i2 - 1) : 0) | ((word & 0xffff000000000000ull) ? 1ull << (i3 - 1) : 0);
}

template<>
FORCE_INLINE inline uint64_t getAcMask<1>(const int16_t* block, int16_t* dst)
{
  for (int i = 1; i < 64; i++)
    dst[i] = block[i] + (block[i] >> 15);

  const uint64_t* wblock = (uint64_t*)block;
  return wordMask<0, 1, 5, 6>(wblock[0]) | wordMask<14, 15, 27, 28>(wblock[1]) |
         wordMask<2, 4, 7, 13>(wblock[2]) | wordMask<16, 26, 29, 42>(wblock[3]) |
         wordMask<3, 8, 12, 17>(wblock[4]) | wordMask<25, 30, 41, 43>(wblock[5]) |
         wordMask<9, 11, 18, 24>(wblock[6]) | wordMask<31, 40, 44, 53>(wblock[7]) |
         wordMask<10, 19, 23, 32>(wblock[8]) | wordMask<39, 45, 52, 54>(wblock[9]) |
         wordMask<20, 22, 33, 38>(wblock[10]) | wordMask<46, 51, 55, 60>(wblock[11]) |
         wordMask<21, 34, 37, 47>(wblock[12]) | wordMask<50, 56, 59, 61>(wblock[13]) |
         wordMask<35, 36, 48, 49>(wblock[14]) | wordMask<57, 58, 62, 63>(wblock[15]);
}

static int32_t acMask(uint8_t acbits)
{
//  return (1 << acbits) - 1;
  return maskTable[acbits];
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

int64_t HuffmanEncoder::encode(const int16_t* block, const int* mcuComponents, int mcuBlockCount,
  const HuffmanEncoder* componentEncoders, const int* componentEncoderIndices, int* componentDc, int mcuCount,
  uint64_t* output, int64_t outputOffset, int64_t outputSizeInItems)
{
  switch (m_simdLength)
  {
  case 16:
    return encodeBlocks<16>(block, mcuComponents, mcuBlockCount, componentEncoders, componentEncoderIndices, componentDc, mcuCount, output, outputOffset, outputSizeInItems);
  case 8:
    return encodeBlocks<8>(block, mcuComponents, mcuBlockCount, componentEncoders, componentEncoderIndices, componentDc, mcuCount, output, outputOffset, outputSizeInItems);
  case 1:
    return encodeBlocks<1>(block, mcuComponents, mcuBlockCount, componentEncoders, componentEncoderIndices, componentDc, mcuCount, output, outputOffset, outputSizeInItems);
  }

  return 0;
}

#endif

static int64_t ffByteCount64(const int8_t* bytes, int64_t count)
{
  const uint64_t* words = (const uint64_t*)bytes;
  int64_t wordsUsed = count > 0 ? bitWord<uint64_t>(count - 1) + 1 : 0;
  int64_t bytesToAdd = 0;

  if (Platform::Cpu::havePopcntInstruction<uint64_t>())
  {
    for (int64_t w = 0; w < wordsUsed; w++)
    {
      uint64_t flags = words[w] & (words[w] >> 4);
      flags &= flags >> 2;
      flags &= flags >> 1;
      bytesToAdd += Platform::Cpu::popcnt((uint64_t)(flags & 0x0101010101010101ull));
    }
  }
  else
  {
    for (int64_t w = 0; w < wordsUsed; w++)
    {
      uint64_t bytes = words[w];
      for (int j = 0; j < 8; j++)
        bytesToAdd += ((bytes >> (j * 8)) & 0xff) == 0xff ? 1 : 0;
    }
  }

  return bytesToAdd;
}

template<int SimdLength>
int64_t ffByteCount(const int8_t* bytes, int64_t bitCount)
{
  typedef Platform::Cpu::SIMD<int8_t, SimdLength> SimdHelper;
  typedef typename SimdHelper::Type SimdType;
  int64_t simdCount = bitCount / (SimdLength * 8);
  int64_t accRounds = simdCount / 255;
  int64_t ffBytes = 0;
  SimdType ff = SimdType::populate((int8_t)0xff);

  for (int64_t i = 0; i < accRounds; i++)
  {
    SimdType acc = SimdType::zero();

    for (int j = 0; j < 255; j++, bytes += SimdLength)
      acc += (SimdType::load(bytes) == ff).mask();

    acc = SimdHelper::select(acc == SimdType::zero(), acc, SimdType::zero() - acc);
    alignas(SimdLength) uint8_t accBytes[SimdLength];
    acc.store((int8_t*)accBytes);
    for (int j = 0; j < SimdLength; j++)
      ffBytes += accBytes[j];
  }
  int64_t processed = accRounds * 8 * SimdLength * 255;
  if (bitCount - processed)
    ffBytes += ffByteCount64(bytes, bitCount - processed);

  return ffBytes;
}

int64_t HuffmanEncoder::byteStuffingByteCount(const uint64_t* output, int64_t outputOffset)
{
  switch (m_byteSimdLength)
  {
  case 32:
    return ffByteCount<32>((const int8_t*)output, outputOffset);
  case 16:
    return ffByteCount<16>((const int8_t*)output, outputOffset);
  default:
    return ffByteCount64((const int8_t*)output, outputOffset);
  }
}

template <bool revertBytes>
static int64_t wordByteStuffing(uint64_t word, uint8_t* bytes, int64_t bytesToAdd)
{
  if (!(word & 0x8080808080808080ull & ~(word + 0x0101010101010101ull))) // may be false negative but that is acceptable
  {
    if (revertBytes)
      word = toBigEndian(word);
#if 0
    for (int j = 7; j >= 0; j--)
      bytes[bytesToAdd + j] = (uint8_t)((word >> (j * 8)) & 0xff);
#else
    *(uint64_t*)(bytes + bytesToAdd) = word;
#endif
  }
  else
  {
    for (int j = 7; j >= 0; j--)
    {
      uint8_t byte = (word >> ((revertBytes ? 7 - j : j) * 8)) & 0xff;
      assert(byte != 0xff || bytesToAdd > 0);
      if (byte == 0xff)
        bytes[j + bytesToAdd--] = 0;
      bytes[j + bytesToAdd] = byte;
    }
  }

  return bytesToAdd;
}

template <bool revertBytes>
static int64_t byteStuffing64(uint8_t* bytes, int64_t byteCount, int64_t bytesToAdd)
{
#if 0
  for (int64_t i = byteCount - 1; i >= 0; i--)
  {
    uint8_t byte = outputBytes[i];
    if (outputBytes[i] == 0xff)
    {
      assert(bytesToAdd > 0);
      outputBytes[i + bytesToAdd--] = 0;
    }
    outputBytes[i + bytesToAdd] = byte;
  }
#else
  uint64_t* words = (uint64_t*)bytes;
  for (int64_t w = (byteCount ? byteCount - 1 : 0) / sizeof(int64_t); w >= 0; w--)
    bytesToAdd = wordByteStuffing<revertBytes>(words[w], bytes + w * 8, bytesToAdd);
#endif

  return bytesToAdd;
}

template<int SimdLength, bool revertBytes>
void simdByteStuffing(uint8_t* bytes, int64_t byteCount, int64_t bytesToAdd)
{
  using namespace Platform::Cpu;
  typedef typename SIMD<int8_t, SimdLength>::Type SimdType;
  typedef typename SIMD<int64_t, SimdLength/8>::Type SimdType64;
  int64_t simdCount = byteCount / SimdLength;
  SimdType ff = SimdType::populate((int8_t)0xff);

  if (simdCount * SimdLength < byteCount)
    bytesToAdd = byteStuffing64<revertBytes>(bytes + simdCount * SimdLength, byteCount - simdCount * SimdLength, bytesToAdd);

  for(int64_t i = simdCount - 1; i >= 0; i--)
  {
    SimdType simdBytes = SimdType::load((int8_t*)bytes + i * SimdLength);
    if (!(simdBytes == ff).bitMask())
      (revertBytes ? SimdType{SimdType64{simdBytes}.revertedByteOrder()} : simdBytes).template store<false>((int8_t*)bytes + i * SimdLength + bytesToAdd);
    else
    {
      uint64_t* words = (uint64_t*)(bytes + i * SimdLength);
      for (int j = SimdLength / sizeof(uint64_t) - 1; j >= 0; j--)
        bytesToAdd = wordByteStuffing<revertBytes>(words[j], (uint8_t*)(words + j), bytesToAdd);
    }
  }
  assert(bytesToAdd == 0);
}

int64_t HuffmanEncoder::byteStuffing(uint64_t* output, int64_t outputOffset, int64_t bytesToAdd)
{
  constexpr bool revertByteOrder = Platform::Cpu::byteOrder != Platform::Cpu::BigEndian;
  uint8_t* outputBytes = (uint8_t*)output;
  int64_t bytesUsed = outputOffset > 0 ? bitWord<uint8_t>(outputOffset - 1) + 1 : 0;

  outputOffset += bytesToAdd * 8;
  switch (m_byteSimdLength)
  {
  case 32:
    simdByteStuffing<32, revertByteOrder>(outputBytes, bytesUsed, bytesToAdd);
    break;
  case 16:
    simdByteStuffing<16, revertByteOrder>(outputBytes, bytesUsed, bytesToAdd);
    break;
  default:
    bytesToAdd = byteStuffing64<revertByteOrder>(outputBytes, bytesUsed, bytesToAdd);
    assert(bytesToAdd == 0);
    break;
  }

  return outputOffset;
}

int64_t HuffmanEncoder::padToByteBoundary(uint64_t* output, int64_t outputOffset)
{
  int bitsToAdd = 8 - outputOffset % 8;
  if (bitsToAdd == 8)
    return outputOffset;

  RegisterBitsBuffer<true> bitsBuffer(output, outputOffset);
  bitsBuffer.putBits(0xff >> (8 - bitsToAdd), bitsToAdd);
  return bitsBuffer.flush();
}

uint64_t* HuffmanEncoder::allocBitBuffer(int64_t wordCount)
{
  switch (m_byteSimdLength)
  {
  case 32:
    return Platform::Cpu::SIMD<int8_t, 32>::allocMemory<uint64_t>(wordCount);
  case 16:
    return Platform::Cpu::SIMD<int8_t, 16>::allocMemory<uint64_t>(wordCount);
  }

  return (uint64_t*)malloc(sizeof(uint64_t) * wordCount);
}

void HuffmanEncoder::freeBitBuffer(uint64_t* buffer)
{
  switch (m_byteSimdLength)
  {
  case 32:
    return Platform::Cpu::SIMD<int8_t, 32>::freeMemory(buffer);
  case 16:
    return Platform::Cpu::SIMD<int8_t, 16>::freeMemory(buffer);
  }

  free(buffer);
}

}
