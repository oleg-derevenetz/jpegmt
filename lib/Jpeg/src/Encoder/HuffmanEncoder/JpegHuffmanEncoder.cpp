#include "JpegHuffmanEncoder.h"

#include "JpegByteStuffingTemplates.h"
#include "JpegHuffmanEncoderTemplates.h"

namespace Jpeg
{

HuffmanEncoderOptions::HuffmanEncoderOptions(int encoderMaxSimdLength, int byteStuffingMaxSimdLength) :
  m_encoderSimdLength(detectSimdLength(encoderMaxSimdLength)),
  m_byteStuffingSimdLength(detectByteStuffingSimdLength(byteStuffingMaxSimdLength))
{
}

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

int64_t HuffmanEncoder::encode(const int16_t* block, int prevDcValue, uint64_t* output, int64_t outputOffset, int64_t outputSizeInItems, const HuffmanEncoderOptions& options) const
{
  int mcuComponents = 0, encoderIndex = 0;
  return encode(block, &mcuComponents, 1, this, &encoderIndex, &prevDcValue, 1, output, outputOffset, outputSizeInItems, options);
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
static uint64_t toZigzagOrder(const int16_t* block, int16_t* zigzag)
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

template <int i0, int i1, int i2, int i3>
static uint64_t wordMask(uint64_t word)
{
  // i0 > 0 ? i0 - 1 : 0 - to prevent incorrect msvc warning
  return ((word & 0xffff) && i0 > 0 ? 1ull << (i0 > 0 ? i0 - 1 : 0) : 0) | ((word & 0xffff0000) ? 1ull << (i1 - 1) : 0) |
         ((word & 0xffff00000000ull) ? 1ull << (i2 - 1) : 0) | ((word & 0xffff000000000000ull) ? 1ull << (i3 - 1) : 0);
}

template<>
FORCE_INLINE uint64_t getAcMask<1>(const int16_t* block, int16_t* dst)
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

template <>
int64_t HuffmanEncoder::encodeBlocksImplementation<1>(const int16_t* block, const int* mcuComponents, int mcuBlockCount,
  const HuffmanEncoder* componentEncoders, const int* componentEncoderIndices, int* componentDc, int mcuCount,
  uint64_t* output, int64_t outputOffset, int64_t outputSizeInItems)
{
  return encodeBlocks<1>(block, mcuComponents, mcuBlockCount, componentEncoders, componentEncoderIndices, componentDc, mcuCount, output, outputOffset, outputSizeInItems);
}

namespace
{

struct Int64ReturnTypeCallable
{
  typedef int64_t ReturnType;
  static int64_t makeDefaultValue()
  {
    return 0;
  }
};

}

struct EncodeBlocksCallable : public Int64ReturnTypeCallable
{
  template <typename T, int SimdLength>
  static int64_t perform(const int16_t* block, const int* mcuComponents, int mcuBlockCount,
    const HuffmanEncoder* componentEncoders, const int* componentEncoderIndices, int* componentDc, int mcuCount,
    uint64_t* output, int64_t outputOffset, int64_t outputSizeInItems)
  {
    static_assert(sizeof(T) == sizeof(int16_t), "invalid type used");
    return HuffmanEncoder::encodeBlocksImplementation<SimdLength>(block, mcuComponents, mcuBlockCount, componentEncoders, componentEncoderIndices, componentDc, mcuCount, output, outputOffset, outputSizeInItems);
  }
};


int64_t HuffmanEncoder::encode(const int16_t* block, const int* mcuComponents, int mcuBlockCount,
  const HuffmanEncoder* componentEncoders, const int* componentEncoderIndices, int* componentDc, int mcuCount,
  uint64_t* output, int64_t outputOffset, int64_t outputSizeInItems, const HuffmanEncoderOptions& options)
{
  return SimdFunctionChooser<EncodeBlocksCallable>::perform<int16_t>(options.m_encoderSimdLength, block, mcuComponents, mcuBlockCount, componentEncoders, componentEncoderIndices, componentDc, mcuCount, output, outputOffset, outputSizeInItems);
}

#endif

template <> int64_t byteStuffingByteCountImplementation<1>(const uint64_t* output, int64_t outputOffset)
{
  return ffByteCount64((const int8_t*)output, outputOffset);
}

template <> void byteStuffingImplementation<1>(uint8_t* bytes, int64_t byteCount, int64_t bytesToAdd)
{
  bytesToAdd = byteStuffing64(bytes, byteCount, bytesToAdd);
  assert(bytesToAdd == 0);
}

template <> uint64_t* allocBitBufferImplementation<1>(int64_t wordCount)
{
  return (uint64_t*)malloc(sizeof(uint64_t) * wordCount);
}

template <> void freeBitBufferImplementation<1>(uint64_t* buffer)
{
  return free(buffer);
}

namespace
{

struct ByteStuffingByteCountCallable : public Int64ReturnTypeCallable
{
  template <typename T, int SimdLength>
  static int64_t perform(const uint64_t* output, int64_t outputOffset)
  {
    static_assert(sizeof(T) == sizeof(int8_t), "invalid type used");
    return byteStuffingByteCountImplementation<SimdLength>(output, outputOffset);
  }
};

struct ByteStuffingCallable : public NoReturnValueCallable
{
  template <typename T, int SimdLength>
  static void perform(uint8_t* bytes, int64_t byteCount, int64_t bytesToAdd)
  {
    static_assert(sizeof(T) == sizeof(int8_t), "invalid type used");
    byteStuffingImplementation<SimdLength>(bytes, byteCount, bytesToAdd);
  }
};

struct AllocBitBufferCountCallable
{
  typedef uint64_t* ReturnType;
  static uint64_t* makeDefaultValue()
  {
    return nullptr;
  }

  template <typename T, int SimdLength>
  static uint64_t* perform(int64_t wordCount)
  {
    static_assert(sizeof(T) == sizeof(int8_t), "invalid type used");
    return allocBitBufferImplementation<SimdLength>(wordCount);
  }
};

struct FreeBitBufferCallable : public NoReturnValueCallable
{
  template <typename T, int SimdLength>
  static void perform(uint64_t* buffer)
  {
    static_assert(sizeof(T) == sizeof(int8_t), "invalid type used");
    freeBitBufferImplementation<SimdLength>(buffer);
  }
};

}

int64_t HuffmanEncoder::byteStuffingByteCount(const uint64_t* output, int64_t outputOffset, const HuffmanEncoderOptions& options)
{
  return SimdFunctionChooser<ByteStuffingByteCountCallable>::perform<int8_t>(options.m_byteStuffingSimdLength, output, outputOffset);
}

int64_t HuffmanEncoder::byteStuffing(uint64_t* output, int64_t outputOffset, int64_t bytesToAdd, const HuffmanEncoderOptions& options)
{
  uint8_t* outputBytes = (uint8_t*)output;
  int64_t bytesUsed = outputOffset > 0 ? bitWord<uint8_t>(outputOffset - 1) + 1 : 0;

  outputOffset += bytesToAdd * 8;
  SimdFunctionChooser<ByteStuffingCallable>::perform<int8_t>(options.m_byteStuffingSimdLength, outputBytes, bytesUsed, bytesToAdd);
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

uint64_t* HuffmanEncoder::allocBitBuffer(int64_t wordCount, const HuffmanEncoderOptions& options)
{
  return SimdFunctionChooser<AllocBitBufferCountCallable>::perform<int8_t>(options.m_byteStuffingSimdLength, wordCount);
}

void HuffmanEncoder::freeBitBuffer(uint64_t* buffer, const HuffmanEncoderOptions& options)
{
  SimdFunctionChooser<FreeBitBufferCallable>::perform<int8_t>(options.m_byteStuffingSimdLength, buffer);
}

}
