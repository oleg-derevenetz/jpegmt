#include "JpegForwardDct.h"

#include <Helper/Platform/Cpu/simd.h>

#include "../EncoderBuffer/JpegEncoderBuffer.h"
#include "../JpegQuantizer.h"

#include "JpegForwardDctTemplates.h"

namespace Jpeg
{

template <>
struct Int16SimdHelper<1>
{
  struct ExtendedType
  {
    int32_t value;

    template <int fixedPointBits> int16_t round()
    {
      return (value + (1 << (fixedPointBits - 1))) >> fixedPointBits;
    }

    ExtendedType operator+(ExtendedType other) const
    {
      return ExtendedType{value + other.value};
    }
  };

  static ExtendedType mulExtended(int16_t a, int16_t b)
  {
    return ExtendedType{a * b};
  }

  template <int16_t aFactor, int16_t bFactor>
  static ExtendedType mulAdd(int16_t a, int16_t b)
  {
    return ExtendedType{a * aFactor + b * bFactor};
  }
};

#ifndef TRANSPOSED_SIMD_BUFFER
namespace
{

struct BlockSimd4x16
{
  Platform::Cpu::int16x16_t w0, w1, w2, w3;
  typedef Platform::Cpu::SIMD<int16_t, 16>::NativeType NativeType;

  BlockSimd4x16(NativeType v0, NativeType v1, NativeType v2, NativeType v3) : w0{v0}, w1{v1}, w2{v2}, w3{v3} {}
};

}

static BlockSimd4x16 loadBlockSimd4x16(const int16_t* block)
{
  using namespace Platform::Cpu;

  int16x16_t w0 = int16x16_t::load(block);
  int16x16_t w1 = int16x16_t::load(block + 16);
  int16x16_t w2 = int16x16_t::load(block + 32);
  int16x16_t w3 = int16x16_t::load(block + 48);
  // w0=(00 01 02 03 04 05 06 07  10 11 12 13 14 15 16 17)
  // w1=(20 21 22 23 24 25 26 27  30 31 32 33 34 35 36 37)
  // w2=(40 41 42 43 44 45 46 47  50 51 52 53 54 55 56 57)
  // w3=(60 61 62 63 64 65 66 67  70 71 72 73 74 75 76 77)

  return BlockSimd4x16{
    int128x2_t::shuffle<0, 2>(w0, w2),
    int128x2_t::shuffle<1, 3>(w0, w2),
    int128x2_t::shuffle<0, 2>(w1, w3),
    int128x2_t::shuffle<1, 3>(w1, w3)
  };
  // w4 = (00 01 02 03 04 05 06 07  40 41 42 43 44 45 46 47)
  // w5 = (10 11 12 13 14 15 16 17  50 51 52 53 54 55 56 57)
  // w6 = (20 21 22 23 24 25 26 27  60 61 62 63 64 65 66 67)
  // w7 = (30 31 32 33 34 35 36 37  70 71 72 73 74 75 76 77)

}

static BlockSimd4x16 transposeForDct(BlockSimd4x16 block)
{
  using namespace Platform::Cpu;

  // w0=(00 01 02 03 04 05 06 07  40 41 42 43 44 45 46 47)
  // w1=(10 11 12 13 14 15 16 17  50 51 52 53 54 55 56 57)
  // w2=(20 21 22 23 24 25 26 27  60 61 62 63 64 65 66 67)
  // w3=(30 31 32 33 34 35 36 37  70 71 72 73 74 75 76 77)

  int16x16_t w4 = SIMD<int16_t, 16>::interleaveEach4Low(block.w0, block.w1);
  int16x16_t w5 = SIMD<int16_t, 16>::interleaveEach4High(block.w0, block.w1);
  int16x16_t w6 = SIMD<int16_t, 16>::interleaveEach4Low(block.w2, block.w3);
  int16x16_t w7 = SIMD<int16_t, 16>::interleaveEach4High(block.w2, block.w3);
  // transpose coefficients(phase 1)
  // w4=(00 10 01 11 02 12 03 13  40 50 41 51 42 52 43 53)
  // w5=(04 14 05 15 06 16 07 17  44 54 45 55 46 56 47 57)
  // w6=(20 30 21 31 22 32 23 33  60 70 61 71 62 72 63 73)
  // w7=(24 34 25 35 26 36 27 37  64 74 65 75 66 76 67 77)

  return BlockSimd4x16{
    int64x4_t::shuffle<1, 3, 0, 2>(SIMD<int32_t, 8>::interleaveEach2Low(w4, w6)),
    int64x4_t::shuffle<1, 3, 0, 2>(SIMD<int32_t, 8>::interleaveEach2High(w4, w6)),
    int64x4_t::shuffle<0, 2, 1, 3>(SIMD<int32_t, 8>::interleaveEach2Low(w5, w7)),
    int64x4_t::shuffle<0, 2, 1, 3>(SIMD<int32_t, 8>::interleaveEach2High(w5, w7))
  };

  // transpose coefficients(phase 2)
  // w0=(00 10 20 30 01 11 21 31  40 50 60 70 41 51 61 71)
  // w1=(02 12 22 32 03 13 23 33  42 52 62 72 43 53 63 73)
  // w2=(04 14 24 34 05 15 25 35  44 54 64 74 45 55 65 75)
  // w3=(06 16 26 36 07 17 27 37  46 56 66 76 47 57 67 77)

  // transpose coefficients(phase 3)
  // w0=(01 11 21 31 41 51 61 71  00 10 20 30 40 50 60 70)
  // w1=(03 13 23 33 43 53 63 73  02 12 22 32 42 52 62 72)
  // w2=(04 14 24 34 44 54 64 74  05 15 25 35 45 55 65 75)
  // w3=(06 16 26 36 46 56 66 76  07 17 27 37 47 57 67 77)
}

template <int passNumber>
FORCE_INLINE BlockSimd4x16 performDct(BlockSimd4x16 block)
{
  using namespace Platform::Cpu;
  typedef SIMD<int16_t, 16> SimdHelper;

  using namespace int16;
  using namespace int32;

  // w0=(01 11 21 31 41 51 61 71  00 10 20 30 40 50 60 70)
  // w1=(03 13 23 33 43 53 63 73  02 12 22 32 42 52 62 72)
  // w2=(04 14 24 34 44 54 64 74  05 15 25 35 45 55 65 75)
  // w3=(06 16 26 36 46 56 66 76  07 17 27 37 47 57 67 77)

  int16x16_t tmp6_7 = block.w0 - block.w3; // tmp6_7 = b1_0 - b6_7
  int16x16_t tmp0_1 = int16x16_t{int128x2_t::shuffle<1, 0>(block.w0 + block.w3)}; // tmp0_1 = b0_1 + b7_6
  int16x16_t tmp3_2 = block.w1 + block.w2; // tmp3_2 = b3_2 - b4_5
  int16x16_t tmp4_5 = block.w1 - block.w2; // tmp4_5 = b3_2 + b4_5

  // Even part

  int16x16_t tmp10_11 = tmp0_1 + tmp3_2; // tmp10_11 = tmp0_1+tmp3_2

  int16x16_t negMask = SimdHelper::create(1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1);
  int16x16_t w0 = int16x16_t{int128x2_t{tmp10_11}.shuffled<1, 0>()} + SimdHelper::mulSign(tmp10_11, negMask); // (tmp10+tmp11)_(tmp10-tmp11)

  if (passNumber == 1)
    block.w0 = w0 << PASS1_BITS;
  else
  {
    int16x16_t w0Rounding = SimdHelper::populate(1 << (PASS1_BITS - 1));
    block.w0 = (w0 + w0Rounding) >> PASS1_BITS;
  }

  // (Original)
  // z1 = (tmp12 + tmp13) * 0.541196100;
  // data2 = z1 + tmp13 * 0.765366865;
  // data6 = z1 + tmp12 * -1.847759065;
  //
  // (This implementation)
  // data2 = tmp13 * (0.541196100 + 0.765366865) + tmp12 * 0.541196100;
  // data6 = tmp13 * 0.541196100 + tmp12 * (0.541196100 - 1.847759065);

  constexpr int descaleShift = passNumber == 1 ? CONST_BITS - PASS1_BITS : CONST_BITS + PASS1_BITS;
  int16x16_t w2 = tmp0_1 - tmp3_2;          // tmp13_12 = tmp0_1-tmp3_2
  block.w2 = SimdHelper::mulAdd<FIX_0_541196100 + FIX_0_765366865, FIX_0_541196100 - FIX_1_847759065, FIX_0_541196100, FIX_0_541196100>(
    w2, int16x16_t{int128x2_t{w2}.shuffled<1, 0>()}).round<descaleShift>();

  // Odd part

  // (Original)
  // z5 = (z3 + z4) * 1.175875602;
  // z3 = z3 * -1.961570560;  z4 = z4 * -0.390180644;
  // z3 += z5;  z4 += z5;
  //
  // (This implementation)
  // z3 = z3 * (1.175875602 - 1.961570560) + z4 * 1.175875602;
  // z4 = z3 * 1.175875602 + z4 * (1.175875602 - 0.390180644);

  int16x16_t z3_4 = tmp4_5 + tmp6_7;
  SimdHelper::ExtendedType z3_4_int32 = SimdHelper::mulAdd<FIX_1_175875602 - FIX_1_961570560, FIX_1_175875602 - FIX_0_390180644, FIX_1_175875602, FIX_1_175875602>(
    z3_4, int16x16_t{int128x2_t{z3_4}.shuffled<1, 0>()});

  // (Original)
  // z1 = tmp4 + tmp7;  z2 = tmp5 + tmp6;
  // tmp4 = tmp4 * 0.298631336;  tmp5 = tmp5 * 2.053119869;
  // tmp6 = tmp6 * 3.072711026;  tmp7 = tmp7 * 1.501321110;
  // z1 = z1 * -0.899976223;  z2 = z2 * -2.562915447;
  // data7 = tmp4 + z1 + z3;  data5 = tmp5 + z2 + z4;
  // data3 = tmp6 + z2 + z3;  data1 = tmp7 + z1 + z4;
  //
  // (This implementation)
  // tmp4 = tmp4 * (0.298631336 - 0.899976223) + tmp7 * -0.899976223;
  // tmp5 = tmp5 * (2.053119869 - 2.562915447) + tmp6 * -2.562915447;
  // tmp6 = tmp5 * -2.562915447 + tmp6 * (3.072711026 - 2.562915447);
  // tmp7 = tmp4 * -0.899976223 + tmp7 * (1.501321110 - 0.899976223);
  // data7 = tmp4 + z3;  data5 = tmp5 + z4;
  // data3 = tmp6 + z3;  data1 = tmp7 + z4;

  block.w3 = (SimdHelper::mulAdd<FIX_0_298631336 - FIX_0_899976223, FIX_2_053119869 - FIX_2_562915447, -FIX_0_899976223, -FIX_2_562915447>(
    tmp4_5, int16x16_t{int128x2_t{tmp6_7}.shuffled<1, 0>()}) + z3_4_int32).round<descaleShift>();

  block.w1 = (SimdHelper::mulAdd<FIX_3_072711026 - FIX_2_562915447, FIX_1_501321110 - FIX_0_899976223, -FIX_2_562915447, -FIX_0_899976223>(
    tmp6_7, int16x16_t{int128x2_t{tmp4_5}.shuffled<1, 0>()}) + z3_4_int32).round<descaleShift>();

  return block;
}

static BlockSimd4x16 pass1ToPass2(BlockSimd4x16 block)
{
  using namespace Platform::Cpu;
  int16x16_t w1 = block.w1;

  // data0_4, data3_1, data2_6, data7_5
  block.w1 = int16x16_t{int128x2_t::shuffle<1, 3>(w1, block.w3)};
  block.w3 = int16x16_t{int128x2_t::shuffle<0, 2>(w1, block.w3)};
  // data0_4, data1_5, data2_6, data3_7
  return block;
}

static BlockSimd4x16 pass2toOut(BlockSimd4x16 block)
{
  using namespace Platform::Cpu;

  // data0_4, data3_1, data2_6, data7_5
  return BlockSimd4x16{
    int128x2_t::shuffle<0, 3>(block.w0, block.w1),
    int128x2_t::shuffle<0, 2>(block.w2, block.w1),
    int128x2_t::shuffle<1, 3>(block.w0, block.w3),
    int128x2_t::shuffle<1, 2>(block.w2, block.w3)
  };
  // data0_1, data2_3, data4_5, data6_7
}

static void storeBlockSimd4x16(BlockSimd4x16 block, int16_t* dst)
{
  block.w0.store(dst);
  block.w1.store(dst + 16);
  block.w2.store(dst + 32);
  block.w3.store(dst + 48);
}

void ForwardDct::perform(EncoderBuffer& buffer, const Quantizer* quantizers, const int* componentQuantizerIndices)
{
  if (Platform::Cpu::SIMD<int16_t, 16>::isSupported())
  {
    for (int n = 0; n < buffer.m_blockCount; n++)
    {
      for (int i = 0; i < buffer.m_metaData.m_components.size(); i++)
      {
        const EncoderBuffer::MetaData::Component& component = buffer.m_metaData.m_components.at(i);
        const Quantizer& quantizer = quantizers[componentQuantizerIndices[i]];

        for (int j = 0; j < component.m_blockCount; j++)
        {
          int16_t* data = buffer.m_componentBuffers[n * buffer.m_metaData.m_mcuBlockCount + component.m_blockOffset + j];
          BlockSimd4x16 block = loadBlockSimd4x16(data);
          block = transposeForDct(block);
          block = performDct<1>(block);
          block = pass1ToPass2(block);
          block = transposeForDct(block);
          block = performDct<2>(block);
          block = pass2toOut(block);
          quantizer.quantizeBlock(block.w0, block.w1, block.w2, block.w3);
          storeBlockSimd4x16(block, data);
        }
      }
    }
  }
  else
  {
  }
}
#else // ifndef TRANSPOSED_SIMD_BUFFER

template<>
void performDctImplementation<int16_t, 1>(int simdBlockCount, const EncoderBuffer::MetaData& bufferMetaData, const Quantizer* quantizers, const int* componentQuantizerIndices, int16_t* bufferData)
{
  return performDct<int16_t, 1>(simdBlockCount, bufferMetaData, quantizers, componentQuantizerIndices, bufferData);
}

template<>
void performDctImplementation<int32_t, 1>(int simdBlockCount, const EncoderBuffer::MetaData& bufferMetaData, const Quantizer* quantizers, const int* componentQuantizerIndices, int32_t* bufferData)
{
  return performDct<int32_t, 1>(simdBlockCount, bufferMetaData, quantizers, componentQuantizerIndices, bufferData);
}

namespace
{

struct dctCallable : public NoReturnValueCallable
{
  template <typename T, int SimdLength>
  static void perform(int simdBlockCount, const EncoderBuffer::MetaData& bufferMetaData, const Quantizer* quantizers, const int* componentQuantizerIndices, T* bufferData)
  {
    return performDctImplementation<T, SimdLength>(simdBlockCount, bufferMetaData, quantizers, componentQuantizerIndices, bufferData);
  }
};

}

void ForwardDct::perform(EncoderBuffer& buffer, const Quantizer* quantizers, const int* componentQuantizerIndices)
{
  switch(buffer.m_metaData.m_itemType)
  {
  case EncoderBuffer::MetaData::Int16:
    return SimdFunctionChooser<dctCallable>::perform<int16_t>(buffer.m_metaData.m_simdLength, buffer.m_simdBlockCount, buffer.m_metaData, quantizers, componentQuantizerIndices, (int16_t*)buffer.m_itemBuffer);
  case EncoderBuffer::MetaData::Int32:
    return SimdFunctionChooser<dctCallable>::perform<int32_t>(buffer.m_metaData.m_simdLength, buffer.m_simdBlockCount, buffer.m_metaData, quantizers, componentQuantizerIndices, (int32_t*)buffer.m_itemBuffer);
  }
}
#endif // ifndef TRANSPOSED_SIMD_BUFFER

}
