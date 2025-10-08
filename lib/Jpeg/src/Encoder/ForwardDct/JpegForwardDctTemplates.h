#include <Helper/Platform/Cpu/simd.h>

#include "../EncoderBuffer/JpegEncoderBuffer.h"
#include "../Common.h"
#include "../JpegQuantizer.h"

#define CONST_BITS  13
#define PASS1_BITS  2

#define FIX_0_298631336  (2446)          /* FIX(0.298631336) */
#define FIX_0_390180644  (3196)          /* FIX(0.390180644) */
#define FIX_0_541196100  (4433)          /* FIX(0.541196100) */
#define FIX_0_765366865  (6270)          /* FIX(0.765366865) */
#define FIX_0_899976223  (7373)          /* FIX(0.899976223) */
#define FIX_1_175875602  (9633)          /* FIX(1.175875602) */
#define FIX_1_501321110  (12299)         /* FIX(1.501321110) */
#define FIX_1_847759065  (15137)         /* FIX(1.847759065) */
#define FIX_1_961570560  (16069)         /* FIX(1.961570560) */
#define FIX_2_053119869  (16819)         /* FIX(2.053119869) */
#define FIX_2_562915447  (20995)         /* FIX(2.562915447) */
#define FIX_3_072711026  (25172)         /* FIX(3.072711026) */

namespace Jpeg
{

namespace
{

enum QuantizationMode
{
  Unit,
  Fast,
  Slow,
};

template <int SimdSize, int passNumber, QuantizationMode quantizationMode>
FORCE_INLINE void store(int32_t (*result)[Dct::BlockSize][SimdSize], int x, int y, typename Platform::Cpu::SIMD<int32_t, SimdSize>::Type value, const Quantizer& quantizer)
{
  if (passNumber == 2)
    value = quantizer.quantize<SimdSize>(value, y * Dct::BlockSize + x);
  Platform::Cpu::SIMD<int32_t, SimdSize>::store(result[y][x], value);
}

template <int SimdSize, int passNumber, QuantizationMode quantizationMode>
static void processDimension(const int32_t (*values)[SimdSize], int32_t (*result)[Dct::BlockSize][SimdSize], const Quantizer& quantizer)
{
  static_assert(passNumber == 1 || passNumber == 2, "invalid pass number");
  typedef Platform::Cpu::SIMD<int32_t, SimdSize> SimdHelper;
  typedef typename SimdHelper::Type SimdType;

  assert(Dct::BlockSize == 8);
  for (int x = 0; x < 8; x++, values += 8)
  {
    SimdType v0 = SimdHelper::load(values[0]), v7 = SimdHelper::load(values[7]);
    SimdType v1 = SimdHelper::load(values[1]), v6 = SimdHelper::load(values[6]);
    SimdType v2 = SimdHelper::load(values[2]), v5 = SimdHelper::load(values[5]);
    SimdType v3 = SimdHelper::load(values[3]), v4 = SimdHelper::load(values[4]);
    SimdType tmp0 = v0 + v7, tmp7 = v0 - v7;
    SimdType tmp1 = v1 + v6, tmp6 = v1 - v6;
    SimdType tmp2 = v2 + v5, tmp5 = v2 - v5;
    SimdType tmp3 = v3 + v4, tmp4 = v3 - v4;

    SimdType tmp10 = tmp0 + tmp3;
    SimdType tmp13 = tmp0 - tmp3;
    SimdType tmp11 = tmp1 + tmp2;
    SimdType tmp12 = tmp1 - tmp2;

    SimdType half = SimdHelper::populate(1 << (PASS1_BITS - 1));
    store<SimdSize, passNumber, quantizationMode>(result, x, 0, passNumber == 1 ? ((tmp10 + tmp11) << PASS1_BITS) : (tmp10 + tmp11 + half) >> PASS1_BITS, quantizer);
    store<SimdSize, passNumber, quantizationMode>(result, x, 4, passNumber == 1 ? ((tmp10 - tmp11) << PASS1_BITS) : (tmp10 - tmp11 + half) >> PASS1_BITS, quantizer);

    SimdType z1 = (tmp12 + tmp13) * FIX_0_541196100;

    constexpr int descaleShift = passNumber == 1 ? CONST_BITS - PASS1_BITS : CONST_BITS + PASS1_BITS;
    half = SimdHelper::populate(1 << (descaleShift - 1));
    store<SimdSize, passNumber, quantizationMode>(result, x, 2, (z1 + tmp13 * FIX_0_765366865 + half) >> descaleShift, quantizer);
    store<SimdSize, passNumber, quantizationMode>(result, x, 6, (z1 + tmp12 * -FIX_1_847759065 + half) >> descaleShift, quantizer);

    z1 = tmp4 + tmp7;
    SimdType z2 = tmp5 + tmp6;
    SimdType z3 = tmp4 + tmp6;
    SimdType z4 = tmp5 + tmp7;
    SimdType z5 = (z3 + z4) * FIX_1_175875602; /* sqrt(2) * c3 */

    tmp4 *= FIX_0_298631336; /* sqrt(2) * (-c1+c3+c5-c7) */
    tmp5 *= FIX_2_053119869; /* sqrt(2) * ( c1+c3-c5+c7) */
    tmp6 *= FIX_3_072711026; /* sqrt(2) * ( c1+c3+c5-c7) */
    tmp7 *= FIX_1_501321110; /* sqrt(2) * ( c1+c3-c5-c7) */
    z1 *= -FIX_0_899976223; /* sqrt(2) * ( c7-c3) */
    z2 *= -FIX_2_562915447; /* sqrt(2) * (-c1-c3) */
    z3 *= -FIX_1_961570560; /* sqrt(2) * (-c3-c5) */
    z4 *= -FIX_0_390180644; /* sqrt(2) * ( c5-c3) */

    z3 += z5;
    z4 += z5;

    store<SimdSize, passNumber, quantizationMode>(result, x, 7, (tmp4 + z1 + z3 + half) >> descaleShift, quantizer);
    store<SimdSize, passNumber, quantizationMode>(result, x, 5, (tmp5 + z2 + z4 + half) >> descaleShift, quantizer);
    store<SimdSize, passNumber, quantizationMode>(result, x, 3, (tmp6 + z2 + z3 + half) >> descaleShift, quantizer);
    store<SimdSize, passNumber, quantizationMode>(result, x, 1, (tmp7 + z1 + z4 + half) >> descaleShift, quantizer);
  }
}

template <int SimdSize, int passNumber, QuantizationMode quantizationMode>
FORCE_INLINE void store(int16_t (*result)[Dct::BlockSize][SimdSize], int x, int y, typename Platform::Cpu::SIMD<int16_t, SimdSize>::ParamType value, const Quantizer& quantizer)
{
  if (passNumber == 2)
  {
    if (quantizationMode == Slow)
      Platform::Cpu::SIMD<int16_t, SimdSize>::store(result[y][x], quantizer.quantize32<SimdSize>(value, y * Dct::BlockSize + x));
    else if (quantizationMode == Fast)
      Platform::Cpu::SIMD<int16_t, SimdSize>::store(result[y][x], quantizer.quantize<SimdSize>(value, y * Dct::BlockSize + x));
    else if (quantizationMode == Unit)
      Platform::Cpu::SIMD<int16_t, SimdSize>::store(result[y][x], quantizer.descale<SimdSize>(value));
  }
  else
    Platform::Cpu::SIMD<int16_t, SimdSize>::store(result[y][x], value);
}

template <int SimdSize>
struct Int16SimdHelper : public Platform::Cpu::SIMD<int16_t, SimdSize>
{
};

template <int SimdSize, int passNumber, QuantizationMode quantizationMode>
static void processDimension(const int16_t (*values)[SimdSize], int16_t (*result)[Dct::BlockSize][SimdSize], const Quantizer& quantizer)
{
  static_assert(passNumber == 1 || passNumber == 2, "invalid pass number");
  typedef Platform::Cpu::SIMD<int16_t, SimdSize> SimdHelper;
  typedef Int16SimdHelper<SimdSize> Int16Helper;
  typedef typename SimdHelper::Type SimdType;
  typedef typename Int16Helper::ExtendedType ExtendedSimdType;

  assert(Dct::BlockSize == 8);
  for (int x = 0; x < 8; x++, values += 8)
  {
    SimdType v0 = SimdHelper::load(values[0]), v7 = SimdHelper::load(values[7]);
    SimdType v1 = SimdHelper::load(values[1]), v6 = SimdHelper::load(values[6]);
    SimdType v2 = SimdHelper::load(values[2]), v5 = SimdHelper::load(values[5]);
    SimdType v3 = SimdHelper::load(values[3]), v4 = SimdHelper::load(values[4]);
    SimdType tmp0 = v0 + v7, tmp7 = v0 - v7;
    SimdType tmp1 = v1 + v6, tmp6 = v1 - v6;
    SimdType tmp2 = v2 + v5, tmp5 = v2 - v5;
    SimdType tmp3 = v3 + v4, tmp4 = v3 - v4;

    SimdType tmp10 = tmp0 + tmp3;
    SimdType tmp13 = tmp0 - tmp3;
    SimdType tmp11 = tmp1 + tmp2;
    SimdType tmp12 = tmp1 - tmp2;

    SimdType half = SimdHelper::populate(1 << (PASS1_BITS - 1));
    store<SimdSize, passNumber, quantizationMode>(result, x, 0, passNumber == 1 ? ((tmp10 + tmp11) << PASS1_BITS) : (tmp10 + tmp11 + half) >> PASS1_BITS, quantizer);
    store<SimdSize, passNumber, quantizationMode>(result, x, 4, passNumber == 1 ? ((tmp10 - tmp11) << PASS1_BITS) : (tmp10 - tmp11 + half) >> PASS1_BITS, quantizer);

    constexpr int descaleShift = passNumber == 1 ? CONST_BITS - PASS1_BITS : CONST_BITS + PASS1_BITS;
#if 0
    ExtendedSimdType z0ext = Int16Helper::mulExtended(tmp12 + tmp13, FIX_0_541196100);
    store<SimdSize, passNumber, quantizationMode>(result, x, 2, (z0ext + SimdHelper::mulExtended(tmp13,  FIX_0_765366865)).template round<descaleShift>(), quantizer);
    store<SimdSize, passNumber, quantizationMode>(result, x, 6, (z0ext + SimdHelper::mulExtended(tmp12, -FIX_1_847759065)).template round<descaleShift>(), quantizer);
#else
    store<SimdSize, passNumber, quantizationMode>(result, x, 2, Int16Helper::template mulAdd<FIX_0_541196100, FIX_0_541196100 + FIX_0_765366865>(tmp12, tmp13).template round<descaleShift>(), quantizer);
    store<SimdSize, passNumber, quantizationMode>(result, x, 6, Int16Helper::template mulAdd<FIX_0_541196100 - FIX_1_847759065, FIX_0_541196100>(tmp12, tmp13).template round<descaleShift>(), quantizer);
#endif

    SimdType z1 = tmp4 + tmp7;
    SimdType z2 = tmp5 + tmp6;
    SimdType z3 = tmp4 + tmp6;
    SimdType z4 = tmp5 + tmp7;
#if 0
    ExtendedSimdType z5ext = SimdHelper::mulExtended(z3 + z4, FIX_1_175875602); /* sqrt(2) * c3 */
    ExtendedSimdType tmp4ext = SimdHelper::mulExtended(tmp4, FIX_0_298631336); /* sqrt(2) * (-c1+c3+c5-c7) */
    ExtendedSimdType tmp5ext = SimdHelper::mulExtended(tmp5, FIX_2_053119869); /* sqrt(2) * ( c1+c3-c5+c7) */
    ExtendedSimdType tmp6ext = SimdHelper::mulExtended(tmp6, FIX_3_072711026); /* sqrt(2) * ( c1+c3+c5-c7) */
    ExtendedSimdType tmp7ext = SimdHelper::mulExtended(tmp7, FIX_1_501321110); /* sqrt(2) * ( c1+c3-c5-c7) */
    ExtendedSimdType z1ext = SimdHelper::mulExtended(z1, -FIX_0_899976223); /* sqrt(2) * ( c7-c3) */
    ExtendedSimdType z2ext = SimdHelper::mulExtended(z2, -FIX_2_562915447); /* sqrt(2) * (-c1-c3) */
    ExtendedSimdType z3ext = SimdHelper::mulExtended(z3, -FIX_1_961570560); /* sqrt(2) * (-c3-c5) */
    ExtendedSimdType z4ext = SimdHelper::mulExtended(z4, -FIX_0_390180644); /* sqrt(2) * ( c5-c3) */

    z3ext += z5ext;
    z4ext += z5ext;

    store<SimdSize, passNumber, quantizationMode>(result, x, 7, (tmp4ext + z1ext + z3ext).template round<descaleShift>(), quantizer);
    store<SimdSize, passNumber, quantizationMode>(result, x, 5, (tmp5ext + z2ext + z4ext).template round<descaleShift>(), quantizer);
    store<SimdSize, passNumber, quantizationMode>(result, x, 3, (tmp6ext + z2ext + z3ext).template round<descaleShift>(), quantizer);
    store<SimdSize, passNumber, quantizationMode>(result, x, 1, (tmp7ext + z1ext + z4ext).template round<descaleShift>(), quantizer);
#else
    SimdType z5 = z3 + z4;
    ExtendedSimdType w7 = Int16Helper::template mulAdd<FIX_0_298631336, FIX_1_175875602>(tmp4, z5) + Int16Helper::template mulAdd<-FIX_0_899976223, -FIX_1_961570560>(z1, z3);
    store<SimdSize, passNumber, quantizationMode>(result, x, 7, w7.template round<descaleShift>(), quantizer);
    ExtendedSimdType w5 = Int16Helper::template mulAdd<FIX_2_053119869, FIX_1_175875602>(tmp5, z5) + Int16Helper::template mulAdd<-FIX_2_562915447, -FIX_0_390180644>(z2, z4);
    store<SimdSize, passNumber, quantizationMode>(result, x, 5, w5.template round<descaleShift>(), quantizer);
    ExtendedSimdType w3 = Int16Helper::template mulAdd<FIX_3_072711026, FIX_1_175875602>(tmp6, z5) + Int16Helper::template mulAdd<-FIX_2_562915447, -FIX_1_961570560>(z2, z3);
    store<SimdSize, passNumber, quantizationMode>(result, x, 3, w3.template round<descaleShift>(), quantizer);
    ExtendedSimdType w1 = Int16Helper::template mulAdd<FIX_1_501321110, FIX_1_175875602>(tmp7, z5) + Int16Helper::template mulAdd<-FIX_0_899976223, -FIX_0_390180644>(z1, z4);
    store<SimdSize, passNumber, quantizationMode>(result, x, 1, w1.template round<descaleShift>(), quantizer);
#endif
  }
}

#ifdef TRANSPOSED_SIMD_BUFFER
template <int SimdSize>
void performBlockDct(const Quantizer& quantizer, int16_t (*values)[Dct::BlockSize][SimdSize], int16_t (*transposedBuffer)[Dct::BlockSize][SimdSize])
{
  processDimension<SimdSize, 1, Unit>(values[0], transposedBuffer, quantizer);
  if (quantizer.isUnit())
    processDimension<SimdSize, 2, Unit>(transposedBuffer[0], values, quantizer);
  else if (quantizer.haveNonPositiveShift())
    processDimension<SimdSize, 2, Slow>(transposedBuffer[0], values, quantizer);
  else
    processDimension<SimdSize, 2, Fast>(transposedBuffer[0], values, quantizer);
}

template <int SimdSize>
void performBlockDct(const Quantizer& quantizer, int32_t (*values)[Dct::BlockSize][SimdSize], int32_t (*transposedBuffer)[Dct::BlockSize][SimdSize])
{
  processDimension<SimdSize, 1, Unit>(values[0], transposedBuffer, quantizer);
  processDimension<SimdSize, 2, Slow>(transposedBuffer[0], values, quantizer);
/*
  for(int k = 0; k < Dct::BlockSize2; k++)
    values[0][k][0] = quantizer.quantize<SimdSize>(values[0][k][0], k);
  values = values;
*/
}

template <typename T, int SimdSize>
void performDct(int simdBlockCount, const EncoderBuffer::MetaData& bufferMetaData, const Quantizer* quantizers, const int* componentQuantizerIndices, T* bufferData)
{
  alignas(32) T transposedBuffer[Dct::BlockSize][Dct::BlockSize][SimdSize];

  for (int n = 0; n < simdBlockCount; n++)
  {
    for (size_t i = 0; i < bufferMetaData.m_components.size(); i++)
    {
      const EncoderBuffer::MetaData::Component& component = bufferMetaData.m_components.at(i);
      const Quantizer& quantizer = quantizers[componentQuantizerIndices[i]];

      for (int j = 0; j < component.m_blockCount; j++)
      {
        T (*values)[Dct::BlockSize][SimdSize] = (T (*)[Dct::BlockSize][SimdSize])(bufferData + (n * bufferMetaData.m_mcuBlockCount + component.m_blockOffset + j) * Dct::BlockSize2 * SimdSize);
        performBlockDct<SimdSize>(quantizer, values, transposedBuffer);
      }
    }
  }
}

}

template<typename T, int SimdLength>
void performDctImplementation(int simdBlockCount, const EncoderBuffer::MetaData& bufferMetaData, const Quantizer* quantizers, const int* componentQuantizerIndices, T* bufferData);

#endif // TRANSPOSED_SIMD_BUFFER

}
