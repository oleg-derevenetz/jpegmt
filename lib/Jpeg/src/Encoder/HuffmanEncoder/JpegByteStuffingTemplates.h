#include <cassert>

#include <Helper/Platform/Cpu/cpu.h>
#include <Helper/Platform/Cpu/intrinsics.h>
#include <Helper/Platform/Cpu/simd.h>

#include "JpegHuffmanEncoderUtils.h"

namespace
{

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

constexpr bool revertByteStuffingByteOrder()
{
  return Platform::Cpu::byteOrder != Platform::Cpu::BigEndian;
}

static int64_t wordByteStuffing(uint64_t word, uint8_t* bytes, int64_t bytesToAdd)
{
  constexpr bool revertBytes = revertByteStuffingByteOrder();
  if (!(word & 0x8080808080808080ull & ~(word + 0x0101010101010101ull))) // may be false negative but that is acceptable
  {
    if (revertBytes)
      word = toBigEndian(word);
#if 0
    for (int j = 7; j >= 0; j--)
      bytes[bytesToAdd + j] = (uint8_t)((word >> (j * 8)) & 0xff);
#else
    * (uint64_t*)(bytes + bytesToAdd) = word;
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
    bytesToAdd = wordByteStuffing(words[w], bytes + w * 8, bytesToAdd);
#endif

  return bytesToAdd;
}

template<int SimdLength>
void simdByteStuffing(uint8_t* bytes, int64_t byteCount, int64_t bytesToAdd)
{
  using namespace Platform::Cpu;
  typedef typename SIMD<int8_t, SimdLength>::Type SimdType;
  typedef typename SIMD<int64_t, SimdLength / 8>::Type SimdType64;
  int64_t simdCount = byteCount / SimdLength;
  SimdType ff = SimdType::populate((int8_t)0xff);
  constexpr bool revertBytes = revertByteStuffingByteOrder();

  if (simdCount * SimdLength < byteCount)
    bytesToAdd = byteStuffing64(bytes + simdCount * SimdLength, byteCount - simdCount * SimdLength, bytesToAdd);

  for (int64_t i = simdCount - 1; i >= 0; i--)
  {
    SimdType simdBytes = SimdType::load((int8_t*)bytes + i * SimdLength);
    if (!(simdBytes == ff).bitMask())
      (revertBytes ? SimdType{SimdType64{simdBytes}.revertedByteOrder()} : simdBytes).template store<false>((int8_t*)bytes + i * SimdLength + bytesToAdd);
    else
    {
      uint64_t* words = (uint64_t*)(bytes + i * SimdLength);
      for (int j = SimdLength / sizeof(uint64_t) - 1; j >= 0; j--)
        bytesToAdd = wordByteStuffing(words[j], (uint8_t*)(words + j), bytesToAdd);
    }
  }
  assert(bytesToAdd == 0);
}

}

namespace Jpeg
{
  template <int SimdLength> int64_t byteStuffingByteCountImplementation(const uint64_t* output, int64_t outputOffset);
  template <int SimdLength> void byteStuffingImplementation(uint8_t* bytes, int64_t byteCount, int64_t bytesToAdd);
  template <int SimdLength> uint64_t* allocBitBufferImplementation(int64_t wordCount);
  template <int SimdLength> void freeBitBufferImplementation(uint64_t* buffer);
}
