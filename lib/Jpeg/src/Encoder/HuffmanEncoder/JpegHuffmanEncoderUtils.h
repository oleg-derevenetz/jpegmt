#pragma once

#include <cstdint>

#include "../Common.h"

namespace
{

constexpr static inline int getPowerOf2(int n)
{
  return n == 0 || (n > 1 && (n & 1)) ? -1 : (n == 1 ? 0 : (getPowerOf2(n >> 1) == -1 ? -1 : getPowerOf2(n >> 1) + 1));
}

template <typename T>
constexpr static inline int64_t bitWord(int64_t bitIndex)
{
  constexpr int WordBitsPowerOf2 = getPowerOf2(sizeof(T) * 8);
  return bitIndex >> WordBitsPowerOf2;
}
template <typename T>
constexpr static inline int wordBitNum(int64_t bitIndex)
{
  constexpr int WordBitsPowerOf2 = getPowerOf2(sizeof(T) * 8);
  return bitIndex & ((1LL << WordBitsPowerOf2) - 1);
}

FORCE_INLINE static uint64_t toBigEndian(uint64_t source)
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

FORCE_INLINE static uint64_t fromBigEndian(uint64_t source)
{
  return toBigEndian(source);
}

}
