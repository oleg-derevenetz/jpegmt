#include "JpegByteStuffingTemplates.h"

namespace Jpeg
{
#ifdef PLATFORM_CPU_FEATURE_INT8x16

template <> int64_t byteStuffingByteCountImplementation<16>(const uint64_t* output, int64_t outputOffset)
{
  return ffByteCount<16>((const int8_t*)output, outputOffset);
}

template <> void byteStuffingImplementation<16>(uint8_t* bytes, int64_t byteCount, int64_t bytesToAdd)
{
  simdByteStuffing<16>(bytes, byteCount, bytesToAdd);
}

template <> uint64_t* allocBitBufferImplementation<16>(int64_t wordCount)
{
  return Platform::Cpu::SIMD<int8_t, 16>::allocMemory<uint64_t>(wordCount);
}

template <> void freeBitBufferImplementation<16>(uint64_t* buffer)
{
  return Platform::Cpu::SIMD<int8_t, 16>::freeMemory(buffer);
}

#else // PLATFORM_CPU_FEATURE_INT8x16

template <> int64_t byteStuffingByteCountImplementation<16>(const uint64_t*, int64_t)
{
  assert(false);
  return 0;
}

template <> void byteStuffingImplementation<16>(uint8_t*, int64_t, int64_t)
{
  assert(false);
}

template <> uint64_t* allocBitBufferImplementation<16>(int64_t)
{
  assert(false);
  return nullptr;
}

template <> void freeBitBufferImplementation<16>(uint64_t*)
{
  assert(false);
}

#endif // PLATFORM_CPU_FEATURE_INT8x16
}
