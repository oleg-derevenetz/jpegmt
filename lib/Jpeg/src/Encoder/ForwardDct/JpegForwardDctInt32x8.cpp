#include "JpegForwardDctTemplates.h"

namespace Jpeg
{
#ifdef TRANSPOSED_SIMD_BUFFER

#if defined(PLATFORM_CPU_FEATURE_INT32x8)
template<>
void performDctImplementation<int32_t, 8>(int simdBlockCount, const EncoderBuffer::MetaData& bufferMetaData, const Quantizer* quantizers, const int* componentQuantizerIndices, int32_t* bufferData)
{
  return performDct<int32_t, 8>(simdBlockCount, bufferMetaData, quantizers, componentQuantizerIndices, bufferData);
}
#else // PLATFORM_CPU_FEATURE_INT32x8
template<>
void performDctImplementation<int32_t, 8>(int, const EncoderBuffer::MetaData&, const Quantizer*, const int*, int32_t*)
{
  assert(false);
}
#endif // PLATFORM_CPU_FEATURE_INT32x8

#endif // TRANSPOSED_SIMD_BUFFER
}
