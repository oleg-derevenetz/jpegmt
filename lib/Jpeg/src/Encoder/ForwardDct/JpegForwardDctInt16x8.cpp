#include "JpegForwardDctTemplates.h"

namespace Jpeg
{
#ifdef TRANSPOSED_SIMD_BUFFER

#if defined(PLATFORM_CPU_FEATURE_INT16x8)
template<>
void performDctImplementation<int16_t, 8>(int simdBlockCount, const EncoderBuffer::MetaData& bufferMetaData, const Quantizer* quantizers, const int* componentQuantizerIndices, int16_t* bufferData)
{
  return performDct<int16_t, 8>(simdBlockCount, bufferMetaData, quantizers, componentQuantizerIndices, bufferData);
}
#else // PLATFORM_CPU_FEATURE_INT16x8
template<>
void performDctImplementation<int16_t, 8>(int, const EncoderBuffer::MetaData&, const Quantizer*, const int*, int16_t*)
{
  assert(false);
}
#endif // PLATFORM_CPU_FEATURE_INT16x8

#endif // TRANSPOSED_SIMD_BUFFER
}
