#include "JpegForwardDctTemplates.h"

namespace Jpeg
{
#ifdef TRANSPOSED_SIMD_BUFFER

#if defined(PLATFORM_CPU_FEATURE_INT16x16)
template<>
void performDctImplementation<int16_t, 16>(int simdBlockCount, const EncoderBuffer::MetaData& bufferMetaData, const Quantizer* quantizers, const int* componentQuantizerIndices, int16_t* bufferData)
{
  return performDct<int16_t, 16>(simdBlockCount, bufferMetaData, quantizers, componentQuantizerIndices, bufferData);
}
#else // PLATFORM_CPU_FEATURE_INT16x16
template<>
void performDctImplementation<int16_t, 16>(int, const EncoderBuffer::MetaData&, const Quantizer*, const int*, int16_t*)
{
  assert(false);
}
#endif // PLATFORM_CPU_FEATURE_INT16x16

#endif // TRANSPOSED_SIMD_BUFFER
}
