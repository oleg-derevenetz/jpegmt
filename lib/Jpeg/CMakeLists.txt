cmake_minimum_required(VERSION 3.16)

project(jpegmt CXX)

set(HELPER_LIB_INCLUDE_PATH "${jpegmt_SOURCE_DIR}/../Helper/include" CACHE STRING "")

include_directories("${jpegmt_SOURCE_DIR}/include" "${HELPER_LIB_INCLUDE_PATH}")

# sources
set(JPEGMT_SOURCES 
    src/Encoder/JpegEncoder.cpp
    src/Encoder/EncoderBuffer/JpegEncoderBuffer.cpp
    src/Encoder/EncoderBuffer/JpegEncoderBufferInt16x8.cpp
    src/Encoder/EncoderBuffer/JpegEncoderBufferInt32x4.cpp
    src/Encoder/ForwardDct/JpegForwardDct.cpp
    src/Encoder/ForwardDct/JpegForwardDctInt16x8.cpp
    src/Encoder/ForwardDct/JpegForwardDctInt32x4.cpp
    src/Encoder/HuffmanEncoder/JpegByteStuffingInt8x16.cpp
    src/Encoder/HuffmanEncoder/JpegHuffmanEncoder.cpp
    src/Encoder/HuffmanEncoder/JpegHuffmanEncoderInt16x8.cpp
    src/Encoder/JpegQuantizer.cpp
    src/JpegDCT.cpp
    src/JpegFileFormat.cpp
    src/JpegThreadPool.cpp
    src/JpegWriter.cpp
)

set(JPEGMT_SSE41_SOURCES
)

set(JPEGMT_AVX2_SOURCES
    src/Encoder/EncoderBuffer/JpegEncoderBufferInt16x16.cpp
    src/Encoder/EncoderBuffer/JpegEncoderBufferInt32x8.cpp
    src/Encoder/ForwardDct/JpegForwardDctInt16x16.cpp
    src/Encoder/ForwardDct/JpegForwardDctInt32x8.cpp
    src/Encoder/HuffmanEncoder/JpegByteStuffingInt8x32.cpp
    src/Encoder/HuffmanEncoder/JpegHuffmanEncoderInt16x16.cpp
)

# internal headers
list(APPEND JPEGMT_SOURCES
    src/Encoder/Common.h
    src/Encoder/JpegEncoder.h
    src/Encoder/EncoderBuffer/JpegEncoderBuffer.h
    src/Encoder/EncoderBuffer/JpegEncoderBufferTemplates.h
    src/Encoder/EncoderBuffer/RgbToYcc.h
    src/Encoder/ForwardDct/JpegForwardDct.h
    src/Encoder/ForwardDct/JpegForwardDctTemplates.h
    src/Encoder/HuffmanEncoder/JpegByteStuffingTemplates.h
    src/Encoder/HuffmanEncoder/JpegHuffmanEncoder.h
    src/Encoder/HuffmanEncoder/JpegHuffmanEncoderTemplates.h
    src/Encoder/HuffmanEncoder/JpegHuffmanEncoderUtils.h
    src/Encoder/JpegQuantizer.h
    src/JpegFileFormat.h
)

# interface headers
list(APPEND JPEGMT_SOURCES
    include/Jpeg/JpegDCT.h
    include/Jpeg/JpegImageMetaData.h
    include/Jpeg/JpegThreadPool.h
    include/Jpeg/JpegWriter.h
    include/Jpeg/Rgb.h
)

add_library(jpegmt STATIC ${JPEGMT_SOURCES})
add_library(jpegmt_avx2 STATIC ${JPEGMT_AVX2_SOURCES})

if (MSVC)
  set(COMPILER_OPTIONS -WX)
  set(SSE41_COMPILER_OPTIONS ${COMPILER_OPTIONS})
  set(AVX2_COMPILER_OPTIONS ${COMPILER_OPTIONS} -arch:AVX2)
else()
  set(COMPILER_OPTIONS -Werror -Wno-ignored-attributes)
  set(SSE41_COMPILER_OPTIONS ${COMPILER_OPTIONS} -msse4.1)
  set(AVX2_COMPILER_OPTIONS ${COMPILER_OPTIONS} -mavx2)
endif()

target_compile_options(jpegmt PRIVATE ${COMPILER_OPTIONS})
target_compile_options(jpegmt_avx2 PRIVATE ${AVX2_COMPILER_OPTIONS})
target_link_libraries(jpegmt PRIVATE jpegmt_avx2)
