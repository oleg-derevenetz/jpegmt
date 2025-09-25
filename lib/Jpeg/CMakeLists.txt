project(jpegmt CXX)

set(HELPER_LIB_INCLUDE_PATH "${jpegmt_SOURCE_DIR}/../Helper/include" CACHE STRING "")

include_directories("${jpegmt_SOURCE_DIR}/include" "${HELPER_LIB_INCLUDE_PATH}")

# sources
set(JPEGMT_SOURCES 
    src/Encoder/JpegEncoder.cpp
    src/Encoder/JpegEncoderBuffer.cpp
    src/Encoder/JpegForwardDct.cpp
    src/Encoder/JpegHuffmanEncoder.cpp
    src/Encoder/JpegQuantizer.cpp
    src/JpegDCT.cpp
    src/JpegFileFormat.cpp
    src/JpegThreadPool.cpp
    src/JpegWriter.cpp
)

# internal headers
list(APPEND JPEGMT_SOURCES
    src/Encoder/JpegEncoder.h
    src/Encoder/JpegEncoderBuffer.h
    src/Encoder/JpegForwardDct.h
    src/Encoder/JpegHuffmanEncoder.h
    src/Encoder/JpegQuantizer.h
    src/Encoder/RgbToYcc.h
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
