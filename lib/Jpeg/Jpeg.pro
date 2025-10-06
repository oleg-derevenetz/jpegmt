TARGET = jpegmt

CONFIG += precompile_header
CONFIG += staticlib

TEMPLATE = lib

PRECOMPILED_HEADER = src/Stable.h

contains(QMAKE_HOST.arch, x86) {
  CONFIG(debug, debug|release) : BUILD_TYPE = debug32
  CONFIG(release, debug|release) : BUILD_TYPE = release32
}
else {
  CONFIG(debug, debug|release) : BUILD_TYPE = debug
  CONFIG(release, debug|release) : BUILD_TYPE = release
}

DESTDIR = $${BUILD_TYPE}

win32-msvc* {
  CONFIG(debug, debug|release) : QMAKE_CXXFLAGS += -Ob1
  QMAKE_CXXFLAGS += -WX -arch:AVX2
}

linux-g++* {
  QMAKE_CXXFLAGS += -msse4.1 -mavx2 -Werror -Wno-ignored-attributes
}

ROOT_DIR = ../..
include($${ROOT_DIR}/Libs.pri)

INCLUDEPATH += $${HELPER_INCLUDE_DIR} include

SOURCES += \
    src/Encoder/JpegEncoder.cpp \
    src/Encoder/EncoderBuffer/JpegEncoderBuffer.cpp \
    src/Encoder/EncoderBuffer/JpegEncoderBufferInt16x8.cpp \
    src/Encoder/EncoderBuffer/JpegEncoderBufferInt16x16.cpp \
    src/Encoder/EncoderBuffer/JpegEncoderBufferInt32x4.cpp \
    src/Encoder/EncoderBuffer/JpegEncoderBufferInt32x8.cpp \
    src/Encoder/ForwardDct/JpegForwardDct.cpp \
    src/Encoder/ForwardDct/JpegForwardDctInt16x8.cpp \
    src/Encoder/ForwardDct/JpegForwardDctInt16x16.cpp \
    src/Encoder/ForwardDct/JpegForwardDctInt32x4.cpp \
    src/Encoder/ForwardDct/JpegForwardDctInt32x8.cpp \
    src/Encoder/HuffmanEncoder/JpegByteStuffingInt8x16.cpp \
    src/Encoder/HuffmanEncoder/JpegByteStuffingInt8x32.cpp \
    src/Encoder/HuffmanEncoder/JpegHuffmanEncoder.cpp \
    src/Encoder/HuffmanEncoder/JpegHuffmanEncoderInt16x8.cpp \
    src/Encoder/HuffmanEncoder/JpegHuffmanEncoderInt16x16.cpp \
    src/Encoder/JpegQuantizer.cpp \
    src/JpegDCT.cpp \
    src/JpegFileFormat.cpp \
    src/JpegThreadPool.cpp \
    src/JpegWriter.cpp

# internal headers
HEADERS += \
    src/Encoder/Common.h \
    src/Encoder/JpegEncoder.h \
    src/Encoder/EncoderBuffer/JpegEncoderBuffer.h \
    src/Encoder/EncoderBuffer/JpegEncoderBufferTemplates.h \
    src/Encoder/EncoderBuffer/RgbToYcc.h \
    src/Encoder/ForwardDct/JpegForwardDct.h \
    src/Encoder/ForwardDct/JpegForwardDctTemplates.h \
    src/Encoder/HuffmanEncoder/JpegByteStuffingTemplates.h \
    src/Encoder/HuffmanEncoder/JpegHuffmanEncoder.h \
    src/Encoder/HuffmanEncoder/JpegHuffmanEncoderTemplates.h \
    src/Encoder/HuffmanEncoder/JpegHuffmanEncoderUtils.h \
    src/Encoder/JpegQuantizer.h \
    src/JpegFileFormat.h

# interface headers
HEADERS += \
    include/Jpeg/JpegDCT.h \
    include/Jpeg/JpegImageMetaData.h \
    include/Jpeg/JpegThreadPool.h \
    include/Jpeg/JpegWriter.h \
    include/Jpeg/Rgb.h
