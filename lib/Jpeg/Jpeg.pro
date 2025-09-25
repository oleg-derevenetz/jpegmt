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

win32-msvc* {
  CONFIG(debug, debug|release) : QMAKE_CXXFLAGS += -Ob1
  QMAKE_CXXFLAGS += /arch:AVX2
}

ROOT_DIR = ../..
include($${ROOT_DIR}/Libs.pri)

INCLUDEPATH += $${HELPER_INCLUDE_DIR} include

SOURCES += \
    src/Encoder/JpegEncoder.cpp \
    src/Encoder/JpegEncoderBuffer.cpp \
    src/Encoder/JpegForwardDct.cpp \
    src/Encoder/JpegHuffmanEncoder.cpp \
    src/Encoder/JpegQuantizer.cpp \
    src/JpegDCT.cpp \
    src/JpegFileFormat.cpp \
    src/JpegThreadPool.cpp \
    src/JpegWriter.cpp

# internal headers
HEADERS += \
    src/Encoder/JpegEncoder.h \
    src/Encoder/JpegEncoderBuffer.h \
    src/Encoder/JpegForwardDct.h \
    src/Encoder/JpegHuffmanEncoder.h \
    src/Encoder/JpegQuantizer.h \
    src/Encoder/RgbToYcc.h \
    src/JpegFileFormat.h

# interface headers
HEADERS += \
    include/Jpeg/JpegDCT.h \
    include/Jpeg/JpegImageMetaData.h \
    include/Jpeg/JpegThreadPool.h \
    include/Jpeg/JpegWriter.h \
    include/Jpeg/Rgb.h
