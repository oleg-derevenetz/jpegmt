TARGET = jpegtest
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

CONFIG += precompile_header
PRECOMPILED_HEADER = src/Stable.h

contains(QMAKE_HOST.arch, x86) {
  CONFIG(debug, debug|release) : BUILD_TYPE = debug32
  CONFIG(release, debug|release) : BUILD_TYPE = release32
}
else {
  CONFIG(debug, debug|release) : BUILD_TYPE = debug
  CONFIG(release, debug|release) : BUILD_TYPE = release
}

ROOT_DIR = ..
include($${ROOT_DIR}/Libs.pri)

INCLUDEPATH += $${HELPER_INCLUDE_DIR}
LIBS += -L$${HELPER_LIB_DIR} -lhelper

INCLUDEPATH += $${ROOT_DIR}/lib/Jpeg/include
LIBS += -L$${ROOT_DIR}/lib/Jpeg/$${BUILD_TYPE} -ljpegmt

defined(LIBJPEG_INCLUDE_DIR, var) : defined(LIBJPEG_LIB_DIR, var) {
    DEFINES += WITH_LIBJPEG
    INCLUDEPATH += $${LIBJPEG_INCLUDE_DIR}
    LIBS += -L$${LIBJPEG_LIB_DIR} -ljpeg-static
    SOURCES += src/JpegLibWrapper.cpp
    HEADERS += src/JpegLibWrapper.h
}

defined(LIBPNG_INCLUDE_DIR, var) : defined(LIBPNG_LIB_DIR, var) : defined(LIBPNG_LIB_NAME, var) {
    DEFINES += WITH_LIBPNG
    INCLUDEPATH += $${LIBPNG_INCLUDE_DIR}
    LIBS += -L$${LIBPNG_LIB_DIR} -l$${LIBPNG_LIB_NAME}
    defined(ZLIB_LIB_DIR, var) : LIBS += -L$${ZLIB_LIB_DIR}
    LIBS += -l$${ZLIB_LIB_NAME}
}

SOURCES += \
    src/main.cpp \
    src/ElapsedTimer.cpp \
    src/Image.cpp \
    src/ImageUtils.cpp

HEADERS += \
    src/ElapsedTimer.h \
    src/Image.h \
    src/ImageUtils.h
