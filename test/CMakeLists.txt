cmake_minimum_required(VERSION 3.26)

project(jpegtest CXX)

include_directories("${jpegmt_SOURCE_DIR}/include" "${helper_SOURCE_DIR}/include")

# sources
set(TEST_SOURCES 
    src/main.cpp
    src/ElapsedTimer.cpp
    src/Image.cpp
    src/ImageUtils.cpp
)

# headers
list(APPEND TEST_SOURCES
    src/ElapsedTimer.h
    src/Image.h
    src/ImageUtils.h
)

set(LIBJPEG_PATH "" CACHE PATH "directory of jpeg/jpeg-turbo library installation")
find_package(libjpeg-turbo PATHS "${LIBJPEG_PATH}")
if(libjpeg-turbo_FOUND)
    add_definitions(-DWITH_LIBJPEG)  
    include_directories(${libjpeg-turbo_INCLUDE_DIRS})
    list(APPEND TEST_SOURCES
        src/JpegLibWrapper.cpp
        src/JpegLibWrapper.h
    )  
endif()

set(LIBPNG_PATH "" CACHE PATH "directory of png library installation")
find_package(PNG PATHS "${LIBPNG_PATH}")
if(PNG_FOUND)
    add_definitions(-DWITH_LIBPNG)  
    include_directories(${PNG_INCLUDE_DIRS})
endif()

add_executable(jpegtest ${TEST_SOURCES})

target_link_libraries(jpegtest jpegmt helper) 

if(libjpeg-turbo_FOUND)
    target_link_libraries(jpegtest libjpeg-turbo::jpeg-static) 
endif()
if(PNG_FOUND)
    target_link_libraries(jpegtest PNG::png_static) 
endif()
