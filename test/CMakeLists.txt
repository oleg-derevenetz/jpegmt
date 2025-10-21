cmake_minimum_required(VERSION 3.16)

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

find_package(Threads REQUIRED)

find_package(JPEG)
if(JPEG_FOUND)
    add_definitions(-DWITH_LIBJPEG)  
    include_directories(${JPEG_INCLUDE_DIRS})
    list(APPEND TEST_SOURCES
        src/JpegLibWrapper.cpp
        src/JpegLibWrapper.h
    )  
endif()

find_package(PNG)
if(PNG_FOUND)
    add_definitions(-DWITH_LIBPNG)  
    include_directories(${PNG_INCLUDE_DIRS})
endif()

add_executable(jpegtest ${TEST_SOURCES})

target_link_libraries(jpegtest jpegmt helper) 

if(JPEG_FOUND)
    target_link_libraries(jpegtest JPEG::JPEG) 
endif()
if(PNG_FOUND)
    target_link_libraries(jpegtest PNG::PNG) 
endif()

target_link_libraries(jpegtest Threads::Threads) 
