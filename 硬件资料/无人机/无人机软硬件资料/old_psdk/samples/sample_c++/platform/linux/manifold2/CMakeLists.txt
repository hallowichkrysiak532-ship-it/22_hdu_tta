cmake_minimum_required(VERSION 3.5)
project(dji_sdk_demo_linux_cxx CXX)

find_package(PkgConfig REQUIRED)

set(CMAKE_C_FLAGS "-pthread -std=gnu99")
set(CMAKE_EXE_LINKER_FLAGS "-pthread")
set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")
add_definitions(-D_GNU_SOURCE)

set(COMMON_CXX_FLAGS "-std=c++11 -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_CXX_FLAGS} -fprofile-arcs -ftest-coverage -Wno-deprecated-declarations")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -lgcov")


include_directories(../../../module_sample)
include_directories(../../../../sample_c/module_sample)
include_directories(../common)
include_directories(../manifold2/application)
include_directories(../manifold2/gcs)
include_directories(../manifold2/proxy_src)
include_directories(../manifold2/ttalink_src)
include_directories(../manifold2/stream_pusher)
include_directories(../manifold2/flight_control)
include_directories(../manifold2/pid)
include_directories(../manifold2/camera_manager)
include_directories(../../../../../ttalink)
include_directories(../../../../../ttalink/common)

file(GLOB_RECURSE MODULE_SAMPLE_SRC
        ../../../module_sample/liveview/*.c*
        ../../../module_sample/perception/*.c*
        ../../../../sample_c/module_sample/*.c*
        )
file(GLOB_RECURSE MODULE_COMMON_SRC ../common/*.c*)
file(GLOB_RECURSE MODULE_HAL_SRC hal/*.c*)
file(GLOB_RECURSE MODULE_APP_SRC application/*.c* gcs/*.c* proxy_src/*.c* ttalink_src/*.c*
pid/*.c* stream_pusher/*.c* flight_control/*.c* camera_manager/*.c* )

execute_process(COMMAND uname -m
        OUTPUT_VARIABLE DEVICE_SYSTEM_ID)
if (DEVICE_SYSTEM_ID MATCHES x86_64)
    set(TOOLCHAIN_NAME x86_64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_x86_64=1)
elseif (DEVICE_SYSTEM_ID MATCHES aarch64)
    set(TOOLCHAIN_NAME aarch64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_aarch64=1)
else ()
    message(FATAL_ERROR "FATAL: Please confirm your platform.")
endif ()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../common/3rdparty)

link_directories(${CMAKE_CURRENT_LIST_DIR}/../../../../../psdk_lib/lib/${TOOLCHAIN_NAME})
link_libraries(${CMAKE_CURRENT_LIST_DIR}/../../../../../psdk_lib/lib/${TOOLCHAIN_NAME}/libpayloadsdk.a -lstdc++)

add_executable(${PROJECT_NAME}
        ${MODULE_APP_SRC}
        # ${MODULE_SAMPLE_SRC}
        ${MODULE_HAL_SRC}
        ${MODULE_COMMON_SRC}
        )

target_include_directories(dji_sdk_demo_linux_cxx PRIVATE
    ${CMAKE_SOURCE_DIR} 
)

# Try to see if user has OpenCV installed
# if yes, default callback will display the image
find_package(OpenCV QUIET)
if (OpenCV_FOUND)
    message("\n${PROJECT_NAME}...")
    message(STATUS "Found OpenCV installed in the system, will use it to display image in AdvancedSensing APIs")
    message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS " - Libraries: ${OpenCV_LIBRARIES}")
    add_definitions(-DOPEN_CV_INSTALLED)
    # 修正版本检查逻辑
    if (OpenCV_VERSION VERSION_LESS "4.0.0")
        add_definitions(-DOPEN_CV_VERSION_3)
    else()
        add_definitions(-DOPEN_CV_VERSION_4)
    endif()
else ()
    message(STATUS "Did not find OpenCV in the system, image data is inside RecvContainer as raw data")
endif ()

find_package(FFMPEG REQUIRED)
if (FFMPEG_FOUND)
    message(STATUS "Found FFMPEG installed in the system")

    #set(FFMPEG_INCLUDE_DIR /usr/include/aarch64-linux-gnu)
    #set(FFMPEG_LIBRARIES /usr/lib/aarch64-linux-gnu/libavcodec.so /usr/lib/aarch64-linux-gnu/libavformat.so /usr/lib/aarch64-linux-gnu/libavutil.so /usr/lib/aarch64-linux-gnu/libswscale.so /usr/lib/aarch64-linux-gnu/libtheora.so /usr/lib/aarch64-linux-gnu/libz.so)

    message(STATUS " - Includes: ${FFMPEG_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${FFMPEG_LIBRARIES}")
    EXECUTE_PROCESS(COMMAND ffmpeg -version
            OUTPUT_VARIABLE ffmpeg_version_psdk_libput
            OUTPUT_STRIP_TRAILING_WHITESPACE
            )
    string(REGEX MATCH "version.*Copyright" ffmpeg_version_line ${ffmpeg_version_psdk_libput})
    string(REGEX MATCH " .* " ffmpeg_version ${ffmpeg_version_line})
    string(REGEX MATCH "^ 5.*$" ffmpeg_major_version ${ffmpeg_version})

    if (ffmpeg_major_version STREQUAL "")
        message(STATUS " - Version: ${ffmpeg_version}")
    else ()
        message(FATAL_ERROR " - Not support FFMPEG version: ${ffmpeg_major_version}, please install 4.x.x instead.")
    endif ()

    target_link_libraries(${PROJECT_NAME} ${FFMPEG_LIBRARIES})
    include_directories(${FFMPEG_INCLUDE_DIR})
    add_definitions(-DFFMPEG_INSTALLED)
else ()
    message(STATUS "Cannot Find FFMPEG")
endif (FFMPEG_FOUND)
include_directories(${CMAKE_CURRENT_LIST_DIR}/../../../../../psdk_lib/include)

# 尝试查找 OPUS 库
find_library(OPUS_LIBRARY opus)
if (OPUS_LIBRARY)
    message(STATUS "Found OPUS installed in the system")
    message(STATUS " - Libraries: ${OPUS_LIBRARY}")
    add_definitions(-DOPUS_INSTALLED)
    target_link_libraries(${PROJECT_NAME} ${OPUS_LIBRARY})
else ()
    message(STATUS "Cannot Find OPUS")
endif ()

# 尝试查找 LIBUSB 库
find_library(LIBUSB_LIBRARY usb-1.0)
if (LIBUSB_LIBRARY)
    message(STATUS "Found LIBUSB installed in the system")
    message(STATUS " - Libraries: ${LIBUSB_LIBRARY}")
    add_definitions(-DLIBUSB_INSTALLED)
    target_link_libraries(${PROJECT_NAME} ${LIBUSB_LIBRARY})
else ()
    message(STATUS "Cannot Find LIBUSB")
endif ()

if (NOT EXECUTABLE_OUTPUT_PATH)
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
endif ()

target_link_libraries(${PROJECT_NAME} m ${GST_LIBRARIES} ${GSTREAMER-APP_LIBRARIES})
#target_link_libraries(${PROJECT_NAME} m)

add_custom_command(TARGET ${PROJECT_NAME}
        PRE_LINK COMMAND cmake ..
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR})
if (OpenCV_FOUND)
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
endif ()

#target_link_libraries(${PROJECT_NAME} /root/tta_ros/lib/libTTARtspPusher.so)
target_link_libraries(${PROJECT_NAME} /usr/lib/libTTARtspPusher.so)

add_definitions(-DTTALINK_USE_CONVENIENCE_FUNCTIONS)
add_definitions(-DTTALINK_SEND_UART_BYTES=ttalink_send_bytes)
add_definitions(-DTTALINK_START_SEND_UART=ttalink_start_send)
add_definitions(-DTTALINK_END_SEND_UART=ttalink_end_send)

add_definitions(-DDRIVER_VER=0.10)
add_definitions(-DSOFT_A=0)
add_definitions(-DSOFT_B=0)
add_definitions(-DSOFT_C=0)
add_definitions(-DSOFT_SVN=1)
