cmake_minimum_required(VERSION 3.16)
project(optical_avoidance LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(
    -Wall -Wextra -Wpedantic
    $<$<CONFIG:Release>:-O3>
    $<$<CONFIG:Release>:-mcpu=cortex-a76>
    $<$<CONFIG:Release>:-funroll-loops>
    $<$<CONFIG:Release>:-fomit-frame-pointer>
)

find_package(OpenMP REQUIRED)

find_package(MetavisionSDK
    COMPONENTS base core stream
    REQUIRED
)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

set(ALGO_SOURCES
    src/neighborhood_filter.cpp
    src/obstacle_tracker.cpp
    src/ttc_estimator.cpp
    src/frame_slicer.cpp
    src/adaptive_thresholds.cpp
    src/detection_pipeline.cpp
    src/output_paths.cpp
)

add_executable(optical_avoidance
    src/main.cpp
    src/bias_configurator.cpp
    ${ALGO_SOURCES}
)

target_include_directories(optical_avoidance
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(optical_avoidance
    PRIVATE
        MetavisionSDK::base
        MetavisionSDK::core
        MetavisionSDK::stream
        OpenMP::OpenMP_CXX
)

add_executable(replay_viewer
    src/replay_viewer.cpp
    ${ALGO_SOURCES}
)

target_include_directories(replay_viewer
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(replay_viewer
    PRIVATE
        MetavisionSDK::base
        MetavisionSDK::core
        MetavisionSDK::stream
        opencv_core
        opencv_imgproc
        opencv_highgui
        OpenMP::OpenMP_CXX
)

# Headless benchmark — bez OpenCV, do powtarzalnych pomiarów i sweepów.
add_executable(benchmark
    src/benchmark.cpp
    ${ALGO_SOURCES}
)

target_include_directories(benchmark
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(benchmark
    PRIVATE
        MetavisionSDK::base
        MetavisionSDK::core
        MetavisionSDK::stream
        OpenMP::OpenMP_CXX
)

message(STATUS "Build type    : ${CMAKE_BUILD_TYPE}")
message(STATUS "Metavision SDK: found")
message(STATUS "OpenMP        : ${OpenMP_CXX_VERSION}")
