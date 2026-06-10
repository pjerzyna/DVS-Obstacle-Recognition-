cmake_minimum_required(VERSION 3.16)
project(optical_avoidance LANGUAGES CXX)

# ─────────────────────────────────────────────
#  Standardy i flagi kompilacji
# ─────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Profil Release: agresywna optymalizacja na Cortex-A76 (RPi 5)
# -mcpu=cortex-a76     → instrukcje specyficzne dla rdzenia RPi 5
# -mfpu=neon-fp-armv8  → włącza jednostkę NEON (SIMD, 128-bit)
# -ffast-math          → relaxed IEEE 754, pozwala auto-wektoryzować float
# -funroll-loops       → rozwijanie pętli wewnętrznych filtra sąsiedztwa
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(
    -Wall -Wextra -Wpedantic
    $<$<CONFIG:Release>:-O3>
    $<$<CONFIG:Release>:-mcpu=cortex-a76>
    # $<$<CONFIG:Release>:-mfpu=neon-fp-armv8>
    $<$<CONFIG:Release>:-ffast-math>
    $<$<CONFIG:Release>:-funroll-loops>
    $<$<CONFIG:Release>:-fomit-frame-pointer>
)

# ─────────────────────────────────────────────
#  OpenMP (wielowątkowość na 4 rdzeniach)
# ─────────────────────────────────────────────
find_package(OpenMP REQUIRED)

# ─────────────────────────────────────────────
#  Metavision / OpenEB SDK
#  Ścieżki domyślne po kompilacji OpenEB na RPi OS Bookworm
# ─────────────────────────────────────────────
find_package(MetavisionSDK
    COMPONENTS base core stream
    REQUIRED
)
# Alternatywna ścieżka jeśli SDK jest w /usr/local:
# list(APPEND CMAKE_PREFIX_PATH "/usr/local")

# ─────────────────────────────────────────────
#  OpenCV (operacje macierzowe 320×320)
# ─────────────────────────────────────────────
find_package(OpenCV 4 REQUIRED
    COMPONENTS core imgproc
)

# ─────────────────────────────────────────────
#  Źródła projektu
# ─────────────────────────────────────────────
set(SOURCES
    src/main.cpp
    src/bias_configurator.cpp
    src/neighborhood_filter.cpp
    src/obstacle_tracker.cpp
    src/ttc_estimator.cpp
)

add_executable(optical_avoidance ${SOURCES})

target_include_directories(optical_avoidance
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(optical_avoidance
    PRIVATE
        MetavisionSDK::base
        MetavisionSDK::core
	MetavisionSDK::stream
        ${OpenCV_LIBS}
        OpenMP::OpenMP_CXX
)

# ─────────────────────────────────────────────
#  Informacja buildowa
# ─────────────────────────────────────────────
message(STATUS "Build type    : ${CMAKE_BUILD_TYPE}")
message(STATUS "Metavision SDK: found")
message(STATUS "OpenCV ver    : ${OpenCV_VERSION}")
message(STATUS "OpenMP        : ${OpenMP_CXX_VERSION}")
