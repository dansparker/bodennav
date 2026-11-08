cmake_minimum_required(VERSION 3.20)
project(bodennav VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(BODENNAV_WITH_SOAPY "Use SoapySDR for the SDR receivers" ON)
option(BODENNAV_WERROR "Treat warnings as errors" OFF)
option(BODENNAV_SANITIZE "Build with address and undefined behaviour sanitizers" OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
  if(BODENNAV_WERROR)
    add_compile_options(-Werror)
  endif()
  if(BODENNAV_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
  endif()
endif()

find_package(Threads REQUIRED)

# Hardware independent part: signal processing, decoders, CANaerospace, simulator.
add_library(bodennav_core STATIC
  src/dsp/dsp.cpp
  src/dsp/morse.cpp
  src/nav/frequency.cpp
  src/nav/vor.cpp
  src/nav/ils.cpp
  src/nav/marker.cpp
  src/nav/adf.cpp
  src/can/canaerospace.cpp
  src/sim/siggen.cpp
  src/io/file_source.cpp
  src/app/config.cpp
  src/app/state.cpp
)
target_include_directories(bodennav_core PUBLIC src)

add_executable(bodennav
  src/app/main.cpp
  src/app/receiver.cpp
  src/can/socketcan.cpp
  src/io/sdr_source.cpp
)
target_link_libraries(bodennav PRIVATE bodennav_core Threads::Threads)

if(BODENNAV_WITH_SOAPY)
  find_package(SoapySDR CONFIG QUIET)
  if(SoapySDR_FOUND)
    message(STATUS "SoapySDR ${SoapySDR_VERSION}: SDR receivers enabled")
    target_compile_definitions(bodennav PRIVATE BODENNAV_HAVE_SOAPY)
    target_include_directories(bodennav PRIVATE ${SoapySDR_INCLUDE_DIRS})
    target_link_libraries(bodennav PRIVATE ${SoapySDR_LIBRARIES})
  else()
    message(WARNING "SoapySDR not found: only IQ files can be used (apt install libsoapysdr-dev)")
  endif()
endif()

add_executable(bodennav-siggen tools/siggen_main.cpp)
target_link_libraries(bodennav-siggen PRIVATE bodennav_core)

include(CTest)
if(BUILD_TESTING)
  add_executable(bodennav_tests
    tests/test_main.cpp
    tests/test_dsp.cpp
    tests/test_nav.cpp
    tests/test_can.cpp
  )
  target_link_libraries(bodennav_tests PRIVATE bodennav_core)
  add_test(NAME bodennav_tests COMMAND bodennav_tests)
endif()

install(TARGETS bodennav bodennav-siggen RUNTIME DESTINATION bin)
