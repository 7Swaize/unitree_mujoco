cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

include(CheckCXXSourceRuns)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")

find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(iceoryx_interfaces REQUIRED CONFIG)


file(GLOB SIM_SRC
  src/cpp/joystick/joystick.cc

  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  mujoco/simulate/simulate.cc

  src/cpp/lodepng/lodepng.cpp

  src/cpp/sensors/idl/ImageData_.cpp
  src/cpp/sensors/common/camera_publisher.cpp
  src/cpp/utils/simd.cpp
)

add_executable(go2sim_cpp ${SIM_SRC} src/cpp/main.cc)

# Checks from: https://github.com/ipc-sim/IPC/tree/main/cmake/find
check_cxx_source_runs("
    #include <immintrin.h>
    int main() {
        __m256i a = _mm256_set_epi32 (-1, 2, -3, 4, -1, 2, -3, 4);
        __m256i result = _mm256_abs_epi32 (a);
        return 0;
    }" DETECTED_AVX2)

check_cxx_source_runs("
    #include <immintrin.h>
    int main() {
        __m256d a = _mm256_set_pd (-1, 2, -3, 4);
        __m256d b = _mm256_set_pd (-2, 3, -4, 1);
        __m256d c = _mm256_set_pd (-11, 6, 4, -1);
        __m256d result = _mm256_fmsub_pd (a, b, c);
        return 0;
    }" DETECTED_FMA)

check_cxx_source_runs("
    #include <emmintrin.h>
    #include <nmmintrin.h>
    int main() {
        long long a[2] = { 1, 2 };
        long long b[2] = { -1, 3 };
        long long c[2];
        __m128i va = _mm_loadu_si128((__m128i*)a);
        __m128i vb = _mm_loadu_si128((__m128i*)b);
        __m128i vc = _mm_cmpgt_epi64(va, vb);
        _mm_storeu_si128((__m128i*)c, vc);
        return (c[0] == -1LL && c[1] == 0LL) ? 0 : 1;
    }" DETECTED_SSE42)

check_cxx_source_runs("
    #include <arm_neon.h>
    int main() {
        uint32x4_t v = vdupq_n_u32(0);
        return 0;
    }" DETECTED_NEON)


if(DETECTED_AVX2)
  message(STATUS "SIMD: Enabling AVX2")
  if(MSVC)
    target_compile_options(go2sim_cpp PRIVATE /arch:AVX2)
  else()
    target_compile_options(go2sim_cpp PRIVATE -mavx2)
  endif()
endif()

if(DETECTED_FMA)
  message(STATUS "SIMD: Enabling FMA")
  if(NOT MSVC)
    target_compile_options(go2sim_cpp PRIVATE -mfma)
  endif()
endif()

if(DETECTED_SSE42 AND NOT DETECTED_AVX2)
  message(STATUS "SIMD: Enabling SSE4.2")
  if(MSVC)
    target_compile_options(go2sim_cpp PRIVATE /arch:SSE4.2) # Sure, SSE4.2 is also bundled with AVX
  else()
    target_compile_options(go2sim_cpp PRIVATE -msse4.2)
  endif()
endif()

if(DETECTED_NEON)
  message(STATUS "SIMD: Enabling NEON")
  if(NOT MSVC AND CMAKE_SIZEOF_VOID_P EQUAL 4) # 32-bit arch requires -mfpu=neon flag 
    target_compile_options(go2sim_cpp PRIVATE -mfpu=neon)
  endif()
endif()


if(MSVC)
  target_compile_options(go2sim_cpp PRIVATE
    /O2
    /fp:fast
    /DNDEBUG
    # DLLs do not use PICs: https://stackoverflow.com/questions/13309662/enable-pic-equivalent-in-visual-studio
  )
else()
  target_compile_options(go2sim_cpp PRIVATE
    -O3
    -ffast-math
    -DNDEBUG
    -fPIC
  )
endif()



target_include_directories(go2sim_cpp PRIVATE
  mujoco/include
  mujoco/simulate
  src/cpp/lodepng
  src/cpp/sensors
  src/cpp
)

target_link_directories(go2sim_cpp PRIVATE mujoco/lib)

target_link_libraries(go2sim_cpp PRIVATE
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  Boost::program_options
  fmt
  iceoryx_interfaces::iceoryx_interfaces
)


install(TARGETS go2sim_cpp
  RUNTIME DESTINATION go2sim/cpp
)

install(FILES
  mujoco/lib/libmujoco.so.3.3.6
  DESTINATION go2sim/lib
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/resources/
  DESTINATION go2sim/resources
)