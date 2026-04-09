cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -ffast-math -DNDEBUG -fPIC")
endif()

list(APPEND CMAKE_PREFIX_PATH "/home/gsmst/iceoryx2/target/ff/cc/install")
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(iceoryx2-cxx REQUIRED)


include_directories(
  mujoco/include
  mujoco/simulate
  cpp/lodepng
  cpp/sensors
  cpp
)
link_directories(mujoco/lib)

file(GLOB SIM_SRC
  cpp/joystick/joystick.cc

  # exclude mujoco/simulate/main.cc
  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  mujoco/simulate/simulate.cc
  cpp/lodepng/lodepng.cpp

  cpp/sensors/idl/ImageData_.cpp
  cpp/sensors/common/camera_publisher.cpp
  cpp/utils/simd.cpp
)

link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
)

add_executable(unitree_mujoco ${SIM_SRC} cpp/main.cc)

add_executable(jstest cpp/joystick/jstest.cc cpp/joystick/joystick.cc)

target_link_libraries(unitree_mujoco PRIVATE iceoryx2-cxx::static-lib-cxx)
target_link_libraries(jstest PRIVATE iceoryx2-cxx::static-lib-cxx)