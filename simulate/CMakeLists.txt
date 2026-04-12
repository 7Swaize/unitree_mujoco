cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)


if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -ffast-math -DNDEBUG -fPIC")
endif()

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


target_include_directories(go2sim_cpp PRIVATE
  mujoco/include
  mujoco/simulate
  src/cpp/lodepng
  src/cpp/sensors
  src/cpp
)

target_link_directories(go2sim_cpp PRIVATE mujoco/lib)


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