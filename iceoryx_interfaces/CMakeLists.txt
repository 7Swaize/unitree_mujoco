cmake_minimum_required(VERSION 3.16)
project(iceoryx_interfaces LANGUAGES CXX)

add_library(iceoryx_interfaces INTERFACE)

find_package(iceoryx2-cxx REQUIRED)

target_include_directories(iceoryx_interfaces INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/cpp/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(iceoryx_interfaces INTERFACE iceoryx2-cxx::shared-lib-cxx) # header only -> dependencies are propagated to consumers

install(DIRECTORY src/cpp/include/ DESTINATION include)
install(TARGETS iceoryx_interfaces EXPORT iceoryx_interfacesTargets) # allow find package
install(EXPORT iceoryx_interfacesTargets
    FILE iceoryx_interfacesTargets.cmake
    NAMESPACE iceoryx_interfaces::
    DESTINATION lib/cmake/iceoryx_interfaces
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    cmake/iceoryx_interfacesConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/iceoryx_interfacesConfig.cmake
    INSTALL_DESTINATION lib/cmake/iceoryx_interfaces
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/iceoryx_interfacesConfig.cmake
    DESTINATION lib/cmake/iceoryx_interfaces
)