cmake_minimum_required(VERSION 2.6)
project(convert_camera_model)

find_package(libv-lma REQUIRED)
find_package(libv-geometry REQUIRED)
find_package(libv-graphic REQUIRED)
find_package(libv-io REQUIRED)
find_package(Boost REQUIRED system chrono)
include(libv_configure_executable)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
