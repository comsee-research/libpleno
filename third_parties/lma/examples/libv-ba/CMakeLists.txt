cmake_minimum_required(VERSION 2.6)

project(test-projection)

find_package(libv-lma REQUIRED)
find_package(libv-geometry REQUIRED)
find_package(libv-graphic REQUIRED)
find_package(Boost COMPONENTS system chrono)
find_package(OpenGL)

include_directories(${CMAKE_PREFIX_PATH})

include(libv_configure_executable)

target_link_libraries(
    ${PROJECT_NAME}
    ${LIBV_LIBRARIES}
    ${Boost_SYSTEM_LIBRARIES}
    ${Boost_CHRONO_LIBRARIES}
    ${OPENGL_LIBRARIES}
)

