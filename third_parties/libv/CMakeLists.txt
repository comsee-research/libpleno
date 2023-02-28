# Build all libraries.
# You should not use this.
# You should build only the libraries you need in your project.

cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(libv)
enable_testing()

add_subdirectory(build)
add_subdirectory(core)
add_subdirectory(graphic)
add_subdirectory(geometry)
