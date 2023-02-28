cmake_minimum_required(VERSION 2.6)
project(libv-geometry)

find_package(libv-build REQUIRED)

# Uniquement dans les en-têtes :
libv_find_package(eigen_conversions)
libv_find_package(geometry_msgs)

set(libv_geometry_cmake "
  find_package(libv-build REQUIRED)
  find_package(libv-core REQUIRED)
  find_package(Eigen REQUIRED)
")

libv_auto_library()
