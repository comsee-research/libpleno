#==============================================================================
#         Copyright 2015 INSTITUT PASCAL UMR 6602 CNRS/Univ. Clermont II
#
#          Distributed under the Boost Software License, Version 1.0.
#                 See accompanying file LICENSE.txt or copy at
#                     http://www.boost.org/LICENSE_1_0.txt
#==============================================================================

cmake_minimum_required(VERSION 2.6)

project(libv-lma)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})

find_package(libv-core REQUIRED)
find_package(Boost 1.51 REQUIRED chrono system)
find_package(Eigen REQUIRED)

include(libv_configure_library)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
