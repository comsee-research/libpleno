#==============================================================================
#         Copyright 2015 INSTITUT PASCAL UMR 6602 CNRS/Univ. Clermont II
#
#          Distributed under the Boost Software License, Version 1.0.
#                 See accompanying file LICENSE.txt or copy at
#                     http://www.boost.org/LICENSE_1_0.txt
#==============================================================================

cmake_minimum_required(VERSION 2.6)
project(animate_sphere)

find_package(libv-lma REQUIRED)
find_package(libv-geometry REQUIRED)
find_package(libv-graphic REQUIRED)
find_package(Boost REQUIRED chrono system date_time thread)
find_package(OpenGL REQUIRED)

include(libv_configure_executable)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES})

