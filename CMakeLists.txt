cmake_minimum_required(VERSION 2.8.11)
include(CheckSymbolExists)

#========================
# Project details / setup
#========================

# Extract version from header, done first to satisfy CMP0048,
# see `cmake --help-policy CMP0048` for more information.
file(
	STRINGS                         # Read in a file to memory.
	pleno/version.h           		# File to parse for version number.
	PLENO_PROJECT_VERSION          	# Where to store the results (should just be one line)
	REGEX "#define PLENO_VERSION " 	# The space after is important to ignore 'PLENO_VERSION_INT'
)
# If no matches found, something is wrong with PointMatcher.h
if (NOT PLENO_PROJECT_VERSION)
	message(SEND_ERROR "Cannot find version number in '${CMAKE_CURRENT_SOURCE_DIR}/pleno/version.h'.")
endif (NOT PLENO_PROJECT_VERSION)
# Transform '#define PLENO_VERSION "X.Y.Z"' into 'X.Y.Z'
string(REGEX REPLACE ".*\"(.*)\".*" "\\1" PLENO_PROJECT_VERSION "${PLENO_PROJECT_VERSION}")

# In 3.0+, project(...) should specify VERSION to satisfy CMP0048
if (CMAKE_VERSION VERSION_LESS 3.0.0)
	project(libpleno)
else (CMAKE_VERSION VERSION_LESS 3.0.0)
	cmake_policy(SET CMP0048 NEW)
	project(libpleno VERSION ${PLENO_PROJECT_VERSION})
endif (CMAKE_VERSION VERSION_LESS 3.0.0)

# Ensure proper build type
if (NOT CMAKE_BUILD_TYPE)
  message("-- No build type specified; defaulting to CMAKE_BUILD_TYPE=Release.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
else (NOT CMAKE_BUILD_TYPE)
  if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("\n=================================================================================")
    message("\n-- Build type: Debug. Performance will be terrible!")
    message("-- Add -DCMAKE_BUILD_TYPE=Release to the CMake command line to get an optimized build.")
    message("\n=================================================================================")
  endif (CMAKE_BUILD_TYPE STREQUAL "Debug")
endif (NOT CMAKE_BUILD_TYPE)


#================= extra building definitions ==============================
if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-O3)
endif(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")

add_definitions(-fconcepts)

# For Windows
if( MSVC )
	add_definitions( /D _VARIADIC_MAX=10 ) # VS2012 does not support tuples correctly yet
	add_definitions( /D _USE_MATH_DEFINES) # defines M_PI for Visual Studio
	add_definitions( /D _ENABLE_EXTENDED_ALIGNED_STORAGE) # this variable must be defined with VS2017 to acknowledge alignment changes of aligned_storage
endif()

#======================= installation =====================================

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")

if(WIN32 AND NOT CYGWIN)
	set(DEF_INSTALL_CMAKE_DIR CMake)
else()
	set(DEF_INSTALL_CMAKE_DIR lib/cmake/pleno)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files")


# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
	set(var INSTALL_${p}_DIR)
	if(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	endif()
endforeach()

#===========================================================================

#======================== External Dependencies ===============================

# initially
set(EXTERNAL_LIBS "")
set(EXTRA_DEPS "")

# compile local version of lib
#add_subdirectory(contrib)

#--------------------
# DEPENDENCY:  libv-core, libv-graphic, libv-geometry, libv-lma
#--------------------
find_package(libv REQUIRED core lma graphic geometry)
include_directories(${LIBV_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${LIBV_LIBRARIES})

#--------------------
# DEPENDENCY: eigen 3
#--------------------
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${EIGEN})

find_path(EIGEN_INCLUDE_DIR Eigen/Core
	/usr/local/include/eigen3
	/usr/include/eigen3
)
# Suppress Eigen's warning by adding it to the system's library
include_directories(SYSTEM "${EIGEN_INCLUDE_DIR}")

#--------------------
# DEPENDENCY: OpenCV
#--------------------
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${OpenCV_LIBS})

#--------------------
# DEPENDENCY: OpenGL
#--------------------
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${OPENGL_LIBRARIES})

#--------------------
# DEPENDENCY: OpenMP (optional)
#--------------------
set(USE_OPEN_MP FALSE CACHE BOOL "Set to TRUE to use OpenMP")
if (USE_OPEN_MP)
	find_package(OpenMP)
	if (OPENMP_FOUND)
		add_definitions(-fopenmp -DHAVE_OPENMP)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		if (CMAKE_COMPILER_IS_GNUCC)
			set(EXTERNAL_LIBS ${EXTERNAL_LIBS} gomp)
    	message("-- OpenMP found, parallel computer enabled")
		endif(CMAKE_COMPILER_IS_GNUCC)
	endif(OPENMP_FOUND)
endif (USE_OPEN_MP)
#============================= end dependencies =================================


#========================== libpleno itself ==============================

# Check the compiler version as we need full C++11 support.
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	# using Clang
	if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "3.3")
		message(WARNING, "Your clang compiler has version ${CMAKE_CXX_COMPILER_VERSION}, while only version 3.3 or later is supported")
	endif ()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
	# using AppleClang
	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "5.1")
		message(WARNING "Your XCode environment has version ${CMAKE_CXX_COMPILER_VERSION}, while only version 5.1 or later is supported")
	endif()
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	# using GCC
	if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "4.8.2")
		message(WARNING, "Your g++ compiler has version ${CMAKE_CXX_COMPILER_VERSION}, while only version 4.8.2 or later is supported")
	endif ()
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	# using MSVC
	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "19.0.23506")
		message(WARNING "Your Microsoft Visual C++ compiler has version ${CMAKE_CXX_COMPILER_VERSION}, while only version MSVC 2015 Update 1+ or later is supported")
	endif()
endif ()

# enable C++11 support.
if (CMAKE_VERSION VERSION_LESS "3.8")
	if (MSVC)
		message(FATAL_ERROR, "CMake version 3.8 or later is required to compiler ${PROJECT_NAME} with Microsoft Visual C++")
	endif ()
	if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		set (CMAKE_CXX_FLAGS "-std=c++1z ${CMAKE_CXX_FLAGS}")
	else ()
		set (CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
	endif ()
else ()
	set (CMAKE_CXX_STANDARD 17)
endif ()

# SOURCE

# Pleno lib and install
set(PLENO_GEOMETRY_SRC
	pleno/geometry/transformation.cpp
	pleno/geometry/distortions.cpp
	pleno/geometry/mesh.cpp
	pleno/geometry/pose.cpp
	pleno/geometry/ray.cpp
	pleno/geometry/sensor.cpp
	pleno/geometry/observation.cpp
	pleno/geometry/internals.cpp
	pleno/geometry/mia.cpp
	pleno/geometry/mla.cpp
	pleno/geometry/plane.cpp
	pleno/geometry/rgba.cpp
#--Camera	
	pleno/geometry/camera/camera.cpp
	pleno/geometry/camera/unified.cpp
	pleno/geometry/camera/orthographic.cpp
	pleno/geometry/camera/pinhole.cpp
	pleno/geometry/camera/thinlens.cpp
	pleno/geometry/camera/plenoptic.cpp
#--Model
	pleno/geometry/model/microimage.cpp	
#--Depth	
	pleno/geometry/depth/depthmap.cpp
	pleno/geometry/depth/pointcloud.cpp
	pleno/geometry/depth/depthimage.cpp
)

set(PLENO_OPTIMIZATION_SRC
	pleno/optimization/pose2d.cpp
	pleno/optimization/pose3d.cpp
	pleno/optimization/grid3d.cpp
	pleno/optimization/mia.cpp
	pleno/optimization/mla.cpp
	pleno/optimization/focal.cpp
	pleno/optimization/distortions.cpp
	pleno/optimization/thinlens.cpp
	pleno/optimization/sensor.cpp
	pleno/optimization/homography.cpp
	pleno/optimization/kappa.cpp
	pleno/optimization/depth.cpp
	pleno/optimization/functions.cpp
#--Errors
	pleno/optimization/errors/grid.cpp
	pleno/optimization/errors/mic.cpp
	pleno/optimization/errors/bap.cpp
	pleno/optimization/errors/corner.cpp
	pleno/optimization/errors/radius.cpp
	pleno/optimization/errors/blur.cpp
	pleno/optimization/errors/extrinsics.cpp
	pleno/optimization/errors/invdistortion.cpp
	pleno/optimization/errors/disparity.cpp
	pleno/optimization/errors/scaling.cpp
	pleno/optimization/errors/lidarcamera.cpp
)

set(PLENO_IO_SRC
	pleno/io/printer.cpp
	pleno/io/images.cpp
	pleno/io/depths.cpp
)

set(PLENO_GRAPHIC_SRC
	pleno/graphic/viewer_3d.cpp
	pleno/graphic/viewer_2d.cpp
	pleno/graphic/gui.cpp
	pleno/graphic/draw.cpp
	pleno/graphic/colormap.cpp
)

set(PLENO_PROCESSING_SRC
	pleno/processing/estimation.cpp
#--Tools
	pleno/processing/tools/chrono.cpp	
#--Precalibration
	pleno/processing/precalibration/preprocess.cpp
	pleno/processing/precalibration/display.cpp
#--Calibration
	pleno/processing/calibration/mia.cpp	
	pleno/processing/calibration/plenoptic.cpp	
	pleno/processing/calibration/extrinsics.cpp	
	pleno/processing/calibration/blur.cpp	
	pleno/processing/calibration/invdistortion.cpp
	pleno/processing/calibration/scaling.cpp
	pleno/processing/calibration/lidarcamera.cpp
#--Detection
	pleno/processing/detection/mic.cpp
	pleno/processing/detection/corners/corners.cpp
		pleno/processing/detection/corners/caracterize.cpp
		pleno/processing/detection/corners/optimize.cpp
		pleno/processing/detection/corners/clusterize.cpp
	pleno/processing/detection/bapfeatures.cpp
#--Algorithm
	pleno/processing/algorithms/p3p.cpp
#--Imgprocessing
	pleno/processing/imgproc/access.cpp
	pleno/processing/imgproc/devignetting.cpp
	pleno/processing/imgproc/debayering.cpp
	pleno/processing/imgproc/interp.cpp
	pleno/processing/imgproc/morph.cpp
	pleno/processing/imgproc/trim.cpp
	pleno/processing/imgproc/quality.cpp
#--Depth
	pleno/processing/depth/depth.cpp
	pleno/processing/depth/filter.cpp
	pleno/processing/depth/neighbors.cpp
	pleno/processing/depth/pairing.cpp
	pleno/processing/depth/search.cpp
	pleno/processing/depth/initialization.cpp
	pleno/processing/depth/compute.cpp
	pleno/processing/depth/strategy.cpp
)

set(PLENO_SIMULATION_SRC
#--Generate
	pleno/simulation/generate/pose.cpp
	pleno/simulation/generate/mic.cpp
	pleno/simulation/generate/corner.cpp
	pleno/simulation/generate/bapfeatures.cpp
	pleno/simulation/generate/image.cpp 
)

set(PLENO_SRC
	${PLENO_GEOMETRY_SRC}
	${PLENO_OPTIMIZATION_SRC}
	${PLENO_IO_SRC}
	${PLENO_GRAPHIC_SRC}
	${PLENO_PROCESSING_SRC}
	${PLENO_SIMULATION_SRC}	
)

include_directories(${CMAKE_SOURCE_DIR}/pleno)
include_directories(${CMAKE_SOURCE_DIR}/pleno/geometry)
include_directories(${CMAKE_SOURCE_DIR}/pleno/optimization)
include_directories(${CMAKE_SOURCE_DIR}/pleno/graphic)
include_directories(${CMAKE_SOURCE_DIR}/pleno/io)
include_directories(${CMAKE_SOURCE_DIR}/pleno/processing)
include_directories(${CMAKE_SOURCE_DIR}/pleno/simulation)

file(GLOB_RECURSE PLENO_HEADERS "pleno/*.h" "pleno/*.hpp")

# MacOS and Windows deal with shared/dynamic library differently. For
# simplicity, we only authorize static library in those cases.
if(APPLE OR WIN32)
	add_library(pleno ${PLENO_SRC} ${PLENO_HEADERS} )
	install(TARGETS pleno ARCHIVE DESTINATION ${INSTALL_LIB_DIR})

else(APPLE OR WIN32)
	set(SHARED_LIBS "true" CACHE BOOL "To build shared (true) or static (false) library")

	if (SHARED_LIBS)
		add_library(pleno SHARED ${PLENO_SRC} ${PLENO_HEADERS})
		install(TARGETS pleno
						ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
						LIBRARY DESTINATION ${INSTALL_LIB_DIR}
						RUNTIME DESTINATION ${INSTALL_BIN_DIR})
	else(SHARED_LIBS)
		add_library(pleno ${PLENO_SRC} ${PLENO_HEADERS} )
		install(TARGETS pleno ARCHIVE DESTINATION ${INSTALL_LIB_DIR})
	endif(SHARED_LIBS)
endif(APPLE OR WIN32)

target_include_directories(pleno PRIVATE ${CMAKE_SOURCE_DIR}/pleno)
target_include_directories(pleno PRIVATE ${CMAKE_SOURCE_DIR}/pleno/geometry)
target_include_directories(pleno PRIVATE ${CMAKE_SOURCE_DIR}/pleno/optimization)
target_include_directories(pleno PRIVATE ${CMAKE_SOURCE_DIR}/pleno/graphic)
target_include_directories(pleno PRIVATE ${CMAKE_SOURCE_DIR}/pleno/io)
target_include_directories(pleno PRIVATE ${CMAKE_SOURCE_DIR}/pleno/processing)
target_include_directories(pleno PRIVATE ${CMAKE_SOURCE_DIR}/pleno/simulation)

# link all libraries to libpleno
add_definitions(-Wall -Wextra -Werror)
if (NOT OPENMP_FOUND)
	add_definitions(-Wno-unknown-pragmas)
endif ()

target_link_libraries(pleno ${EXTERNAL_LIBS} pthread)

if(EXTRA_DEPS)
	add_dependencies(pleno ${EXTRA_DEPS})
endif(EXTRA_DEPS)
set_target_properties(pleno PROPERTIES VERSION "${PLENO_PROJECT_VERSION}" SOVERSION 1)


#========================= Install header ===========================
foreach(_file ${PLENO_HEADERS})
	get_filename_component(_dir ${_file} PATH)
	file(RELATIVE_PATH _dir ${PROJECT_SOURCE_DIR}/pleno/ ${_dir})
	install(FILES ${_file} DESTINATION ${INSTALL_INCLUDE_DIR}/pleno/${_dir})
endforeach()

#========================= Documentation ===========================

set(GENERATE_API_DOC false CACHE BOOL "Set to true to build the documentation using Doxygen")

if(GENERATE_API_DOC)

	message("-- API Documentation (doxygen): enabled")

	# Note: there seems to be equations in the documentation leading to the
	# use of Latex anyway. This cause problems for user without latex...

	set(DOXYFILE_LATEX "NO")
	include(UseDoxygen.cmake)

	set(DOC_INSTALL_TARGET "share/doc/${PROJECT_NAME}/api" CACHE STRING "Target where to install doxygen documentation")

	add_dependencies(pleno doc)

	install(FILES README.md DESTINATION share/doc/${PROJECT_NAME})
	if (DOXYGEN_FOUND)
		install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/html DESTINATION ${DOC_INSTALL_TARGET})
	endif (DOXYGEN_FOUND)

else(GENERATE_API_DOC)

	message("-- API Documentation (doxygen): disabled")

endif(GENERATE_API_DOC)


#=============== trigger other makefile ======================

# Example programs
#add_subdirectory(examples)
# Evaluation programs
#add_subdirectory(evaluations)

# Unit testing
option(BUILD_TESTS "Build all tests." OFF)
if (BUILD_TESTS)
	enable_testing()
	add_subdirectory(utest)
endif()

#=================== allow find_package() =========================
#
# the following case be used in an external project requiring libpleno:
#  ...
#  find_package(libpleno)
#  include_directories(${libpleno_INCLUDE_DIRS})
#  target_link_libraries(executableName ${libpleno_LIBRARIES})
#  ...

# 1- local build #

# Register the local build in case one doesn't use "make install"
export(PACKAGE libpleno)

# Create variable for the local build tree
get_property(CONF_INCLUDE_DIRS DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

# Create variable with the library location
get_target_property(PLENO_LIB pleno LOCATION)

# Configure config file for local build tree
configure_file(tools.in/libplenoConfig.cmake.in
  "${PROJECT_BINARY_DIR}/libplenoConfig.cmake" @ONLY)

# 2- installation build #

# Change the include location for the case of an install location
set(CONF_INCLUDE_DIRS ${INSTALL_INCLUDE_DIR} ${CONF_INCLUDE_DIRS} )

# Change the library location for an install location
get_filename_component(PLENO_LIB_NAME ${PLENO_LIB} NAME)
set(PLENO_LIB ${INSTALL_LIB_DIR}/${PLENO_LIB_NAME})

# We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
configure_file(tools.in/libplenoConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libplenoConfig.cmake" @ONLY)

# The same versioning file can be used for both cases
configure_file(tools.in/libplenoConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/libplenoConfigVersion.cmake" @ONLY)


# Install the libplenoConfig.cmake and libplenoConfigVersion.cmake
install(
  FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libplenoConfig.cmake"
    "${PROJECT_BINARY_DIR}/libplenoConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev
)

install (
  FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libplenoConfig.cmake"
    "${PROJECT_BINARY_DIR}/libplenoConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake/"
)

#Handle pkg-config file
set(LIBRARY_CC_ARGS "")
foreach(_LIB IN LISTS PLENO_LIB EXTERNAL_LIBS)
  get_filename_component(_FILE_NAME ${_LIB} NAME)
  if(${_FILE_NAME} STREQUAL ${_LIB}) # not an absolute path
    set(LIBRARY_CC_ARGS "${LIBRARY_CC_ARGS} -l${_LIB}")
  else()
    set(LIBRARY_CC_ARGS "${LIBRARY_CC_ARGS} ${_LIB}")
  endif()
endforeach()
unset(_LIB)
unset(_FILE_NAME)

configure_file(tools.in/pleno.pc.in libpleno.pc @ONLY)
configure_file(tools.in/pleno.pc.in pleno.pc @ONLY) # for backward compatibility
install(FILES
  ${CMAKE_BINARY_DIR}/libpleno.pc
  ${CMAKE_BINARY_DIR}/pleno.pc # for backward compatibility
  DESTINATION lib/pkgconfig
)
unset(LIBRARY_CC_ARGS)

#====================== uninstall target ===============================
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/tools.in/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
