cmake_minimum_required(VERSION 3.1)
project(libv-core)

# pour compiler cette bibliothèque
find_package(libv-build REQUIRED)
find_package(Threads REQUIRED)
libv_find_package(roscpp)
libv_find_package(yaml-cpp)
libv_find_package(Boost COMPONENTS iostreams)
libv_find_package(Boost REQUIRED) # cette ligne doit être APRÈS "Boost COMPONENTS"

# pour compiler les tests
libv_find_package(Eigen)
libv_find_package(OpenCV)
libv_find_package(Qt5 COMPONENTS Gui)

# pour compiler les bibliothèques/applications clientes
set(libv_core_cmake "
  find_package(libv-build REQUIRED)
  libv_set_if_greater(CMAKE_CXX_STANDARD 11)
  set(libv_core_CXX_FLAGS_GNU \" -march=native -fopenmp -fvisibility=hidden \")
  set(libv_core_CXX_FLAGS_Clang \" -march=native -fvisibility=hidden \")
  set(libv_core_CXX_FLAGS_Intel \" -xhost -openmp -fvisibility=hidden \")
  set(libv_core_CXX_FLAGS_MSVC \" /DNOMINMAX /D_USE_MATH_DEFINES /D_WINSOCKAPI_ /DWIN32_LEAN_AND_MEAN \")
")

if(NOT Boost_IOSTREAMS_FOUND)
set(Boost_IOSTREAMS_FOUND FALSE)
endif()

libv_auto_library()

include_directories(SYSTEM
  ${LIBV_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_IOSTREAMS_LIBRARIES}
  Threads::Threads
)

if(yaml-cpp_FOUND)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
endif()
