cmake_minimum_required(VERSION 2.6)
project(libv-graphic)

find_package(libv-build REQUIRED)
find_package(libv-core REQUIRED)
find_package(Boost REQUIRED)
find_package(Qt5 REQUIRED Core Gui Widgets OpenGL PrintSupport Svg)

# pour compiler l'interface ROS
libv_collect_prefix_path()
libv_find_package(four_wheel_steering_msgs)
libv_find_package(rviz)

include_directories(
  ${LIBV_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
  ${Qt5PrintSupport_INCLUDE_DIRS}
  ${Qt5Svg_INCLUDE_DIRS}
)

set(libv_graphic_cmake "
  find_package(libv-core REQUIRED)
")

libv_auto_library()

target_link_libraries(${PROJECT_NAME}
  ${Qt5Core_LIBRARIES}
  ${Qt5Gui_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5OpenGL_LIBRARIES}
  ${Qt5PrintSupport_LIBRARIES}
  ${Qt5Svg_LIBRARIES}
  )

