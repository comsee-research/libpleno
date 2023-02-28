cmake_minimum_required(VERSION 2.6)
project(draw_random_3d)

find_package(libv-graphic REQUIRED)
find_package(OpenGL REQUIRED)

libv_auto_executable()

target_link_libraries(${PROJECT_NAME}
  ${OPENGL_gl_LIBRARY}
  )
