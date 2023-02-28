cmake_minimum_required(VERSION 2.6)
project(calibrate_camera)

find_package(libv-geometry REQUIRED)
find_package(libv-lma REQUIRED)
libv_find_package(OpenCV REQUIRED calib3d highgui)

libv_auto_executable()
