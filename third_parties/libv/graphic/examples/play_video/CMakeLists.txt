cmake_minimum_required(VERSION 2.6)
project(play_video)

find_package(libv-graphic REQUIRED)
find_package(libv-io REQUIRED)

libv_auto_executable()
