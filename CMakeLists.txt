cmake_minimum_required(VERSION 2.8.3)
project(eskf_locate)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(locate
  main.cpp
)
target_link_libraries(locate
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
