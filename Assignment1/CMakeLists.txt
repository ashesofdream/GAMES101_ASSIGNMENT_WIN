cmake_minimum_required(VERSION 3.10)
project(Rasterizer)
set(OpenCV_DIR E:/Program/opencv/build)
find_package(OpenCV REQUIRED)
include_directories(E:/Program/opencv/build/include)
include_directories(E:/Program/eigen3)
set(CMAKE_CXX_STANDARD 17)


add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
