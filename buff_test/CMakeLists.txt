cmake_minimum_required(VERSION 3.12)

project(main)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g") 
#设置线程选项
# set(CMAKE_CXX_FLAGS -pthread)
# message(STATUS "CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: "${OpenCV_VERSION})
message(STATUS "include path:"${OpenCV_INCLUDE_DIRS})
#添加两个子目录 串口通信模块 二维码识别模块
# aux_source_directory(. DIR_SRCS)


add_executable(main ./fan_tracking.hpp ./fan_tracking.cpp)
target_link_libraries(main ${OpenCV_LIBS})
