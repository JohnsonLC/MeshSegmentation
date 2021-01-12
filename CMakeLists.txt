cmake_minimum_required(VERSION 3.5)
project(gragh_hw)

add_definitions(-std=c++11)
add_definitions(-Wall)

# 用于调试
# add_definitions(-O0)

# add_subdirectory(assimp)
include_directories(include)

add_subdirectory(src bin)

