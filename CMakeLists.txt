cmake_minimum_required(VERSION 3.0)

project(transformer_library)
set(CMAKE_CXX_FLAGS)
add_definitions(-Wall -Wno-unknown-pragmas)

#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

include_directories(include)

add_subdirectory(src)

#enable_testing()
add_subdirectory(test)
