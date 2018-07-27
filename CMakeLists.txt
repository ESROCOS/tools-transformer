cmake_minimum_required(VERSION 3.1)

project(transformer_library)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS)

add_definitions(-Wall -Wno-unknown-pragmas)

include_directories(${PROJECT_SOURCE_DIR}/include)

#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

add_subdirectory(src)

enable_testing()
add_subdirectory(test)

install(FILES ${PROJECT_SOURCE_DIR}/include/Transformer.h
	DESTINATION ${CMAKE_INSTALL_PREFIX}/include/tools/transformer)

install(FILES ${PROJECT_SOURCE_DIR}/tools-transformer.pc
	DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)
