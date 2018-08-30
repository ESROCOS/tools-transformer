cmake_minimum_required(VERSION 3.1)

project(transformer_library)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS)

add_definitions(-Wall -Wno-unknown-pragmas)

option(COVERAGE "Enable code coverage. run 'make test && make coverage' to generate the coverage report. The report will be in ${CMAKE_BINARY_DIR}/cov" OFF)

find_package (Eigen3 REQUIRED NO_MODULE)

if(COVERAGE)
  if(CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-fprofile-arcs -ftest-coverage)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -lgcov --coverage -fno-inline -fno-inline-small-functions -fno-default-inline")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -lgcov --coverage -fno-inline -fno-inline-small-functions -fno-default-inline")
    set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -lgcov --coverage -fno-inline -fno-inline-small-functions -fno-default-inline")
    add_custom_target(coverage
                      COMMAND lcov --directory test --capture --output-file cov.info
	              COMMAND lcov --remove cov.info 'test/*' '/usr/*' '*/install/include/*' --output-file cov.info.cleaned
	              COMMAND genhtml -o ./cov cov.info.cleaned
	              COMMAND cmake -E remove cov.info cov.info.cleaned
	              WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	             )
  else()
    message(FATAL_ERROR "Code coverage only works in Debug versions" )
  endif()
endif()


include_directories (${PROJECT_SOURCE_DIR}/include)
include_directories (${EIGEN3_INCLUDE_DIR})

enable_testing ()
add_subdirectory (test)

set(prefix ${CMAKE_INSTALL_PREFIX})
set(exec_prefix ${PROJECT_SOURCE_DIR}/install)
set(libdir ${prefix}/lib)
set(includedir ${prefix}/include/tools/transformer)

configure_file(${PROJECT_SOURCE_DIR}/tools-transformer.pc.in
	       ${PROJECT_SOURCE_DIR}/tools-transformer.pc)


install(FILES ${PROJECT_SOURCE_DIR}/include/Transformer.h 
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/tools/transformer)

install(FILES ${PROJECT_SOURCE_DIR}/tools-transformer.pc
	      DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)
