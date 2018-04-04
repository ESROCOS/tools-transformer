pkg_search_module(Eigen3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN3_DEFINITIONS})

if(NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

add_executable(main main.cc)

add_executable(tests_bin unit_tests.cc frame_tests.cc transform_tests.cc transformer_tests.cc)

if(Boost_FOUND)
  message("found boost")
  include_directories(${Boost_INCLUDE_DIRS})
  target_link_libraries(tests_bin ${Boost_LIBRARIES})
endif()
