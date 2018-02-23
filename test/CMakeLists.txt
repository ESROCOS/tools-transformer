find_package(Eigen3 REQUIRED NO_MODULE)

include_directories (${EIGEN3_INCLUDE_DIR})


add_executable(main main.cc)

add_executable(tests_bin unit_tests.cc frame_tests.cc transform_tests.cc transformer_tests.cc)

if(Boost_FOUND)
  message("found boost")
  include_directories(${Boost_INCLUDE_DIRS})
  target_link_libraries(tests_bin ${Boost_LIBRARIES})
endif()
