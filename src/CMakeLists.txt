find_package(Eigen3 REQUIRED NO_MODULE)

INCLUDE_DIRECTORIES (${EIGEN3_INCLUDE_DIR})

add_library(transformerlib Transformer.cc Frame.cc Transformation.cc)

#target_link_libraries(transformerlib Eigen3::Eigen)
