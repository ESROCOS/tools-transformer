find_package(Eigen3 REQUIRED NO_MODULE)

include_directories (${EIGEN3_INCLUDE_DIR})

add_library(transformerlib Transformer.cc Frame.cc Transformation.cc)
