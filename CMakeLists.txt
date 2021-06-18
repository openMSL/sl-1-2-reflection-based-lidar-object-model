cmake_minimum_required(VERSION 3.12)

file(READ src/model/model_name.conf MODEL_NAME)
string(REPLACE "\n" "" MODEL_NAME ${MODEL_NAME})

project(${MODEL_NAME})

set(FMU_INSTALL_DIR "${CMAKE_BINARY_DIR}" CACHE PATH "Target directory for generated FMU")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(lib/open-simulation-interface)
include_directories(lib/fmi2 lib/eigen/Eigen)
add_subdirectory(src/model)
add_subdirectory(src/osmp)
