cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
SET(CMAKE_BUILD_TYPE "Release")
add_definitions("-Wall -g -O0")
find_package(Eigen3  REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(eskf_lib
	src/eskf_interface.cpp
  src/eskf_helper.cpp
)

target_include_directories(eskf_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(eskf_lib
	${EIGEN3_LIBS}
)

