SET(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8.2-a+simd+fp16+dotprod -mtune=cortex-a55 -O3")
#use eigen optimization

add_library(lidar_matcher SHARED
        src/lidar_matcher.cpp
)
# use ceres optimization
# find_package(Ceres REQUIRED)
# include_directories(${Ceres_INCLUDE_DIRS})

# sophus
add_definitions(-DSOPHUS_USE_BASIC_LOGGING=ON) # sophus: eliminate fmt
include_directories(third_party/sophus)

# eigen
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
target_include_directories(lidar_matcher PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${PROJECT_SOURCE_DIR}/common
        ${PCL_INCLUDE_DIRS}
        ${CMAKE_CURRENT_LIST_DIR}/third_party/sophus
)
target_link_libraries(lidar_matcher
  ${PCL_LIBRARIES}
#   ${CERES_LIBRARIES}
  stdc++fs
)


find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
