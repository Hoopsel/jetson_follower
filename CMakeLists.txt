cmake_minimum_required(VERSION 2.8.3)
project(jetson_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)

# Find OpenCV
find_package(OpenCV 2.4.8 REQUIRED)
if (OpenCV_FOUND)
    message("-- OpenCV ${OpenCV_VERSION} was found")
    include_directories(${OpenCV_INCLUDE_DIRS})
else(OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV was not found")
endif(OpenCV_FOUND)

# Find CUDA
find_package(CUDA 6.5 REQUIRED)
if (CUDA_FOUND)
    message("-- CUDA ${CUDA_VERSION} was found")
    include(FindCUDA)
    include_directories(${CUDA_INCLUDE_DIRS})
else(CUDA_FOUND)
    message(FATAL_ERROR "CUDA was not found")
endif(CUDA_FOUND)

# Find Boost
find_package(Boost 1.54 REQUIRED system chrono)
if (Boost_FOUND)
    message("-- Boost ${Boost_VERSION} was found")
    include_directories(${Boost_INCLUDE_DIRS})
else(Boost_FOUND)
    message(FATAL_ERROR "Boost was not found")
endif(Boost_FOUND)

# QT
find_package(Qt4 REQUIRED QtCore QtGui)

# Prepare Qt for cmake
include(${QT_USE_FILE})

# Run moc automatically when needed
set(CMAKE_AUTOMOC ON)

# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Add definitions
add_definitions(${Qt4_DEFINITIONS})

# Source dir
remove_definitions(-DPROJECT_SOURCE_DIR)
add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

## Catkin depends
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jetson_follower
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge
#  DEPENDS system_lib
)

include_directories(
  include/jetson_follower
  ${catkin_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Qt wrap
qt4_wrap_ui(UI_SRCS ui/calibration.ui)

# Pass options to NVCC
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -gencode arch=compute_30,code=sm_30 --ptxas-options=-v --maxrregcount=32)

# Add cuda library for files using cuda methods
cuda_add_library(jetson_follower_cuda SHARED include/jetson_follower/gpu_utils.cuh src/gpu_utils.cu)
target_link_libraries(jetson_follower_cuda ${OpenCV_LIBS} -lboost_system -lboost_chrono -lpthread -lpng)

# C++11 flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# determine HSV
add_executable(jetson_calibration
  src/main_calibration.cpp
  src/calibration.cpp include/jetson_follower/calibration.h
  src/ros_wrapper.cpp include/jetson_follower/ros_wrapper.h
  ${UI_SRCS})
target_link_libraries(jetson_calibration ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS} -lyaml-cpp)

# track!
add_executable(jetson_follower src/main_follower.cpp src/follower.cpp include/jetson_follower/follower.h)
target_link_libraries(jetson_follower jetson_follower_cuda ${catkin_LIBRARIES} ${OpenCV_LIBS} -lyaml-cpp)
