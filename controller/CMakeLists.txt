cmake_minimum_required(VERSION 3.0.2)
project(controller)

# Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  can
)

# set(CUDA_INCLUDE_DIRS "/usr/local/cuda-11.3/include")
# set(CMAKE_CUDA_COMPILER "/usr/local/cuda-11.3/bin/nvcc")

# set(Torch_DIR /home/tsingchen/libtorch/share/cmake/Torch)
# find_package(Torch REQUIRED)

# enable_language(CUDA)

###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  # ${TORCH_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(
  ${PROJECT_NAME}
  src/JumpController.cpp
)

# Add cmake target dependencies of the library
# as an example, code may need to be generated before libraries
# either from message generation or dynamic reconfigure
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# Declare a C++ executable
# With catkin_make all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
add_executable(
  JumpController
  src/JumpController.cpp
  src/DeltaMechanism.cpp
  src/NNInterface.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(
  JumpController
  # ${TORCH_LIBRARIES}
  ${catkin_LIBRARIES}
)
# target_include_directories(
#   JumpController
#   PRIVATE
#   CUDA_INCLUDE_DIRS
# )
