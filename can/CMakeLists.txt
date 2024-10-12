cmake_minimum_required(VERSION 3.10)
project(can)

## Use C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -Wno-inconsistent-missing-override)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation
        )

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        motor_data.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        message_runtime
)

# add_dependencies(${PROJECT_NAME}_user_msg_pub ${PROJECT_NAME}_generate_messages_cpp)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(
        ${PROJECT_NAME}
        src/WholeBodyDriver.cpp
        )

add_executable(
        WholeBodyDriver
        src/WholeBodyDriver.cpp
)

add_dependencies(
        WholeBodyDriver
        ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(
        WholeBodyDriver
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
