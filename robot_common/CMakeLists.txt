cmake_minimum_required(VERSION 3.10)
project(robot_common)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror -Wno-inconsistent-missing-override)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        hardware_interface
        imu_complementary_filter
        imu_filter_madgwick
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        hardware_interface
        imu_complementary_filter
        imu_filter_madgwick
        DEPENDS
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE sources "src/interface/can_interface/*.cpp" "src/utilities/*.cpp")

add_library(${PROJECT_NAME} SHARED ${sources})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#add_executable(test_can
#        test/test_can.cpp)
#
#target_link_libraries(test_can robot_common ${catkin_LIBRARIES})
