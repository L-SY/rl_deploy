cmake_minimum_required(VERSION 3.10)
project(ps5_joy)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -Wno-enum-compare)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
)

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        DEPENDS
        LIBRARIES
        ${PROJECT_NAME}
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/ps5Joy.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(joy_twist src/joyTwist.cpp)

target_link_libraries(joy_twist ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(joy_rl src/joyRLCommand.cpp)

target_link_libraries(joy_rl ${catkin_LIBRARIES} ${PROJECT_NAME})