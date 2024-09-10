cmake_minimum_required(VERSION 3.0.2)
project(rl_sdk)

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Torch REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
)

# find_package(yaml-cpp REQUIRED)
link_directories(/usr/local/lib)
include_directories(${YAML_CPP_INCLUDE_DIR})

catkin_package(
        CATKIN_DEPENDS
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        library/matplotlibcpp
        library/observation_buffer
        library/loop
)

add_library(rl_sdk src/rl_sdk.cpp)
target_link_libraries(rl_sdk "${TORCH_LIBRARIES}")
set_property(TARGET rl_sdk PROPERTY CXX_STANDARD 14)

add_library(observation_buffer library/observation_buffer/observation_buffer.cpp)
target_link_libraries(observation_buffer "${TORCH_LIBRARIES}")
set_property(TARGET observation_buffer PROPERTY CXX_STANDARD 14)