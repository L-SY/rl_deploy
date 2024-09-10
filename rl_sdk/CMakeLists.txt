cmake_minimum_required(VERSION 3.0.2)
project(rl_sdk)

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

find_package(Torch REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
)
catkin_package(
        INCLUDE_DIRS
        include
        library/matplotlibcpp
        library/observation_buffer
        library/loop
        CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        LIBRARIES
        ${PROJECT_NAME}
        DEPENDS
)

# find_package(yaml-cpp REQUIRED)
link_directories(/usr/local/lib)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        library/matplotlibcpp
        library/observation_buffer
        library/loop
        ${YAML_CPP_INCLUDE_DIR}
        ${TORCH_INCLUDE_DIRS}
)

add_library(rl_sdk src/rl_sdk.cpp library/observation_buffer/observation_buffer.cpp)
target_link_libraries(rl_sdk "${TORCH_LIBRARIES}")
set_property(TARGET rl_sdk PROPERTY CXX_STANDARD 14)
