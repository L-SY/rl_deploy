cmake_minimum_required(VERSION 3.10)
project(diablo_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        robot_common
        controller_manager
        urdf
)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        robot_common
        controller_manager
        urdf
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/DiabloHW.cpp
        src/DIabloHWLoop.cpp
        src/lib/crc_verify.cpp
        src/lib/serial_handle.cpp
        src/lib/SerialPort/SerialPort.cpp
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

add_library(diablo_sdk
        src/lib/crc_verify.cpp
        src/lib/serial_handle.cpp
        src/lib/SerialPort/SerialPort.cpp
)

target_link_libraries(diablo_sdk
        ${catkin_LIBRARIES}
)

add_executable(diablo_hw_run
        src/diablo_hw.cpp
        src/DiabloHW.cpp
        src/DIabloHWLoop.cpp
        src/lib/crc_verify.cpp
        src/lib/serial_handle.cpp
        src/lib/SerialPort/SerialPort.cpp
)

target_link_libraries(diablo_hw_run
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

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
