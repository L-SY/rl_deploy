cmake_minimum_required(VERSION 3.0)

project(jointctrlsdk)

find_package(Threads REQUIRED)

include_directories(inc)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
    src/crc_verify.cpp
    src/serial_handle.cpp
    src/SerialPort/SerialPort.cpp
)


add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${library_name})
target_link_libraries(${PROJECT_NAME} -lpthread -lm)
install(TARGETS jointctrlsdk RUNTIME DESTINATION bin)
