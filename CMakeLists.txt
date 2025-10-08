cmake_minimum_required(VERSION 4.0)
project(UART_PI_communication)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(SCServo_Linux)
add_executable(Servo_Communication
        com.cpp)

target_include_directories(Servo_Communication PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/SCServo_Linux
)

target_link_libraries(Servo_Communication PRIVATE SCServo)

