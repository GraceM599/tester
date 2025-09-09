cmake_minimum_required(VERSION 4.0)
project(tester)

set(CMAKE_CXX_STANDARD 14)

add_executable(tester
        imu_testreader.cpp)
