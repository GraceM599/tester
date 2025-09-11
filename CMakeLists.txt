cmake_minimum_required(VERSION 3.10)
project(tester)

set(CMAKE_CXX_STANDARD 14)

# Original test files
add_executable(original_tester
        imu_testreader.cpp)

add_executable(imu_test2
        imu_test2.cpp)

add_executable(working_i2c_test
        working_i2c_test.cpp)

add_executable(spi_test
        spi_test.cpp)

add_executable(hardware_debug
        hardware_debug.cpp)

add_executable(sensor_probe
        sensor_probe.cpp)

add_executable(direct_cs_test
        direct_cs_test.cpp)

add_executable(i2c_test
        i2c_test.cpp)

add_executable(wiring_check
        wiring_check.cpp)

# Fixed version
add_executable(fixed_register_reader
        fixed_register_reader.cpp)

# Comprehensive diagnostic tool
add_executable(comprehensive_diagnostic
        comprehensive_diagnostic.cpp)
