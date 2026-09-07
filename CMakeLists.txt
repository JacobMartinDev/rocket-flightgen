cmake_minimum_required(VERSION 3.16)
project(flight_computer_hil LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

add_executable(fsw_tests test/test_main.cpp)
add_test(NAME fsw_tests COMMAND fsw_tests)