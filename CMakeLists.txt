cmake_minimum_required(VERSION 3.16)

project(rocket_flightgen LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

add_library(fsw STATIC
    fsw/src/flight_state.cpp
)

target_include_directories(fsw PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/fsw
)

add_executable(fsw_tests
    test/test_main.cpp
)

target_link_libraries(fsw_tests PRIVATE fsw)

add_test(NAME fsw_tests COMMAND fsw_tests)