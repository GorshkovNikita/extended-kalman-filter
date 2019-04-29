cmake_minimum_required(VERSION 3.10)
project(extended_kalman_filter)

set(CMAKE_CXX_STANDARD 14)
set(sources src/main.cpp)



add_executable(extended_kalman_filter ${sources})
target_link_libraries(extended_kalman_filter z ssl uv uWS)
