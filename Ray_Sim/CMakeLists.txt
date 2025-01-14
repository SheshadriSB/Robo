cmake_minimum_required(VERSION 3.10)
project(test)

find_package(raylib REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} raylib)

# Add Wayland-specific libraries if needed
target_link_libraries(${PROJECT_NAME})
