cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(shapeOperator)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

add_executable (shapeOperator shapeOperator.cpp)
target_link_libraries (shapeOperator ${PCL_LIBRARIES})

