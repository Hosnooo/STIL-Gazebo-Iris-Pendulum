# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(DronePendulum)

# Find Gazebo and Ignition libraries
find_package(gazebo REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-transport8 REQUIRED)

# Add include directories for Gazebo and Ignition
include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${IGNITION_COMMON4_INCLUDE_DIRS}
    ${IGNITION_MATH6_INCLUDE_DIRS}
    ${IGNITION_TRANSPORT8_INCLUDE_DIRS}
)

# Add the linker directories for Gazebo and Ignition
link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    ${IGNITION_COMMON4_LIBRARY_DIRS}
    ${IGNITION_MATH6_LIBRARY_DIRS}
    ${IGNITION_TRANSPORT8_LIBRARY_DIRS}
)

# Set the C++ standard and any additional compile options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Add the StabController Library (include all .cpp files in src/StabController)
# file(GLOB STABCONTROLLER_SOURCES ../src/StabController/*.cpp)
# add_library(StabController SHARED ${STABCONTROLLER_SOURCES})

# Add the Pendulum Plugin
add_library(gazebo_wind_plugin SHARED gazebo_wind_plugin.cpp)
target_link_libraries(gazebo_wind_plugin
    # StabController
    ${GAZEBO_LIBRARIES}
    ${IGNITION_COMMON4_LIBRARIES}
    ${IGNITION_MATH6_LIBRARIES}
    ${IGNITION_TRANSPORT8_LIBRARIES}
)

add_subdirectory(StabController)
add_subdirectory(TracController)
# target_link_libraries(StabControllerPlugin TracController)

# Add the Pendulum Plugin
add_library(Mission_plugin SHARED MissionPlugin.cc)
target_link_libraries(Mission_plugin
    ${GAZEBO_LIBRARIES}
    ${IGNITION_COMMON4_LIBRARIES}
    ${IGNITION_MATH6_LIBRARIES}
    ${IGNITION_TRANSPORT8_LIBRARIES}
    StabController
    TracController
)