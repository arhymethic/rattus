cmake_minimum_required(VERSION 3.12)
project(MicromouseSimulator VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

add_definitions(-DSIMULATION)

set(SOURCES
    src/main.cpp
    src/API.cpp
    src/Sensors.cpp
    src/Motors.cpp
    src/FloodFill.cpp
)

add_executable(sim_mouse ${SOURCES})
