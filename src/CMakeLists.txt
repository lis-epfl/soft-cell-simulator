# CMake Required Version
cmake_minimum_required (VERSION 2.6)

# Project Name
project (SoftCellSimulator)

# Add customized CMAKE modules
set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_SOURCE_DIR}/cmake/")

# Compiler options
add_definitions(-ansi -pedantic -Wno-long-long -Wall -Wextra -Wformat=2
                -Wredundant-decls -Wwrite-strings -Wmissing-include-dirs 
                -Wswitch-enum -Wuninitialized 
                -Wswitch-default -Winit-self -Wshadow -Wfloat-equal -fPIC)

# Debug Flags 
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -ggdb3 -DDEBUG ")

# Release flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -funroll-loops -finline-functions -fomit-frame-pointer -DNDEBUG")

set(BOX2D_INCLUDE_DIR ~/software/etc/include)
set(BOX2D_LIBRARY_DIR ~/software/etc/lib)

# Set the Variables for shared libraries
#set (GLUI_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/libraries/glui/")
set (SOFT_SIMULATOR_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/simulator/")

if (CMAKE_BUILD_TYPE STREQUAL "Release")
       	include(OptimizeForArchitecture)
       	OptimizeForArchitecture("auto")
endif ()

add_subdirectory(simulator)
#add_subdirectory(libraries/glui)
add_subdirectory(examples)

