cmake_minimum_required(VERSION 3.1)

project(gooBalls)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# options
option(ENABLE_OPENMP "Build with OpenMP" ON)

# OpenMP
message("OpenMP: " ${ENABLE_OPENMP})

if(ENABLE_OPENMP)
    find_package(OpenMP)
    if (OPENMP_FOUND)
        message("OpenMP found.")
        set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    else(OPENMP_FOUND)
        message("OpenMP not found.")
    endif(OPENMP_FOUND)
endif(ENABLE_OPENMP)

# Eigen
if(NOT GOOBALLS_EIGEN3_INCLUDE_DIR)
    find_package(Eigen3 QUIET)
    if(EIGEN3_INCLUDE_DIR)
        set(GOOBALLS_EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})
    else()
        set(GOOBALLS_EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ext/nanogui/ext/eigen)
    endif()
endif()

message(STATUS "Using Eigen3 from directory: ${GOOBALLS_EIGEN3_INCLUDE_DIR}")

# NANOGUI
set(NANOGUI_EIGEN_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR} CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_EXAMPLE OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_PYTHON  OFF CACHE BOOL " " FORCE)
set(NANOGUI_INSTALL       OFF CACHE BOOL " " FORCE)

include_directories(ext/nanogui/include)
add_subdirectory(ext/nanogui)

add_definitions(${NANOGUI_EXTRA_DEFS})

set_property(TARGET glfw glfw_objects nanogui PROPERTY FOLDER "dependencies")

# Boost
find_package( Boost COMPONENTS
    system
    filesystem
    program_options
    log
    unit_test_framework
    REQUIRED )

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)

# Box2D
find_package( Box2D REQUIRED )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -Wall -Wextra")

add_subdirectory(src/lib)
add_subdirectory(src/GooBallsApp)
add_subdirectory(src/benchmark_neighborhood)
