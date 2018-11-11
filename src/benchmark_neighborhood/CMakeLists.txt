# make app headers available
include_directories(${CMAKE_CURRENT_SOURCE_DIR})


find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

find_package( Boost COMPONENTS
    system
    filesystem
    program_options
    log
    unit_test_framework
    REQUIRED )

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)

# Create executable

add_executable(benchmark_neighborhood main.cpp)
target_link_libraries(benchmark_neighborhood
    libgooBalls
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_LOG_LIBRARY}
    )

# create a target for the headers, we don't actually need the result
# but this way, IDE's also show header files in their project overviews
file(GLOB_RECURSE header_files "*.h*")
add_custom_target(benchNeighHeaders SOURCES ${header_files})
