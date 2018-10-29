# add this path to include path of library files
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# list here your source files (*.cpp) of your module
set(source_files
    generic/hello.cpp
)

add_library(libgooBalls ${source_files})

# add this path to include path of target
target_include_directories(libgooBalls PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

find_package( Boost COMPONENTS
    system
    filesystem
    log
    unit_test_framework
    REQUIRED )

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)

target_link_libraries(libgooBalls
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_LOG_LIBRARY}
    )
# list here your testing files (*.test.cc) of your module
set(test_files
    test_root.cc
    generic/hello.test.cc
    )
add_executable(tests ${test_files})
target_link_libraries(tests
    libgooBalls
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)

# create a target for the headers, we don't actually need the result
# but this way, IDE's also show header files in their project overviews
file(GLOB_RECURSE header_files "*.h*")
add_custom_target(headers SOURCES ${header_files})


