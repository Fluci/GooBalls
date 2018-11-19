# make app headers available
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# list here all source files appart from main
set(source_files
    placeholder.cpp
    )
# list here your testing files (*.test.cc) of your module
set(test_files
    test_root.cc
    )

# compile everything into a library and then link into the target executable (tests, main, etc.)
# this should increase compilation speed
add_library(libgooBallsAll ${source_files})

message(STATUS "nanogui extra incs: ${NANOGUI_EXTRA_INCS}")

target_include_directories(libgooBallsAll PUBLIC )

# link all dependencies
target_link_libraries(libgooBallsAll
    libgooBalls
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_LOG_LIBRARY}
    ${BOX2D_LIBRARIES}
)

# Create app executable

add_executable(gooBalls main.cpp)
target_link_libraries(gooBalls
    libgooBallsAll
    ${BOX2D_LIBRARIES}
    )

# create unit test executable
add_executable(appTests ${test_files})
target_link_libraries(appTests
    libgooBallsAll
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    )

# create a target for the headers, we don't actually need the result
# but this way, IDE's also show header files in their project overviews
file(GLOB_RECURSE appHeader_files "*.h*")
add_custom_target(appHeaders SOURCES ${appHeader_files})
