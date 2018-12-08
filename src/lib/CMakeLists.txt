# add this path to include path of library files
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# list here your source files (*.cpp) of your module
set(source_files
    generic/hello.cpp
    loader/scene_loader.cpp
    observing/abstractObserver.cpp
    observing/abstractDelayedObserver.cpp
    observing/abstractSubject.cpp
    physics/2d/message.cpp
    physics/2d/box2dmessages.cpp
    physics/2d/message.cpp
    physics/2d/engine.cpp
    physics/2d/fluid_solver.cpp
    physics/2d/abstract_sph.cpp
    physics/2d/fluid.cpp
    physics/2d/mesh.cpp
    physics/2d/ssph.cpp
    physics/2d/iisph.cpp
    physics/2d/no_sph.cpp
    physics/2d/visco_elastic.cpp
    physics/2d/kernel_debrun_spiky.cpp
    physics/2d/kernel_poly6.cpp
    physics/2d/kernel_poly_viscosity.cpp
    physics/2d/kernel_divergent_viscosity.cpp
    physics/2d/room.cpp
    physics/2d/recommended_timestep.cpp
    rendering/2d/disk_fluid.cpp
    rendering/2d/fluid.cpp
    rendering/2d/mesh.cpp
    rendering/2d/engine.cpp
    rendering/2d/scene.cpp
    spatial/2d/convex_hull.cpp
    spatial/2d/neighborhood.cpp
    spatial/2d/neighborhood_brute_force.cpp
    spatial/2d/neighborhood_spatial_hashing.cpp
)

message(STATUS "Shader files located at ${CMAKE_CURRENT_SOURCE_DIR}/rendering/2d/shaders")
message(STATUS "Shader copied to ${CMAKE_CURRENT_BINARY_DIR}/shaders")

add_custom_target(copy_shader_files)

add_custom_command(
    TARGET copy_shader_files POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_CURRENT_SOURCE_DIR}/rendering/2d/shaders"
    "${CMAKE_BINARY_DIR}/shaders"
    COMMENT "Copying shader files." VERBATIM 
)

add_library(libgooBalls ${source_files})

add_dependencies(libgooBalls copy_shader_files)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

# add this path to include path of target
target_include_directories(libgooBalls PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${NANOGUI_EXTRA_INCS} ${GOOBALLS_EIGEN3_INCLUDE_DIR})

target_link_libraries(libgooBalls
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_LOG_LIBRARY}
    ${BOX2D_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    nanogui ${NANOGUI_EXTRA_LIBS}
    )
# list here your testing files (*.test.cc) of your module
set(test_files
    test_root.cc
    generic/hello.test.cc
    physics/2d/engine.test.cc
    physics/2d/kernel_test.cc
    physics/2d/kernel_debrun_spiky.test.cc
    physics/2d/kernel_poly6.test.cc
    physics/2d/kernel_poly_viscosity.test.cc
    physics/2d/kernel_divergent_viscosity.test.cc
    observing/abstractSubject.test.cc
    spatial/2d/convex_hull.test.cc
    spatial/2d/neighborhood_spatial_hashing.test.cc
    spatial/2d/neighborhood_brute_force.test.cc
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
