# Lines that begin with a # are comments
# set the minimum required version of cmake, usually the first line
cmake_minimum_required(VERSION 3.22)

# project_name sets the name of the project and causes cmake to
# find the c and c++ compilers
project(av_motion_planner)

# without this, cmake . scatters generated files all over source
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "No in-source builds. Run cmake from a build/ directory.")
endif()

# Find dependencies.
# Many libraries ship with files that allow CMake to find them
# Then general behavior is to call "find_package" but the options
# provided are package specific.  Usually there is then a CMAKE variable
# That is defined to reference the library
find_package(OpenCV REQUIRED)

# Create a library. Can specify if it is shared or static but usually
# don't need or want to.
# name is the name of the library without the extension or lib prefix
# name creates a cmake "target" (in this case the target is called "kinematic_model_lib"
add_library(kinematic_model_lib src/vehicle/kinematic_model.cpp)
# Use target_include_directories so that #include"mylibrary/header.hpp" works
# The use of the <BUILD_INTERFACE> and <INSTALL_INTERFACE> is because when
# using the library from the build directory or after installation the header
# files are actually in different locations.
# During the build, the headers are read from the source code directory
# When used from the installed location, headers are in the
# system include/ directory
target_include_directories(kinematic_model_lib PUBLIC include/)
# specify additional compilation flags for the library
# Public causes the flags to propagate to anything
# that links against this library
target_compile_options(kinematic_model_lib PUBLIC -Wall -Wextra -Wpedantic)
# Enable c++23 support.
# PUBLIC causes the features to propagate to anything
# that links against this library
# whereas PRIVATE would make it apply only to the library itself
target_compile_features(kinematic_model_lib PUBLIC cxx_std_23)

add_library(planning_lib src/planning/road_graph.cpp src/planning/astar.cpp src/planning/frenet.cpp src/planning/quintic_polynomial.cpp)
target_include_directories(planning_lib PUBLIC include/ /usr/include/eigen3)
target_compile_options(planning_lib PUBLIC -Wall -Wextra -Wpedantic)
target_compile_features(planning_lib PUBLIC cxx_std_23)

# Create an executable from the following source code files
# The Name of the executable creates a cmake "target"
add_executable(av_motion_planner_exe src/main.cpp)

# Use target_link_libraries to add dependencies to a "target"
# (e.g., a library or executable)
# This will automatically add all required library files
# that need to be linked
# and paths to th locations of header files.
# Because libname has PUBLIC cxx_std_23 as a compile feature
# Name will also be compiled with c++23 as the standard.
target_link_libraries(av_motion_planner_exe kinematic_model_lib planning_lib ${OpenCV_LIBS})

include(CTest)

if(BUILD_TESTING)
  find_package(Catch2 3 REQUIRED)
  add_executable(test_planning_exe tests/test_road_graph.cpp tests/test_astar.cpp tests/test_frenet.cpp tests/test_quintic.cpp)
  target_link_libraries(test_planning_exe Catch2::Catch2WithMain planning_lib)
  add_test(NAME test_planning COMMAND test_planning_exe)
endif()