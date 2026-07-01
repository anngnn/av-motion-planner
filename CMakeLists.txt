cmake_minimum_required(VERSION 3.22)
project(av_motion_planner)

# without this, cmake . scatters generated files all over source
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "No in-source builds. Run cmake from a build/ directory.")
endif()

find_package(OpenCV REQUIRED)

add_library(kinematic_model_lib src/vehicle/kinematic_model.cpp)

target_include_directories(kinematic_model_lib PUBLIC include/)
target_compile_options(kinematic_model_lib PUBLIC -Wall -Wextra -Wpedantic)
target_compile_features(kinematic_model_lib PUBLIC cxx_std_23)

add_executable(av_motion_planner_exe src/main.cpp)

target_link_libraries(av_motion_planner_exe kinematic_model_lib ${OpenCV_LIBS})