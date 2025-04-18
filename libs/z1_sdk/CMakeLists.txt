cmake_minimum_required(VERSION 3.0)
project(z1_sdk)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++14 -pthread")

include_directories(
  include
)

link_directories(lib)

set(EXAMPLES_FILES
    examples/highcmd_basic.cpp
    examples/highcmd_development.cpp
    examples/lowcmd_development.cpp
    examples/lowcmd_multirobots.cpp
)

foreach(EXAMPLE_FILE IN LISTS EXAMPLES_FILES)
  get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
  add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
  target_link_libraries(${EXAMPLE_NAME} Z1_SDK_${CMAKE_SYSTEM_PROCESSOR})
endforeach()


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
find_package(pybind11 QUIET)
if(${pybind11_FOUND})
  message(STATUS "pybind11 found, building Python interface")
  pybind11_add_module(z1_arm_interface examples_py/arm_python_interface.cpp)
  target_link_libraries(z1_arm_interface PRIVATE Z1_SDK_${CMAKE_SYSTEM_PROCESSOR})
else()
  message(WARNING "pybind11 not found, skipping Python interface build")
endif()