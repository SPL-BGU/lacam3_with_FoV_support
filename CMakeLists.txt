cmake_minimum_required(VERSION 3.16)
project(lacam-project CXX)

SET(CMAKE_CXX_FLAGS "-g -rdynamic")
# set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
# SET(CMAKE_BUILD_TYPE Debug)

add_subdirectory(./lacam3)
add_subdirectory(./third_party/argparse)

add_executable(main main.cpp)
target_compile_features(main PUBLIC cxx_std_17)
target_link_libraries(main lacam3 argparse -static)

# test
enable_testing()
file(GLOB TEST_FILES "./tests/test_*.cpp")
foreach(file ${TEST_FILES})
  string(REGEX MATCH "test\_[^\.]+" name "${file}")
  add_executable(${name} ${file})
  target_link_libraries(${name} lacam3)
  add_test(${name} ${name})
endforeach()
