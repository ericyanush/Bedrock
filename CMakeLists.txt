cmake_minimum_required(VERSION 2.8)
project(Bedrock)

#ensure we are using appropriate standards
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_C_FLAGS "-std=c11")

#build the unit tests target
add_subdirectory(tests)
