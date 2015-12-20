cmake_minimum_required(VERSION 2.8)
project(Bedrock)

#ensure we are using appropriate standards
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED on)

#build the unit tests target
add_subdirectory(tests)
