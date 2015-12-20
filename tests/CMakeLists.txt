cmake_minimum_required(VERSION 2.8)

set(TEST_TARGET bedrockTests)

#build the googlemock/test framework
add_subdirectory(googletest)

#List all test sources
set(BEDROCK_TEST_SRCS src/main.cpp
					  src/RCCTests.cpp
					  src/GPIOTests.cpp)

#Include the bedrock headers
include_directories(../include)
#Include the googletests/mock paths
include_directories(googletest/googletest/include
					googletest/googlemock/include)

add_executable(${TEST_TARGET} ${BEDROCK_TEST_SRCS})
target_link_libraries(${TEST_TARGET} gmock gtest)