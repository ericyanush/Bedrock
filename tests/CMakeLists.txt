cmake_minimum_required(VERSION 2.8)

set(TEST_TARGET bedrockTests)

#build the googlemock/test framework
add_subdirectory(googletest)

#List all test sources
set(BEDROCK_TEST_SRCS src/main.cpp
                      src/RCCTests.cpp
                      src/GPIOTests.cpp
                      src/FlashTests.cpp
                      src/bxCANTestHelpers.cpp
                      src/bxCANTests.cpp
                      src/SystemControlTests.cpp
                      src/NVICTests.cpp
                      src/SysTickTests.cpp
                      src/InterruptManagerTests.cpp
                      src/SysConfigTests.cpp
                      src/GPTimerTests.cpp
                      src/DelayTests.cpp
                      src/SPITests.cpp)

set(BEDROCK_SRCS ../src/InterruptManager.cpp
                 ../src/Delay.cpp)

#Include test class headers
include_directories(include)
#Include the bedrock headers
include_directories(../include)
#Include the googletests/mock paths
include_directories(googletest/googletest/include
                    googletest/googlemock/include)

add_executable(${TEST_TARGET} ${BEDROCK_TEST_SRCS} ${BEDROCK_SRCS})
target_link_libraries(${TEST_TARGET} gmock gtest)
