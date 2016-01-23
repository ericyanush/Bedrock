//
//  NVICTests.cpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-08.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "NVIC.hpp"
#include "gtest/gtest.h"

using namespace Bedrock;

enum class InterruptVector : int32_t {
    Zero,
    One,
    Two,
    Three,
    Four,
    Five,
    ThirtyOne = 31,
    ThirtyTwo = 32,
    SixyThree = 63
};

/**
 Note: upon initializtion of the test fixture, the nvic member object is initialized
 to all zeros. The nvicIsInInitState function checks to see if the nvic member object
 is still all zeros.
 */
class NVICTests : public testing::Test {
    
protected:
    NVIC nvic{0};
    const NVIC untouched{0};
    
    bool nvicIsInInitState() {
        return (memcmp(&nvic, &untouched, sizeof(NVIC)) == 0);
    }
};

TEST_F(NVICTests, TestEnableIrq) {
    nvic.enableIrq(InterruptVector::Three);
    
    //Cast the interrupt to it's underlying number
    constexpr uint32_t intNumber = static_cast<uint32_t>(InterruptVector::Three);
    constexpr uint32_t regNumber = intNumber / 32;
    constexpr uint32_t bitOffset = intNumber % 32;
    
    ASSERT_EQ(1 << bitOffset, nvic.ISER[regNumber]);
    
    nvic.ISER[regNumber] = 0;
    ASSERT_EQ(true, nvicIsInInitState());
}

TEST_F(NVICTests, TestDisableIrq) {
    nvic.disableIrq(InterruptVector::SixyThree);
    
    //Cast the interrupt to it's underlying number
    uint32_t intNumber = static_cast<uint32_t>(InterruptVector::SixyThree);
    uint32_t regNumber = intNumber / 32;
    uint32_t bitOffset = intNumber % 32;
    
    ASSERT_EQ(1 << bitOffset, nvic.ICER[regNumber]);
    nvic.ICER[regNumber] = 0;
    ASSERT_EQ(true, nvicIsInInitState());
}

TEST_F(NVICTests, TestSetPriority) {
    nvic.setIrqPriority(InterruptVector::ThirtyOne, 12);
    constexpr uint32_t intNum = static_cast<uint32_t>(InterruptVector::ThirtyOne);
    ASSERT_EQ(12, nvic.IPR[intNum]);
    nvic.IPR[intNum] = 0;
    ASSERT_EQ(true, nvicIsInInitState());
}

TEST_F(NVICTests, TestGetPriority) {
    constexpr uint32_t intNum = static_cast<uint32_t>(InterruptVector::Five);
    nvic.IPR[intNum] = 21;
    ASSERT_EQ(21, nvic.getIrqPriority(InterruptVector::Five));
}