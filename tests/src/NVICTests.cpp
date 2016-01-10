//
//  NVICTests.cpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-08.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "NVIC.hpp"
#include "gtest/gtest.h"

enum class Irq : int32_t {
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

class NVICTests : public testing::Test {
    
protected:
};

TEST_F(NVICTests, TestEnableIrq) {
    NVIC nvic{0};
    const NVIC untouched{0};
    
    nvic.enableIrq<Irq::Three>();
    
    //Cast the interrupt to it's underlying number
    constexpr uint32_t intNumber = static_cast<uint32_t>(Irq::Three);
    constexpr uint32_t regNumber = intNumber / 32;
    constexpr uint32_t bitOffset = intNumber % 32;
    
    ASSERT_EQ(1 << bitOffset, nvic.ISER[regNumber]);
    
    nvic.ISER[regNumber] = 0;
    ASSERT_EQ(0, memcmp(&untouched, &nvic, sizeof(NVIC)));
}

TEST_F(NVICTests, TestDisableIrq) {
    NVIC nvic{0};
    const NVIC untouched{0};
    
    nvic.disableIrq<Irq::SixyThree>();
    
    //Cast the interrupt to it's underlying number
    uint32_t intNumber = static_cast<uint32_t>(Irq::SixyThree);
    uint32_t regNumber = intNumber / 32;
    uint32_t bitOffset = intNumber % 32;
    
    ASSERT_EQ(1 << bitOffset, nvic.ICER[regNumber]);
    nvic.ICER[regNumber] = 0;
    ASSERT_EQ(0, memcmp(&untouched, &nvic, sizeof(NVIC)));
}

TEST_F(NVICTests, TestSetPriority) {
    NVIC nvic{0};
    const NVIC untouched{0};
    
    nvic.setIrqPriority<Irq::ThirtyOne>(12);
    constexpr uint32_t intNum = static_cast<uint32_t>(Irq::ThirtyOne);
    ASSERT_EQ(12, nvic.IPR[intNum]);
    nvic.IPR[intNum] = 0;
    ASSERT_EQ(0, memcmp(&untouched, &nvic, sizeof(NVIC)));
}

TEST_F(NVICTests, TestGetPriority) {
    NVIC nvic{0};
    
    constexpr uint32_t intNum = static_cast<uint32_t>(Irq::Five);
    nvic.IPR[intNum] = 21;
    ASSERT_EQ(21, nvic.getIrqPriority<Irq::Five>());
}