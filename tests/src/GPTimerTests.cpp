//
//  GPTimerTests.cpp
//  Bedrock-xcode
//
//  Created by Eric Yanush on 2016-01-17.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "gtest/gtest.h"
#include "GPTimer.hpp"

using namespace Bedrock;

/** 
 We will test only the 32bit wide timer, as the only difference is
 function signatures for some functions (32bit vs 16bit timer returing/taking 32/16 bit vals)
 */
using GPTimer_t = GPTimer<uint32_t>;

/**
 Note: upon initializtion of the test fixture, the tim member object is initialized
 to all zeros. The timIsInInitState function checks to see if the tim member object
 is still all zeros.
 */
class GPTimerTests : public testing::Test {
    
protected:
    const GPTimer_t untouched{0};
    GPTimer_t tim{0};
    
    bool timIsInInitState() {
        return (memcmp(&untouched, &tim, sizeof(GPTimer_t)) == 0);
    }
};

TEST_F(GPTimerTests, TestRegLayout) {
    ASSERT_EQ(0x00, offsetof(GPTimer_t, CR1));
    ASSERT_EQ(0x04, offsetof(GPTimer_t, CR2));
    ASSERT_EQ(0x08, offsetof(GPTimer_t, SMCR));
    ASSERT_EQ(0x0C, offsetof(GPTimer_t, DIER));
    ASSERT_EQ(0x10, offsetof(GPTimer_t, SR));
    ASSERT_EQ(0x14, offsetof(GPTimer_t, EGR));
    ASSERT_EQ(0x18, offsetof(GPTimer_t, CCMR[0]));
    ASSERT_EQ(0x1C, offsetof(GPTimer_t, CCMR[1]));
    ASSERT_EQ(0x20, offsetof(GPTimer_t, CCER));
    ASSERT_EQ(0x24, offsetof(GPTimer_t, CNT));
    ASSERT_EQ(0x28, offsetof(GPTimer_t, PSC));
    ASSERT_EQ(0x2C, offsetof(GPTimer_t, ARR));
    ASSERT_EQ(0x34, offsetof(GPTimer_t, CCR[0]));
    ASSERT_EQ(0x38, offsetof(GPTimer_t, CCR[1]));
    ASSERT_EQ(0x3C, offsetof(GPTimer_t, CCR[2]));
    ASSERT_EQ(0x40, offsetof(GPTimer_t, CCR[3]));
    ASSERT_EQ(0x48, offsetof(GPTimer_t, DCR));
    ASSERT_EQ(0x4C, offsetof(GPTimer_t, DMAR));
}

TEST_F(GPTimerTests, TestSetPrescaler) {
    tim.setPrescaler(0xF1EF);
    ASSERT_EQ(0xF1EF, tim.PSC);
    tim.PSC = 0;
    ASSERT_TRUE(timIsInInitState());
}

TEST_F(GPTimerTests, TestSetReloadValue) {
    tim.setReload(0xFEEDBEEF);
    ASSERT_EQ(0xFEEDBEEF, tim.ARR);
    tim.ARR = 0;
    ASSERT_TRUE(timIsInInitState());
}

TEST_F(GPTimerTests, TestGetCount) {
    tim.CNT = 0x12345678;
    ASSERT_EQ(0x12345678, tim.getCount());
}

TEST_F(GPTimerTests, TestEnable) {
    tim.enable();
    ASSERT_EQ(0x1, tim.CR1);
    tim.CR1 &= ~(0x1);
    ASSERT_TRUE(timIsInInitState());
}

TEST_F(GPTimerTests, TestDisable) {
    tim.enable();
    tim.disable();
    ASSERT_EQ(0x0, tim.CR1);
    ASSERT_TRUE(timIsInInitState());
}

TEST_F(GPTimerTests, TestAckUpdate) {
    tim.SR = 1;
    tim.ackUpdate();
    ASSERT_EQ(0, tim.SR);
}

TEST_F(GPTimerTests, TestAckTrigger) {
    tim.SR = (1 << 6);
    tim.ackTrigger();
    ASSERT_EQ(0, tim.SR);
}

TEST_F(GPTimerTests, TestEnableUpdateInterrupt) {
    tim.enableUpdateInterrupt();
    ASSERT_EQ(0x1, tim.DIER & 0x1);
    tim.DIER &= ~(0x1);
    ASSERT_TRUE(timIsInInitState());
}

TEST_F(GPTimerTests, TestDisableUpdateInterrupt) {
    tim.enableUpdateInterrupt();
    tim.disableUpdateInterrupt();
    ASSERT_EQ(0, tim.DIER);
    ASSERT_TRUE(timIsInInitState());
}