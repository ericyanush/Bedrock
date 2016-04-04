//
//  BasicTimerTests.cpp
//  Bedrock-xcode
//
//  Created by Eric Yanush on 2016-04-04.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "BasicTimer.hpp"
#include "gtest/gtest.h"

using namespace Bedrock;

/**
 Note: upon initializtion of the test fixture, the tim member object is initialized
 to all zeros. The timIsInInitState function checks to see if the tim member object
 is still all zeros.
 */
class BasicTimerTests : public testing::Test {
    
protected:
    const BasicTimer untouched{0};
    BasicTimer tim{0};
    
    bool timIsInInitState() {
        return (memcmp(&untouched, &tim, sizeof(BasicTimer)) == 0);
    }
};

TEST_F(BasicTimerTests, TestRegLayout) {
    ASSERT_EQ(0x00, offsetof(BasicTimer, CR1));
    ASSERT_EQ(0x04, offsetof(BasicTimer, CR2));
    ASSERT_EQ(0x0C, offsetof(BasicTimer, DIER));
    ASSERT_EQ(0x10, offsetof(BasicTimer, SR));
    ASSERT_EQ(0x14, offsetof(BasicTimer, EGR));
    ASSERT_EQ(0x24, offsetof(BasicTimer, CNT));
    ASSERT_EQ(0x28, offsetof(BasicTimer, PSC));
    ASSERT_EQ(0x2C, offsetof(BasicTimer, ARR));
}

TEST_F(BasicTimerTests, TestSetPrescaler) {
    tim.setPrescaler(0xF1EF);
    ASSERT_EQ(0xF1EF, tim.PSC);
    tim.PSC = 0;
    ASSERT_TRUE(timIsInInitState());
}

TEST_F(BasicTimerTests, TestGetPrescaler) {
    tim.setPrescaler(0xF1EF);
    ASSERT_EQ(0xF1EF, tim.getPrescaler());
}

TEST_F(BasicTimerTests, TestSetReloadValue) {
    tim.setReload(0xBEEF);
    ASSERT_EQ(0xBEEF, tim.ARR);
    tim.ARR = 0;
    ASSERT_TRUE(timIsInInitState());
}

TEST_F(BasicTimerTests, TestGetReloadValue) {
    tim.setReload(0xDDEE);
    ASSERT_EQ(0xDDEE, tim.getReload());
}

TEST_F(BasicTimerTests, TestGetCount) {
    tim.CNT = 0x5678;
    ASSERT_EQ(0x5678, tim.getCount());
}

TEST_F(BasicTimerTests, TestEnable) {
    tim.enable();
    ASSERT_EQ(0x1, tim.CR1);
    tim.CR1 &= ~(0x1);
    ASSERT_TRUE(timIsInInitState());
}

TEST_F(BasicTimerTests, TestDisable) {
    tim.enable();
    tim.disable();
    ASSERT_EQ(0x0, tim.CR1);
    ASSERT_TRUE(timIsInInitState());
}

TEST_F(BasicTimerTests, TestAckUpdate) {
    tim.SR = 1;
    tim.ackUpdate();
    ASSERT_EQ(0, tim.SR);
}

TEST_F(BasicTimerTests, TestEnableUpdateInterrupt) {
    tim.enableUpdateInterrupt();
    ASSERT_EQ(0x1, tim.DIER & 0x1);
    tim.DIER &= ~(0x1);
    ASSERT_TRUE(timIsInInitState());
}

TEST_F(BasicTimerTests, TestDisableUpdateInterrupt) {
    tim.enableUpdateInterrupt();
    tim.disableUpdateInterrupt();
    ASSERT_EQ(0, tim.DIER);
    ASSERT_TRUE(timIsInInitState());
}

TEST_F(BasicTimerTests, TestCheckIfUpdateInterruptEnabled) {
    tim.disableUpdateInterrupt();
    ASSERT_FALSE(tim.isUpdateInterruptEnabled());
    tim.enableUpdateInterrupt();
    ASSERT_TRUE(tim.isUpdateInterruptEnabled());
}

TEST_F(BasicTimerTests, TestCheckIfUpdateInterruptPending) {
    tim.SR = 0;
    ASSERT_FALSE(tim.isUpdateInterruptPending());
    tim.SR |= 0x1;
    ASSERT_TRUE(tim.isUpdateInterruptPending());
}