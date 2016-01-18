//
//  SysTickTests.cpp
//  Bedrock-xcode
//
//  Created by Eric Yanush on 2016-01-10.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "SysTick.hpp"
#include "gtest/gtest.h"


/**
 Note: upon initializtion of the test fixture, the tick member object is initialized
       to all zeros. The tickIsInInitState function checks to see if the tick member object
       is still all zeros.
 */
class SysTickTests : public testing::Test {
    
protected:
    const SysTick untouched{0};
    SysTick tick{0};
    
    bool tickIsInInitState() {
        return (memcmp(&untouched, &tick, sizeof(SysTick)) == 0);
    }
};

TEST_F(SysTickTests, TestEnable) {
    tick.enable();
    
    ASSERT_EQ(true, (tick.CTRL & 0x1) == 0x1);
    tick.CTRL &= ~(0x1); // Manually disable the systick
    ASSERT_EQ(true, tickIsInInitState());
}

TEST_F(SysTickTests, TestDisable) {
    tick.CTRL |= 0x1; //manually enable the systick
    tick.disable();
    
    ASSERT_EQ(true, (tick.CTRL == 0));
    ASSERT_EQ(true, tickIsInInitState());
}

TEST_F(SysTickTests, TestEnableReloadInterrupt) {
    tick.enableReloadInterrupt();
    
    ASSERT_EQ(2, tick.CTRL); // 0x1 << 1
    tick.CTRL &= ~(0x1 << 1); // manually disable the tick interrupt
    ASSERT_EQ(true, tickIsInInitState()); //ensure nothing else was changed
}

TEST_F(SysTickTests, TestDisableReloadInterrupt) {
    tick.CTRL |= 0x1 << 1; //manually enable the interrupt
    tick.disableReloadInterrupt();
    
    ASSERT_EQ(0, tick.CTRL); // ensure it was turned off
    ASSERT_EQ(true, tickIsInInitState());
}

TEST_F(SysTickTests, TestSetReload) {
        tick.setReload(0xFEEDABCD);
    ASSERT_EQ(0x00EDABCD, tick.LOAD);
    tick.LOAD = 0;
    ASSERT_EQ(true, tickIsInInitState());
    
    tick.setReload(0x00881234);
    ASSERT_EQ(0x00881234, tick.LOAD);
    tick.LOAD = 0;
    ASSERT_EQ(true, tickIsInInitState());
}

TEST_F(SysTickTests, TestGetReload) {
    tick.setReload(0xFEEDABCD);
    ASSERT_EQ(0x00EDABCD, tick.getReload());
}

TEST_F(SysTickTests, TestGetCurrentCount) {
    tick.VAL = 0x00ABCDEE;
    ASSERT_EQ(0x00ABCDEE, tick.getCurrentCount());
}

TEST_F(SysTickTests, TestSetClockSourceAHB) {
    tick.setClockSourceAHB();
    ASSERT_TRUE(((tick.CTRL >> 2) & 0x1) == 0x1);
}

TEST_F(SysTickTests, TestSetClockSourceAHB_DIV_8) {
    tick.CTRL |= (1 << 2);
    tick.setClockSourceAHB_DIV_8();
    ASSERT_EQ(0, tick.CTRL);
}