//
//  FlashTests.cpp
//  Bedrock Tests
//
//  Created by Eric Yanush on 2015-12-26.
//  Copyright © 2015 EricYanush. All rights reserved.
//

#include "gtest/gtest.h"
#include "Flash.hpp"

class FlashTest : public ::testing::Test {
    
protected:
    
    virtual void SetUp() {
        flash = (Flash*) malloc(sizeof(Flash));
    }
    
    virtual void TearDown() {
        free(flash);
    }
    
    Flash* flash;
};

/** Ensure layout of Flash object adheres with Register Map
 * as defined in the STM32F3 Programming Manual
 */
TEST_F(FlashTest, TestFlashRegLayout) {
    ASSERT_EQ(0x00, offsetof(Flash, AC));
    ASSERT_EQ(0x04, offsetof(Flash, KEY));
    ASSERT_EQ(0x08, offsetof(Flash, OPTKEY));
    ASSERT_EQ(0x0C, offsetof(Flash, SR));
    ASSERT_EQ(0x10, offsetof(Flash, CR));
    ASSERT_EQ(0x14, offsetof(Flash, AR));
    ASSERT_EQ(0x18, offsetof(Flash, OB));
    ASSERT_EQ(0x1C, offsetof(Flash, WRP));
}

TEST_F(FlashTest, TestLatencySet) {
    flash->AC = 0;
    flash->setLatency(FlashWait::one);
    ASSERT_EQ(static_cast<uint32_t>(FlashWait::one), flash->AC);
}

TEST_F(FlashTest, TestPrefetchEnable) {
    flash->AC = 0;
    flash->enablePrefetch();
    ASSERT_EQ(1 << 4, flash->AC);
}

TEST_F(FlashTest, TestPrefetchDisable) {
    flash->AC = 0;
    flash->AC |= (1 << 4);
    flash->disablePrefetch();
    ASSERT_EQ(0, flash->AC);
}

