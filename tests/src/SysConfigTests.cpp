//
//  SysConfigTests.cpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-17.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "gtest/gtest.h"
#include "SysConfig.hpp"

using namespace Bedrock;

/**
 Note: upon initializtion of the test fixture, the sysconf member object is initialized
 to all zeros. The sysconfIsInInitState function checks to see if the sysconf member object
 is still all zeros.
 */
class SysConfigTests : public testing::Test {
    
protected:
    const SysConfig untouched{0};
    SysConfig sysconf{0};
    
    bool sysconfIsInInitState() {
        return (memcmp(&untouched, &sysconf, sizeof(SysConfig)) == 0);
    }
};

TEST_F(SysConfigTests, TestSetEXTIPort) {
    sysconf.setPortForExternalInterrupt(6, SysConfig::GPIOPort::D);
    ASSERT_EQ(SysConfig::GPIOPort::D, static_cast<SysConfig::GPIOPort>((sysconf.EXTICR[1] >> 8) & 0xF));
    sysconf.EXTICR[1] = 0;
    ASSERT_TRUE(sysconfIsInInitState());
    
    sysconf.setPortForExternalInterrupt(13, SysConfig::GPIOPort::B);
    ASSERT_EQ(SysConfig::GPIOPort::B, static_cast<SysConfig::GPIOPort>((sysconf.EXTICR[3] >> 4) & 0xF));
    sysconf.EXTICR[3] = 0;
    ASSERT_TRUE(sysconfIsInInitState());
    
    sysconf.setPortForExternalInterrupt(0, SysConfig::GPIOPort::F);
    ASSERT_EQ(SysConfig::GPIOPort::F, static_cast<SysConfig::GPIOPort>((sysconf.EXTICR[0] >> 0) & 0xF));
    sysconf.EXTICR[0] = 0;
    ASSERT_TRUE(sysconfIsInInitState());
}