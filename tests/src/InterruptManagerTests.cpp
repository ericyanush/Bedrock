//
//  InterruptManagerTests.cpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-16.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "gtest/gtest.h"
#include "InterruptManager.hpp"

using namespace Bedrock;

class InterruptManagerTests : public testing::Test {
    
protected:
    static NVIC nvic;
    static NVIC& fakeNVIC() {
        return nvic;
    }
    
    static SystemControl scb;
    static SystemControl& fakeSCB() {
        return scb;
    }
    
    using IntMan = InterruptManager;
    
    IntMan im;
    
    // Variable and method for testing setHandler
    static bool handlerCalled;
    static void testHandler() {
        handlerCalled = true;
    }
    
    virtual void SetUp() {
        im.init(fakeSCB, fakeNVIC);
    }
};

NVIC InterruptManagerTests::nvic{0};
SystemControl InterruptManagerTests::scb{0};
bool InterruptManagerTests::handlerCalled = false;

TEST_F(InterruptManagerTests, TestSetPriorityForInterrupt) {
    IntMan::setPriorityForInterrupt(InterruptVector::Comparator1_2_3, 3);
    ASSERT_EQ(3, nvic.getIrqPriority(InterruptVector::Comparator1_2_3) >> 4); //The processor only implements the upper 4 bits
}

TEST_F(InterruptManagerTests, TestDisableInterrupt) {
    IntMan::disableInterrupt(InterruptVector::FMC);
    const uint32_t vecNum = static_cast<uint32_t>(InterruptVector::FMC);
    ASSERT_TRUE(((nvic.ICER[vecNum/32] >> (vecNum%32)) & 0x1) == 0x1); //Make sure the vector was disabled
}

TEST_F(InterruptManagerTests, TestEnableInterrupt) {
    IntMan::disableInterrupt(InterruptVector::I2C2Event);
    IntMan::enableInterrupt(InterruptVector::I2C2Event);
    const uint32_t vecNum = static_cast<uint32_t>(InterruptVector::I2C2Event);
    ASSERT_TRUE((nvic.ISER[vecNum/32] >> (vecNum%32)) & 0x1); //Make sure the vector was enabled
}

TEST_F(InterruptManagerTests, TestSetHandler) {
    handlerCalled = false;
    
    IntMan::setHandlerForInterrupt(InterruptVector::Timer4, testHandler);
    scb.ICSR = (static_cast<int32_t>(InterruptVector::Timer4) + 16); //Set the currently executing vector
    interruptDispatcher(); // manually call the interrupt handler
    ASSERT_TRUE(handlerCalled);
}