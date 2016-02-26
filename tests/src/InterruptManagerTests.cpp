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
    
    IntMan* im;
    
    virtual void SetUp() {
        im = &InterruptManager::instance();
        im->init(fakeSCB, fakeNVIC);
    }
};

NVIC InterruptManagerTests::nvic{0};
SystemControl InterruptManagerTests::scb{0};

TEST_F(InterruptManagerTests, TestSetPriorityForInterrupt) {
    InterruptManager& manager = InterruptManager::instance();

    manager.setPriorityForInterrupt(InterruptVector::Comparator1_2_3, 3);
    ASSERT_EQ(3, nvic.getIrqPriority(InterruptVector::Comparator1_2_3) >> 4); //The processor only implements the upper 4 bits
}

TEST_F(InterruptManagerTests, TestDisableInterrupt) {
    InterruptManager& manager = InterruptManager::instance();

    manager.disableInterrupt(InterruptVector::FMC);
    const uint32_t vecNum = static_cast<uint32_t>(InterruptVector::FMC);
    ASSERT_TRUE(((nvic.ICER[vecNum/32] >> (vecNum%32)) & 0x1) == 0x1); //Make sure the vector was disabled
}

TEST_F(InterruptManagerTests, TestEnableInterrupt) {
    InterruptManager& manager = InterruptManager::instance();

    manager.disableInterrupt(InterruptVector::I2C2Event);
    manager.enableInterrupt(InterruptVector::I2C2Event);
    const uint32_t vecNum = static_cast<uint32_t>(InterruptVector::I2C2Event);
    ASSERT_TRUE((nvic.ISER[vecNum/32] >> (vecNum%32)) & 0x1); //Make sure the vector was enabled
}

TEST_F(InterruptManagerTests, TestSetHandler) {
    bool handlerCalled = false;
    
    InterruptHandler testHandler = [&handlerCalled]() {
        handlerCalled = true;
    };
    
    InterruptManager::instance().setHandlerForInterrupt(InterruptVector::Timer4, testHandler);
    scb.ICSR = (static_cast<int32_t>(InterruptVector::Timer4) + 16); //Set the currently executing vector
    interruptDispatcher(); // manually call the interrupt handler
    ASSERT_TRUE(handlerCalled);
    
    
    //Test the system with negative numbered interrupts (system interrupts)
    bool systemHandlerCalled = false;
    InterruptHandler systemHandler = [&systemHandlerCalled]() {
        systemHandlerCalled = true;
    };
    
    InterruptManager::instance().setHandlerForInterrupt(InterruptVector::HardFault, systemHandler);
    scb.ICSR = (static_cast<int32_t>(InterruptVector::HardFault) + 16); //Set the currently executing vector
    interruptDispatcher(); // manually call the interrupt handler
    ASSERT_TRUE(systemHandlerCalled);
}

TEST_F(InterruptManagerTests, TestSetNewHandler) {
    bool handler1Called = false;
    InterruptHandler handler1 = [&handler1Called]() {
        handler1Called = true;
    };

    bool handler2Called = false;
    InterruptHandler handler2 = [&handler2Called]() {
        handler2Called = true;
    };

    IntMan &manager = InterruptManager::instance();

    manager.setHandlerForInterrupt(InterruptVector::Timer3, handler1);
    manager.setHandlerForInterrupt(InterruptVector::Timer3, handler2); //replace the "old" handler

    scb.ICSR = (static_cast<int32_t>(InterruptVector::Timer3) + 16); //Set the currently executing vector
    interruptDispatcher();

    ASSERT_TRUE(handler2Called);
    ASSERT_FALSE(handler1Called);
}