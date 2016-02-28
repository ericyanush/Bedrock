//
//  GPIOTests.cpp
//  Bedrock Tests
//
//  Created by Eric Yanush on 2015-12-19.
//  Copyright © 2015 EricYanush. All rights reserved.
//

#include "Delay.hpp"
#include "gtest/gtest.h"
#include <future>
#include <thread>
#include <atomic>

class DelayTests : public ::testing::Test {
protected:
    static Bedrock::SysTick tick;
    static Bedrock::SysTick& fakeSysTick() {
        return tick;
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

Bedrock::SysTick DelayTests::tick{0};

TEST_F(DelayTests, TestDelayInit) {
    Bedrock::Delay::init<5000000, fakeSysTick>();

    ASSERT_EQ(5000000 / 1000, tick.getReload()); //Reload should be set for 1ms
    ASSERT_EQ(ENABLE << 2, tick.CTRL & (ENABLE << 2)); //Ensure we are using the AHB clock with no prescaler

    Bedrock::InterruptManager& manager = Bedrock::InterruptManager::instance();
    ASSERT_NE(nullptr, manager.getHandlerForInterrupt(InterruptVector::SysTick));

    ASSERT_EQ(ENABLE << 1, tick.CTRL & (ENABLE << 1)); //Ensure reload interrupt enabled
    ASSERT_EQ(ENABLE, tick.CTRL & ENABLE); //Ensure SysTick is enabled
}

TEST_F(DelayTests, TestGetMillis) {
    Bedrock::Delay::init<1000000, fakeSysTick>();

    ASSERT_EQ(0, Bedrock::Delay::getMillis());

    Bedrock::InterruptManager& manager = Bedrock::InterruptManager::instance();
    auto sysTickHandler = manager.getHandlerForInterrupt(InterruptVector::SysTick);

    for (uint32_t i = 0; i < 54321; i++) {
        sysTickHandler();
    }

    ASSERT_EQ(54321, Bedrock::Delay::getMillis());
}

TEST_F(DelayTests, TestDelayMS) {
    using namespace std::literals;
    
    Bedrock::Delay::init<1000000, fakeSysTick>();

    std::atomic_bool done;
    done.store(false);
    auto asyncHW = [this, &done]() {
        Bedrock::InterruptManager& manager = Bedrock::InterruptManager::instance();
        auto sysTickIntHandler = manager.getHandlerForInterrupt(InterruptVector::SysTick);

        while(1) {
            std::this_thread::sleep_for(1ms);
            sysTickIntHandler();
            if (done.load()) {
                return;
            }
        }
    };

    std::thread async(asyncHW);

    uint32_t startMS = Bedrock::Delay::getMillis();
    Bedrock::Delay::ms(15);
    uint32_t endMS = Bedrock::Delay::getMillis();

    done = true;

    async.join();

    ASSERT_GE(endMS - startMS, 15); //Must be at-least 200
    ASSERT_LT(endMS - startMS, 15 * 1.1); //Allow 10% for error in test setup
}

/*
 * This test ensures that the appropriate behavior is observed when the internal counter rolls over
 * This test is disabled normally as it takes quite some time to setup the test environment 
 * to test the rollover situation (~1m on i7 quad core).
 * To run all disabled tests, execute the test suite with: --gtest_also_run_disabled_tests
 */
TEST_F(DelayTests, DISABLED_TestCounterRollover) {
    Bedrock::Delay::init<1000000, fakeSysTick>();
   
    Bedrock::InterruptManager& manager = Bedrock::InterruptManager::instance();
    auto sysTickIntHandler = manager.getHandlerForInterrupt(InterruptVector::SysTick);
    
    //Increment to almost a roll-over condition
    for (uint32_t i = 0; i < UINT32_MAX - 5000; i++) {
        sysTickIntHandler();
    }
    
    std::atomic_bool done;
    done.store(false);
    auto asyncHW = [this, &done, &sysTickIntHandler]() {
        while(1) {
            sysTickIntHandler();
            if (done.load()) {
                return;
            }
        }
    };
    
    std::thread async(asyncHW);
    
    uint32_t startMS = Bedrock::Delay::getMillis();
    Bedrock::Delay::ms(10000);
    uint32_t endMS = Bedrock::Delay::getMillis();
    
    done = true;
    
    async.join();
    
    ASSERT_GE(startMS, UINT32_MAX - 5000); //Ensure we started before the rollover point
    ASSERT_GE(endMS - startMS, 10000); //Ensure we delayed at-least the amount we asked for
    ASSERT_LE(endMS - startMS, 10000 * 1.1); //Ensure we didn't go way over (this is mostly for test harness limitations)
}

