//
//  RCCTests.cpp
//  Bedrock Tests
//
//  Created by Eric Yanush on 2015-12-17.
//  Copyright © 2015 EricYanush. All rights reserved.
//

#include "gtest/gtest.h"
#include "RCC.hpp"

//Includes for async tests (clock config)
#include <thread>
#include <future>
#include <chrono>

class RCCTest : public ::testing::Test {
  
protected:
    
    virtual void SetUp() {
    }
    
    virtual void TearDown() {
    }
    
    RCC rcc{0};
};

/** Ensure layout of RCC object adheres with Register Map
  * as defined in the STM32F3 Programming Manual
  */
TEST_F(RCCTest, TestRCCRegLayout) {
    ASSERT_EQ(offsetof(RCC, CR), 0);
    ASSERT_EQ(offsetof(RCC, CFG), 0x4);
    ASSERT_EQ(offsetof(RCC, CI), 0x8);
    ASSERT_EQ(offsetof(RCC, APB2RST), 0xC);
    ASSERT_EQ(offsetof(RCC, APB1RST), 0x10);
    ASSERT_EQ(offsetof(RCC, AHBENR), 0x14);
    ASSERT_EQ(offsetof(RCC, APB2ENR), 0x18);
    ASSERT_EQ(offsetof(RCC, APB1ENR), 0x1C);
    ASSERT_EQ(offsetof(RCC, BDC), 0x20);
    ASSERT_EQ(offsetof(RCC, CSR), 0x24);
    ASSERT_EQ(offsetof(RCC, AHBRST), 0x28);
    ASSERT_EQ(offsetof(RCC, CFG2), 0x2C);
    ASSERT_EQ(offsetof(RCC, CFG3), 0x30);
}

TEST_F(RCCTest, TestEnableGPIOPortClocks) {
    
    rcc.AHBENR = DISABLE;
    rcc.enableGPIOA();
    ASSERT_EQ((ENABLE<<17), rcc.AHBENR);
    
    rcc.AHBENR = DISABLE;
    rcc.enableGPIOB();
    ASSERT_EQ((ENABLE<<18), rcc.AHBENR);
    
    rcc.AHBENR = DISABLE;
    rcc.enableGPIOC();
    ASSERT_EQ((ENABLE<<19), rcc.AHBENR);
    
    rcc.AHBENR = DISABLE;
    rcc.enableGPIOD();
    ASSERT_EQ((ENABLE<<20), rcc.AHBENR);
    
    rcc.AHBENR = DISABLE;
    rcc.enableGPIOE();
    ASSERT_EQ((ENABLE<<21), rcc.AHBENR);
    
    rcc.AHBENR = DISABLE;
    rcc.enableGPIOF();
    ASSERT_EQ((ENABLE<<22), rcc.AHBENR);
    
    rcc.AHBENR = DISABLE;
    rcc.enableGPIOG();
    ASSERT_EQ((ENABLE<<23), rcc.AHBENR);
    
    rcc.AHBENR = DISABLE;
    rcc.enableGPIOH();
    ASSERT_EQ((ENABLE<<16), rcc.AHBENR);
}

TEST_F(RCCTest, TestEnableCANClock) {
    rcc.APB1ENR = DISABLE;
    rcc.enableCAN();
    ASSERT_EQ((ENABLE<<25), rcc.APB1ENR);
}

TEST_F(RCCTest, TestEnableTimer2Clock) {
    rcc.enableTimer2();
    ASSERT_TRUE((rcc.APB1ENR & 0x1) == 0x1);
}

TEST_F(RCCTest, TestInit) {
    // The init function should:
    //  - Ensure the HSI clock is enabled
    //  - Disable CSS, HSE, and PLL
    //  - Disable the HSE Bypass
    //  - Clear the 3 Configuration Registers
    //  - Disable all clock interrupts
    rcc.CR &= ~(ENABLE); // turn the HSI bit off
    rcc.CR |= (ENABLE << 19) | (ENABLE << 16) | (ENABLE << 24); // Enable CSS/HSE/PLL
    rcc.CR |= (ENABLE << 18); // Enable the HSEBYP
    rcc.CFG = 0x12345678;
    rcc.CFG2 = 0x87654321;
    rcc.CFG3 = 0x45678912;
    rcc.CI = 0x81818181;
    
    rcc.init();
    
    ASSERT_EQ(true, (rcc.CR & (1 << 0)) == (1 << 0));
    ASSERT_EQ(false, (rcc.CR & (1 << 19)) == (1 << 19));
    ASSERT_EQ(false, (rcc.CR & (1 << 16)) == (1 << 16));
    ASSERT_EQ(false, (rcc.CR & (1 << 24)) == (1 << 24));
    ASSERT_EQ(false, (rcc.CR & (1 << 18)) == (1 << 18));
    ASSERT_EQ(0, rcc.CFG);
    ASSERT_EQ(0, rcc.CFG2);
    ASSERT_EQ(0, rcc.CFG3);
    ASSERT_EQ(0, rcc.CI);
}

TEST_F(RCCTest, TestConfigSystemClock) {
    using namespace std::literals;
    
    auto asyncHW = [this]() {
        //Wait for HSE enable request
        while ((rcc.CR & (1 << 16)) != (1 << 16)) {
            std::this_thread::sleep_for(100us);
        }
        rcc.CR |= (1 << 17); // set the HSERDY bit
        
        //Wait for PLL enable request
        while ((rcc.CR & (1 << 24)) != (1 << 24)) {
            std::this_thread::sleep_for(100us);
        }
        rcc.CR |= (1 << 25);
        
        //wait for SysClock switch to PLL
        while ((rcc.CFG & 0b11) != static_cast<uint32_t>(RCC::SysClockSource::PLL)) {
            std::this_thread::sleep_for(100us);
        }
        //Acknowledge the clock switchs
        rcc.CFG |= (static_cast<uint32_t>(RCC::SysClockSource::PLL) << 2);
    };
    std::thread async(asyncHW);

    rcc.configSystemClock<nullptr, RCC::PLLMultiplier::Three, RCC::SysClockPrescale::DIV2, RCC::APBPrescale::DIV8, RCC::APBPrescale::DIV4>();
    async.join();
    
    ASSERT_TRUE((rcc.CR & (1 << 16)) == (1 << 16)); //Ensure HSE has been enabled
    ASSERT_TRUE((rcc.CFG & (1 << 16)) == (1 << 16)); //Ensure PLL source has been set to HSE with no PREDIV
    ASSERT_TRUE((rcc.CFG & (0b1111 << 18)) == (static_cast<uint32_t>(RCC::PLLMultiplier::Three) << 18)); // Ensure the PLL multiplier is setup correctly
    ASSERT_TRUE((rcc.CFG & (0b1111 << 4)) == (static_cast<uint32_t>(RCC::SysClockPrescale::DIV2) << 4)); //Ensure Sysclock prescaler is setup correctly
    ASSERT_TRUE((rcc.CFG & (0b111 << 8)) == (static_cast<uint32_t>(RCC::APBPrescale::DIV8) << 8)); // Ensure APB1 Prescaler is set right
    ASSERT_TRUE((rcc.CFG & (0b111 << 11)) == (static_cast<uint32_t>(RCC::APBPrescale::DIV4) << 11)); //Ensure APB2 Prescaler is set right
}