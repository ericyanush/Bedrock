//
//  RCCTests.cpp
//  Bedrock Tests
//
//  Created by Eric Yanush on 2015-12-17.
//  Copyright © 2015 EricYanush. All rights reserved.
//

#include "gtest/gtest.h"
#include "RCC.hpp"

class RCCTest : public ::testing::Test {
  
protected:
    
    virtual void SetUp() {
        rcc = (RCC*) malloc(sizeof(RCC));
    }
    
    virtual void TearDown() {
        free(rcc);
    }
    
    RCC* rcc;
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
    
    rcc->AHBENR = DISABLE;
    rcc->enableGPIOA();
    ASSERT_EQ((ENABLE<<17), rcc->AHBENR);
    
    rcc->AHBENR = DISABLE;
    rcc->enableGPIOB();
    ASSERT_EQ((ENABLE<<18), rcc->AHBENR);
    
    rcc->AHBENR = DISABLE;
    rcc->enableGPIOC();
    ASSERT_EQ((ENABLE<<19), rcc->AHBENR);
    
    rcc->AHBENR = DISABLE;
    rcc->enableGPIOD();
    ASSERT_EQ((ENABLE<<20), rcc->AHBENR);
    
    rcc->AHBENR = DISABLE;
    rcc->enableGPIOE();
    ASSERT_EQ((ENABLE<<21), rcc->AHBENR);
    
    rcc->AHBENR = DISABLE;
    rcc->enableGPIOF();
    ASSERT_EQ((ENABLE<<22), rcc->AHBENR);
    
    rcc->AHBENR = DISABLE;
    rcc->enableGPIOG();
    ASSERT_EQ((ENABLE<<23), rcc->AHBENR);
    
    rcc->AHBENR = DISABLE;
    rcc->enableGPIOH();
    ASSERT_EQ((ENABLE<<16), rcc->AHBENR);
}

TEST_F(RCCTest, TestInit) {
    // The init function should:
    //  - Ensure the HSI clock is enabled
    //  - Disable CSS, HSE, and PLL
    //  - Disable the HSE Bypass
    //  - Clear the 3 Configuration Registers
    //  - Disable all clock interrupts
    rcc->CR &= ~(ENABLE); // turn the HSI bit off
    rcc->CR |= (ENABLE << 19) | (ENABLE << 16) | (ENABLE << 24); // Enable CSS/HSE/PLL
    rcc->CR |= (ENABLE << 18); // Enable the HSEBYP
    rcc->CFG = 0x12345678;
    rcc->CFG2 = 0x87654321;
    rcc->CFG3 = 0x45678912;
    rcc->CI = 0x81818181;
    
    rcc->init();
    
    ASSERT_EQ(true, (rcc->CR & RCC_CR_HSION) == RCC_CR_HSION);
    ASSERT_EQ(false, (rcc->CR & RCC_CR_CSSON) == RCC_CR_CSSON);
    ASSERT_EQ(false, (rcc->CR & RCC_CR_HSEON) == RCC_CR_HSEON);
    ASSERT_EQ(false, (rcc->CR & RCC_CR_PLLON) == RCC_CR_PLLON);
    ASSERT_EQ(false, (rcc->CR & RCC_CR_HSEBYP) == RCC_CR_HSEBYP);
    ASSERT_EQ(0, rcc->CFG);
    ASSERT_EQ(0, rcc->CFG2);
    ASSERT_EQ(0, rcc->CFG3);
    ASSERT_EQ(0, rcc->CI);
}