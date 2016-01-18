//
//  RCC.h
//  Bedrock RCC object for STM32F3
//
//  Created by Eric Yanush on 2015-12-17.
//  Copyright © 2015 EricYanush. All rights reserved.
//

#ifndef RCC_h
#define RCC_h

#include "types.hpp"
#include "Flash.hpp"
#include "stm32f303xe.h"

class RCC {
public:
    
    void init() {
        //ensure HSI is on
        CR |= RCC_CR_HSION;
        //Turn off CSS, HSE, PLL
        CR &= ~(RCC_CR_CSSON | RCC_CR_HSEON | RCC_CR_PLLON);
        //Turn off the HSEBYP; can only be changed once the HSE clock is disabled
        CR &= ~RCC_CR_HSEBYP;
        //Reset the CFG register
        CFG = 0;
        //Reset the CFG2 register
        CFG2 = 0;
        //Reset the CFG3 register
        CFG3 = 0;
        //Disable all the clock interrupts
        CI = 0;
    }
    
    template <FlashProvider flash>
    void configSystemClock() {
        //Enable the HSE, and wait for it to be ready
        CR |= RCC_CR_HSEON;
        while(!(CR & RCC_CR_HSERDY));
        
        //Configure the PLL to use HSE with a Prescaler of 1 and a multiplier of 9
        CFG |= (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV1 | RCC_CFGR_PLLMUL9);
        
        //Configure the Bus prescalers
        CFG &= ~(RCC_CFGR_HPRE); // Clear the AHB bus prescaler
        CFG |= RCC_CFGR_HPRE_DIV1; // Set the AHB bus prescaler to 1 (72MHz)
        
        CFG &= ~(RCC_CFGR_PPRE1); // Clear the APB1 bus prescaler
        CFG |= RCC_CFGR_PPRE1_DIV2; // Set APB1 prescaler to 2 (36MHz)
        
        CFG &= ~(RCC_CFGR_PPRE2); // Clear the APB2 bus prescaler
        CFG |= RCC_CFGR_PPRE2_DIV1; // Set APB2 prescaler to 1 (72MHz)
        
        //Setup the flash latency
        flash().setLatency(FlashWait::two); //Set the flash latency to 2WS
        flash().enablePrefetch(); // Enable the flash prefetch buffer
        
        //Enable the PLL and wait for it to be ready
        CR |= RCC_CR_PLLON;
        while(!(CR & RCC_CR_PLLRDY));
        
        //Switch sysclock to PLL
        CFG &= ~RCC_CFGR_SW; // Clear the sysclock source
        CFG |= RCC_CFGR_SW_PLL; // Set the sysclock to PLL
        //Wait for switch
        while(!((CFG & RCC_CFGR_SWS_PLL) == RCC_CFGR_SWS_PLL));
    }
    
    void enableGPIOA() {
        AHBENR |= (ENABLE << 17);
    }
    
    void enableGPIOB() {
        AHBENR |= (ENABLE << 18);
    }
    
    void enableGPIOC() {
        AHBENR |= (ENABLE << 19);
    }
    
    void enableGPIOD() {
        AHBENR |= (ENABLE << 20);
    }
    
    void enableGPIOE() {
        AHBENR |= (ENABLE << 21);
    }
    
    void enableGPIOF() {
        AHBENR |= (ENABLE << 22);
    }
    
    void enableGPIOG() {
        AHBENR |= (ENABLE << 23);
    }
    
    void enableGPIOH() {
        AHBENR |= (ENABLE << 16);
    }
    
    void enableCAN() {
        APB1ENR |= (ENABLE << 25);
    }
    
    void enableTimer2() {
        APB1ENR |= (ENABLE << 0);
    }
    
    dev_reg32_t CR;
    dev_reg32_t CFG;
    dev_reg32_t CI;
    dev_reg32_t APB2RST;
    dev_reg32_t APB1RST;
    dev_reg32_t AHBENR;
    dev_reg32_t APB2ENR;
    dev_reg32_t APB1ENR;
    dev_reg32_t BDC;
    dev_reg32_t CSR;
    dev_reg32_t AHBRST;
    dev_reg32_t CFG2;
    dev_reg32_t CFG3;
};

using RCCProvider = RCC& (*)(void);

#endif /* RCC_h */
