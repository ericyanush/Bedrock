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

namespace Bedrock {
    class RCC {
    public:
        
        void init() {
            constexpr uint32_t RCC_CR_HSION = (1 << 0);
            constexpr uint32_t RCC_CR_CSSON = (1 << 19);
            constexpr uint32_t RCC_CR_HSEON = (1 << 16);
            constexpr uint32_t RCC_CR_PLLON = (1 << 24);
            constexpr uint32_t RCC_CR_HSEBYP = (1 << 18);
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
        
        enum class PLLMultiplier : uint8_t {
            Two       = 0,
            Three     = 1,
            Four      = 2,
            Five      = 3,
            Six       = 4,
            Seven     = 5,
            Eight     = 6,
            Nine      = 7,
            Ten       = 8,
            Eleven    = 9,
            Twelve    = 10,
            Thirteen  = 11,
            Fourteen  = 12,
            Fifteen   = 13,
            Sixteen   = 14,
            Sixteen2  = 15 //This provides an identical multiplier as Sixteen, defined for compatibilty
        };
        
        enum class SysClockPrescale : uint8_t {
            None   = 0,
            DIV2   = 0b1000,
            DIV4   = 0b1001,
            DIV8   = 0b1010,
            DIV16  = 0b1011,
            DIV64  = 0b1100,
            DIV128 = 0b1101,
            DIV256 = 0b1110,
            DIV512 = 0b1111
        };
        
        enum class APBPrescale : uint8_t {
            None  = 0,
            DIV2  = 0b100,
            DIV4  = 0b101,
            DIV8  = 0b110,
            DIV16 = 0b111
        };
        
        enum class SysClockSource : uint8_t {
            HSI = 0,
            HSE = 1,
            PLL = 2
        };
        
        template <FlashProvider flash, PLLMultiplier pllMult, SysClockPrescale sysPrescale,
        APBPrescale apb1Prescale, APBPrescale apb2Prescale>
        void configSystemClock() {
            constexpr uint32_t PLLSRC_HSE_PREDIV = (1 << 16);
            constexpr uint32_t RCC_CR_HSEON = (1 << 16);
            constexpr uint32_t RCC_CR_HSERDY = (1 << 17);
            constexpr uint32_t RCC_CR_PLLON = (1 << 24);
            constexpr uint32_t RCC_CR_PLLRDY = (1 << 25);
            
            //Enable the HSE, and wait for it to be ready
            CR |= RCC_CR_HSEON;
            while(!(CR & RCC_CR_HSERDY));
            
            //Configure the PLL to use HSE with a Prescaler of 1 and a multiplier of 9
            CFG |= (PLLSRC_HSE_PREDIV | (static_cast<uint32_t>(pllMult) << 18));
            
            //Configure the Bus prescalers
            CFG &= ~(0b1111 << 4); // Clear the AHB bus prescaler
            CFG |= (static_cast<uint32_t>(sysPrescale) << 4); // Set the AHB bus prescaler
            
            CFG &= ~(0b111 << 8); // Clear the APB1 bus prescaler
            CFG |= (static_cast<uint32_t>(apb1Prescale) << 8); // Set APB1 prescaler
            
            CFG &= ~(0b111 << 11); // Clear the APB2 bus prescaler
            CFG |= (static_cast<uint32_t>(apb2Prescale) << 11); // Set APB2 prescaler
            
            //Setup the flash latency, used only for CM4 (STM32F3)
            if (flash != nullptr) {
                flash().setLatency(FlashWait::two); //Set the flash latency to 2WS
                flash().enablePrefetch(); // Enable the flash prefetch buffer
            }
            
            //Enable the PLL and wait for it to be ready
            CR |= RCC_CR_PLLON;
            while(!(CR & RCC_CR_PLLRDY));
            
            //Switch sysclock to PLL
            CFG &= ~(0b11); // Clear the sysclock source
            CFG |= static_cast<uint32_t>(SysClockSource::PLL); // Set the sysclock to PLL
            //Wait for switch
            while(!((CFG & (0b11 << 2)) == (static_cast<uint32_t>(SysClockSource::PLL) << 2)));
        }
        
        void enableDMA1()
        {
            AHBENR |= ENABLE << 0;
        }
        
        void enableDMA2()
        {
            AHBENR |= ENABLE << 1;
        }
        
        void enableGPIOA()
        {
            AHBENR |= (ENABLE << 17);
        }
        
        void enableGPIOB()
        {
            AHBENR |= (ENABLE << 18);
        }
        
        void enableGPIOC()
        {
            AHBENR |= (ENABLE << 19);
        }
        
        void enableGPIOD()
        {
            AHBENR |= (ENABLE << 20);
        }
        
        void enableGPIOE()
        {
            AHBENR |= (ENABLE << 21);
        }
        
        void enableGPIOF()
        {
            AHBENR |= (ENABLE << 22);
        }
        
        void enableGPIOG()
        {
            AHBENR |= (ENABLE << 23);
        }
        
        void enableGPIOH()
        {
            AHBENR |= (ENABLE << 16);
        }
        
        void enableCRC()
        {
            AHBENR |= ENABLE << 6;
        }
        
        void enableTimer1()
        {
            APB2ENR |= ENABLE << 11;
        }
        
        void enableSPI1()
        {
            APB2ENR |= ENABLE << 12;
        }
        
        void enableSPI4()
        {
            APB2ENR |= ENABLE << 15;
        }
        
        void enableTimer2()
        {
            APB1ENR |= (ENABLE << 0);
        }
        
        void enableTimer3()
        {
            APB1ENR |= ENABLE << 1;
        }
        
        void enableTimer4()
        {
            APB1ENR |= ENABLE << 2;
        }
        
        void enableSPI2()
        {
            APB1ENR |= ENABLE << 14;
        }
        
        void enableSPI3()
        {
            APB1ENR |= ENABLE << 15;
        }
        
        void enableCAN()
        {
            APB1ENR |= (ENABLE << 25);
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

}

#endif /* RCC_h */
