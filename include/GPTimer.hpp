//
//  GPTimer.h
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-17.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef GPTimer_h
#define GPTimer_h

#include "types.hpp"

namespace Bedrock {
    template <typename width_t>
    class GPTimer {
    public:
        
        void setPrescaler(uint16_t prescaler) {
            PSC = prescaler;
        }

        uint16_t getPrescaler() {
            return (uint16_t)PSC;
        }

        void setReload(width_t reloadVal) {
            ARR = reloadVal;
        }

        width_t getReload() {
            return ARR;
        }
        
        width_t getCount() {
            return (width_t)CNT;
        }
        
        void enable() {
            CR1 |= 0x1;
        }
        
        void disable() {
            CR1 &= ~(0x1);
        }

        bool isEnabled() {
            return (CR1 & ENABLE) == ENABLE;
        }

        void enableUpdateInterrupt() {
            DIER |= (ENABLE << 0);
        }

        void disableUpdateInterrupt() {
            DIER &= ~(ENABLE << 0);
        }

        bool isUpdateInterruptEnabled() {
            return (DIER & ENABLE) == ENABLE;
        }

        bool isUpdateInterruptPending() {
            return (SR & 0x1) == 0x1;
        }

        void ackUpdate() {
            SR &= ~(1 << 0);
        }
        
        void ackTrigger() {
            SR &= ~(1 << 6);
        }
        
        enum class CapCompSelection : uint8_t
        {
            OUTPUT = 0b00,
            IN_DEF = 0b01,
            IN_ALT = 0b10,
            IN_TRC = 0b11
        };
        
        void setCapCompSelection(uint8_t channel, CapCompSelection sel)
        {
            uint8_t regIndex = channel/3;
            uint8_t bitOffset = ((channel - 1) % 2) * 8;
            
            CCMR[regIndex] &= ~(0b11 << bitOffset);
            CCMR[regIndex] |= static_cast<uint8_t>(sel) << bitOffset;
        }
        
        void enableCapCompChannel(uint8_t channel)
        {
            uint8_t bitOffset = (channel - 1) * 4;
            CCER |= (ENABLE << bitOffset);
        }
        
        void disableCapCompChannel(uint8_t channel)
        {
            uint8_t bitOffset = (channel - 1) * 4;
            CCER &= ~(ENABLE << bitOffset);
        }
        
        width_t readInputCaptureChannel(uint8_t channel)
        {
            return CCR[channel];
        }
        
        void enableCaptureEvent(uint8_t channel)
        {
            EGR |= ENABLE << channel;
        }
        
        void disableCaptureEvent(uint8_t channel)
        {
            EGR &= ~(ENABLE << channel);
        }
        
        bool captureCompleteForChannel(uint8_t channel)
        {
            return (SR >> channel) & 0x1;
        }
        
        void ackCaptureForChannel(uint8_t channel)
        {
            SR &= ~(0x1 << channel);
        }
        
        dev_reg32_t CR1;
        dev_reg32_t CR2;
        dev_reg32_t SMCR;
        dev_reg32_t DIER;
        dev_reg32_t SR;
        dev_reg32_t EGR;
        dev_reg32_t CCMR[2];
        dev_reg32_t CCER;
        dev_reg32_t CNT;
        dev_reg32_t PSC;
        dev_reg32_t ARR;
        uint32_t _pad1;
        dev_reg32_t CCR[4];
        uint32_t _pad2;
        dev_reg32_t DCR;
        dev_reg32_t DMAR;
    };
    
    template <typename width_t>
    using GPTimerProvider = GPTimer<width_t>& (*)(void);
}

#endif /* GPTimer_h */
