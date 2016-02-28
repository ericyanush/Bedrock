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
