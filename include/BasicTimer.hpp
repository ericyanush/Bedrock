//
//  BasicTimer.hpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-04-04.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef BasicTimer_h
#define BasicTimer_h

#include "types.hpp"

namespace Bedrock {
    
    class BasicTimer {
    public:
        
        void setPrescaler(uint16_t prescaler) {
            PSC = prescaler;
        }
        
        uint16_t getPrescaler() {
            return (uint16_t)PSC;
        }
        
        void setReload(uint16_t reloadVal) {
            ARR = reloadVal;
        }
        
        uint16_t getReload() {
            return (uint16_t)ARR;
        }
        
        uint16_t getCount() {
            return (uint16_t)CNT;
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
        
        dev_reg32_t CR1;
        dev_reg32_t CR2;
        uint32_t __pad1;
        dev_reg32_t DIER;
        dev_reg32_t SR;
        dev_reg32_t EGR;
        uint32_t __pad2[3];
        dev_reg32_t CNT;
        dev_reg32_t PSC;
        dev_reg32_t ARR;
    };
    
    using BasicTimerProvider = BasicTimer& (*)(void);
}

#endif /* BasicTimer_h */
