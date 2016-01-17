//
//  NVIC.hpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-06.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef NVIC_h
#define NVIC_h

#include "types.hpp"

class NVIC {
public:
    
    dev_reg32_t ISER[3];
    dev_reg32_t ICER[3];
    dev_reg32_t ISPR[3];
    dev_reg32_t ICPR[3];
    dev_reg32_t IABR[3];
    dev_reg8_t IPR[60 * 4]; //byte aligned access is allowed
    
    void enableIrq(InterruptVector interrupt) {
        const uint32_t intNum = static_cast<uint32_t>(interrupt);
        const uint32_t regIndex = intNum / 32;
        const uint32_t bitIndex = intNum % 32;
        //We eliminate a read of the register, as writing 0's to any bit position
        //   have no effect
        ISER[regIndex] = ENABLE << bitIndex;
    }
    
    void disableIrq(InterruptVector interrupt) {
        const uint32_t intNum = static_cast<uint32_t>(interrupt);
        const uint32_t regIndex = intNum / 32;
        const uint32_t bitIndex = intNum % 32;
        
        //We eliminate a read of the register, as writing 0's to any bit position
        //   have no effect
        ICER[regIndex] = ENABLE << bitIndex; // This register is rc_w1, so we are "Enabling the Disable"
    }
    
    void setIrqPriority(InterruptVector interrupt, uint8_t priority) {
        const uint32_t intNum = static_cast<uint32_t>(interrupt);
        IPR[intNum] = priority;
    }
    
    uint8_t getIrqPriority(InterruptVector interrupt) {
        const uint32_t intNum = static_cast<uint32_t>(interrupt);
        return IPR[intNum];
    }
};

using NVICProvider = NVIC& (*)(void);

#endif /* NVIC_h */
