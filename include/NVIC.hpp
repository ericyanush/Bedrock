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
    
    template<Irq interrupt>
    void enableIrq() {
        static_assert(static_cast<int32_t>(interrupt) > 0, "Cannot modify system interrupts with this function");
        constexpr uint32_t intNum = static_cast<uint32_t>(interrupt);
        constexpr uint32_t regIndex = intNum / 32;
        constexpr uint32_t bitIndex = intNum % 32;
        //We eliminate a read of the register, as writing 0's to any bit position
        //   have no effect
        ISER[regIndex] = ENABLE << bitIndex;
    }
    
    template<Irq interrupt>
    void disableIrq() {
        static_assert(static_cast<int32_t>(interrupt) > 0, "Cannot modify system interrupts with this function");
        constexpr uint32_t intNum = static_cast<uint32_t>(interrupt);
        constexpr uint32_t regIndex = intNum / 32;
        constexpr uint32_t bitIndex = intNum % 32;
        
        //We eliminate a read of the register, as writing 0's to any bit position
        //   have no effect
        ICER[regIndex] = ENABLE << bitIndex; // This register is rc_w1, so we are "Enabling the Disable"
    }
    
    template<Irq interrupt>
    void setIrqPriority(uint8_t priority) {
        static_assert(static_cast<int32_t>(interrupt) > 0, "Cannot modify system interrupts with this function");
        constexpr uint32_t intNum = static_cast<uint32_t>(interrupt);
        IPR[intNum] = priority;
    }
    
    template<Irq interrupt>
    uint8_t getIrqPriority() {
        static_assert(static_cast<int32_t>(interrupt) > 0, "Cannot modify system interrupts with this function");
        constexpr uint32_t intNum = static_cast<uint32_t>(interrupt);
        return IPR[intNum];
    }
};

using NVICProvider = NVIC& (*)(void);

#endif /* NVIC_h */
