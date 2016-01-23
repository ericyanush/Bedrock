//
//  SystemControl.hpp
//  Bedrock-xcode
//
//  Created by Eric Yanush on 2016-01-07.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef SystemControl_h
#define SystemControl_h

#include <stdint.h>
#include "types.hpp"

namespace Bedrock {
    /**
     Note: On devices that do not implement all 8 bits for the priority fields, the least significant bits are ignored,
     therefore, the value of PriorityGroupCount used to configure the priority groups may not reflect the actual
     number of available priority groups or sub-priorities.
     */
    enum class PriorityGroupCount: uint32_t {
        Groups_128  = 0b000,
        Groups_64   = 0b001,
        Groups_32   = 0b010,
        Groups_16   = 0b011,
        Groups_8    = 0b100,
        Groups_4    = 0b101,
        Groups_2    = 0b110,
        Groups_None = 0b111
    };
    
    class SystemControl {
    public:
        dev_reg32_t CPUID;
        dev_reg32_t ICSR;
        dev_reg32_t VTOR;
        dev_reg32_t AIRCR;
        dev_reg32_t SCR;
        dev_reg32_t CCR;
        dev_reg32_t SHPR1;
        dev_reg32_t SHPR2;
        dev_reg32_t SHPR3;
        dev_reg32_t SHCRS;
        dev_reg32_t CFSR;
        dev_reg32_t HFSR;
        uint32_t _pad;
        dev_reg32_t MMAR;
        dev_reg32_t BFAR;
        dev_reg32_t AFSR;
        
        void setVectorBaseAddress(uint32_t vecAddress) {
            VTOR = vecAddress;
        }
        uint32_t getVectorBaseAddress() {
            return VTOR;
        }
        
        void setPriorityGroups(PriorityGroupCount groupCount) {
            //clear the current value and key
            AIRCR &= ~(VECT_KEY | 0b111 << 8);
            //Update the value
            AIRCR |= (VECT_KEY | (static_cast<uint32_t>(groupCount) << 8));
        }
        
        PriorityGroupCount getPriorityGroups() {
            return PriorityGroupCount((AIRCR >> 8) & 0b111);
        }
        
        InterruptVector getActiveVector() {
            return static_cast<InterruptVector>(((int16_t)(ICSR & 0x1FF)) - 16);
        }
        
    private:
        static constexpr uint32_t VECT_KEY = 0x05FA0000; //The key that must be written to modify some register values
    };
    
    using SystemControlProvider = SystemControl& (*)(void);

}

#endif /* SystemControl_h */
