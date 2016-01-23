//
//  Flash.hpp
//  Bedrock
//
//  Created by Eric Yanush on 2015-12-26.
//  Copyright © 2015 EricYanush. All rights reserved.
//

#ifndef Flash_h
#define Flash_h

#include <stdint.h>
#include "types.hpp"

namespace Bedrock {
    enum class FlashWait : uint8_t {
        zero = 0b000,
        one  = 0b001,
        two  = 0b010
    };
    
    class Flash {
    public:
        
        void setLatency(FlashWait state) {
            //clear the current latency
            AC &= 0b000;
            AC |= static_cast<uint32_t>(state);
        }
        
        void enablePrefetch() {
            AC |= (1 << 4);
        }
        
        void disablePrefetch() {
            AC &= ~(1 << 4);
        }
        
        dev_reg32_t AC;
        dev_reg32_t KEY;
        dev_reg32_t OPTKEY;
        dev_reg32_t SR;
        dev_reg32_t CR;
        dev_reg32_t AR;
        dev_reg32_t OB;
        dev_reg32_t WRP;
    };
    
    using FlashProvider = Flash& (*)(void);
}

#endif /* Flash_h */
