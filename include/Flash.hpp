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
#include "types.h"
#include "stm32f303xe.h"

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
        AC |= FLASH_ACR_PRFTBE;
    }
    
    void disablePrefetch() {
        AC &= ~FLASH_ACR_PRFTBE;
    }
    
    dev_reg AC;
    dev_reg KEY;
    dev_reg OPTKEY;
    dev_reg SR;
    dev_reg CR;
    dev_reg AR;
    dev_reg OB;
    dev_reg WRP;
};

typedef Flash* (FlashProvider)(void);

#endif /* Flash_h */
