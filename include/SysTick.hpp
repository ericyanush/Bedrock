//
//  SysTick.hpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-10.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef SysTick_h
#define SysTick_h

#include "types.hpp"

class SysTick {
public:
    
    void enable() {
        CTRL |= (ENABLE << 0);
    }
    void disable() {
        CTRL &= ~(ENABLE << 0);
    }
    
    void enableReloadInterrupt() {
        CTRL |= (ENABLE << 1);
    }
    void disableReloadInterrupt() {
        CTRL &= ~(ENABLE << 1);
    }
    
    void setReload(uint32_t reloadVal) {
        //The reload value is only 24 bits wide.
        LOAD = (reloadVal & 0x00FFFFFF);
    }
    
    uint32_t getReload() {
        return LOAD;
    }
    
    uint32_t getCurrentCount() {
        return VAL;
    }
    
    dev_reg32_t CTRL;
    dev_reg32_t LOAD;
    dev_reg32_t VAL;
    dev_reg32_t CALIB;
};

#endif /* SysTick_h */
