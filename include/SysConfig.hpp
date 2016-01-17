//
//  SysConfig.hpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-17.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef SysConfig_h
#define SysConfig_h

#include "types.hpp"

class SysConfig {
public:
    
    enum class GPIOPort : uint8_t { A, B, C, D, E, F, G };
    
    void setPortForExternalInterrupt(uint8_t exti, GPIOPort port) {
        const uint8_t regNum = exti / 4;
        const uint8_t posShift = (exti % 4) * 4;
        EXTICR[regNum] &= ~(0b1111 << posShift);
        EXTICR[regNum] |= static_cast<uint8_t>(port) << posShift;
    }
    
    dev_reg32_t CFGR1;
    dev_reg32_t RCR;
    dev_reg32_t EXTICR[4];
    dev_reg32_t CFGR2;
    dev_reg32_t CFGR3;
    dev_reg32_t CFGR4;
    
};

#endif /* SysConfig_h */
