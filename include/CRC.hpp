//
//  CRC.hpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-02-22.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef CRC_h
#define CRC_h

#include <stdint.h>
#include "types.hpp"

namespace Bedrock {
    class CRC {
    public:
        dev_reg32_t DR;
        dev_reg32_t IDR;
        dev_reg32_t CR;
        dev_reg32_t INIT;
        dev_reg32_t POL;
        
        void reset() {
            CR |= 0x1;
        }
        void addWord(uint32_t word) {
            DR = word;
        }
        uint32_t getResult() {
            return DR;
        }
    };
    
    using CRCProvider = CRC& (*)(void);
}

#endif /* CRC_h */
