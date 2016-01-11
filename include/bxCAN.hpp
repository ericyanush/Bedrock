//
//  bxCan.hpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-04.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef bxCan_h
#define bxCan_h

#include <stdint.h>
#include "types.hpp"
#include "CANMessage.hpp"
#include "RCC.hpp"

namespace CAN {
    
    class TxMailbox {
    public:
        dev_reg32_t TIR;
        dev_reg32_t TDTR;
        dev_reg32_t TDLR;
        dev_reg32_t TDHR;
    };
    
    class RxFIFO {
    public:
        dev_reg32_t RIR;
        dev_reg32_t RDTR;
        dev_reg32_t RDLR;
        dev_reg32_t RDHR;
    };
    
    class Filters {
    public:
        struct FilterBank {
            dev_reg32_t FR1;
            dev_reg32_t FR2;
        };
        
        dev_reg32_t FMR;
        dev_reg32_t FM1R;
        //Padding
        uint32_t _pad_1;
        dev_reg32_t FS1R;
        //Padding
        uint32_t _pad_2;
        dev_reg32_t FF1AR;
        //Padding
        uint32_t _pad_3;
        dev_reg32_t FA1R;
        //Padding
        uint32_t _pad_4[8];
        FilterBank filterBank[14];
    };
    
    
    class bxCAN {
    public:
        
        bxCAN() { };
        
        dev_reg32_t MCR;
        dev_reg32_t MSR;
        dev_reg32_t TSR;
        dev_reg32_t RF0R;
        dev_reg32_t RF1R;
        dev_reg32_t IER;
        dev_reg32_t ESR;
        dev_reg32_t BTR;
        
        //anonymous padding between base registers and mailbox registers
        uint32_t _pad_1[88];
        
        TxMailbox txMailbox[3];
        
        RxFIFO rxFIFO[2];
        
        //anonymous padding between RX Queues and Filter Registers
        uint32_t _pad_2[12];
        
        Filters filters;
    };
}

#endif /* bxCan_h */
