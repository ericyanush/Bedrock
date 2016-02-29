//
//  DeviceFlashSize.hpp
//  Bedrock-xcode
//
//  Created by Eric Yanush on 2016-02-22.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef DeviceFlashSize_h
#define DeviceFlashSize_h

#include "types.hpp"

namespace Bedrock {
    class DeviceFlashSize {
    public:
        const dev_reg16_t memSize;
        
        uint16_t flashKBs() {
            return memSize;
        }
    };
}

#endif /* DeviceFlashSize_h */
