//
//  DeviceSerial.hpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-02-22.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef DeviceSerial_h
#define DeviceSerial_h

#include "types.hpp"

namespace Bedrock {
    class DeviceSerial {
    public:
        const dev_reg32_t serial[3];
        static constexpr uint32_t SERIAL_WORD_LEN = 3;
    };
    
    using DeviceSerialProvider = DeviceSerial& (*)(void);
}

#endif /* DeviceSerial_h */
