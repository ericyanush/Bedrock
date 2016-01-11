//
//  CANMessage.hpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-05.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef CANMessage_h
#define CANMessage_h

#include <stdint.h>

class CANMessage {
public:
    
    enum class Type : uint8_t {
        Data,
        Remote
    };
    enum class Format : uint8_t {
        Standard,
        Extended
    };
    
    uint32_t id;
    uint32_t timeStamp;
    uint8_t data[8];
    uint8_t dataLen;
    Type type;
    Format format;
    
};

#endif /* CANMessage_h */
