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

namespace Bedrock {
    class CANMessage {
    public:
        
        enum class Type : uint8_t {
            Data = 0,
            Remote = 1
        };
        enum class Format : uint8_t {
            Standard = 0,
            Extended = 1
        };
        
        uint32_t id;
        uint8_t data[8];
        uint8_t dataLen;
        Type type;
        Format format;

        CANMessage() {};
        CANMessage(uint32_t id, Type type, Format format, uint8_t dataLength) :
                   id(id), dataLen(dataLength), type(type), format(format) { };
    };
}

#endif /* CANMessage_h */
