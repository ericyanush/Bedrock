//
//  InterruptManager.hpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-15.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef InterruptManager_hpp
#define InterruptManager_hpp

#include "NVIC.hpp"
#include "SystemControl.hpp"
#include "Interrupts.hpp"
#include <functional>

using InterruptHandler = std::function<void(void)>;

extern "C" {
    void interruptHandler();
}

class InterruptManager {
public:
    static void init(SystemControlProvider sysCtl, NVICProvider nvic) {
        SCB = sysCtl;
        NVIC = nvic;
    }
    static void setHandlerForInterrupt(InterruptVector vector, InterruptHandler handler) {
        handlers[static_cast<uint32_t>(vector)] = handler;
    }
    static void enableInterrupt(InterruptVector vector) {
        NVIC().enableIrq(vector);
    }
    static void disableInterrupt(InterruptVector vector) {
        NVIC().disableIrq(vector);
    }
    static void setPriorityForInterrupt(InterruptVector vector, uint8_t priority) {
        NVIC().setIrqPriority(vector, (uint8_t)(priority << 4)); // The processor only uses the upper 4 bits of the byte
    }
    
private:
    static SystemControlProvider SCB;
    static NVICProvider NVIC;
    static InterruptHandler handlers[MAX_VECTOR];
    friend void interruptHandler();
};


#endif /* InterruptManager_hpp */
