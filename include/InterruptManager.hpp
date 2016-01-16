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
    static void setHandlerForInterrupt(InterruptVector vector, InterruptHandler handler);
    static void enableInterrupt(InterruptVector vector);
    static void disableInterrupt(InterruptVector vector);
    static void setPriorityForInterrupt(InterruptVector vector, uint8_t priority);
    
private:
    friend void interruptHandler();
};


#endif /* InterruptManager_hpp */
