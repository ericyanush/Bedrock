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

extern "C" {
    void interruptDispatcher();
}

namespace Bedrock {
    using InterruptHandler = std::function<void(void)>;
    
    class InterruptManager {
    public:
        static void init(SystemControlProvider sysCtl, NVICProvider nvic);
        static void setHandlerForInterrupt(InterruptVector vector, InterruptHandler handler);
        static void enableInterrupt(InterruptVector vector);
        static void disableInterrupt(InterruptVector vector);
        static void setPriorityForInterrupt(InterruptVector vector, uint8_t priority);
        
    private:
        //The smallest System Vector is -14 (NMI), so to handle it, we need to offset
        //  all the vectors by this amount to position NMI at 0 in our handler array
        static constexpr uint8_t SystemVectorsOffset = 14;
        static SystemControl* scb;
        static NVIC* nvic;
        static InterruptHandler handlers[MAX_VECTOR + SystemVectorsOffset]; //Add 14 for system vectors
        friend void ::interruptDispatcher();
    };

}

#endif /* InterruptManager_hpp */
