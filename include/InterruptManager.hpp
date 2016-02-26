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
#include <array>

extern "C" {
    void interruptDispatcher();
}

namespace Bedrock {
    using InterruptHandler = std::function<void(void)>;
    
    class InterruptManager {
    public:
        void init(SystemControlProvider sysCtl, NVICProvider nvic);
        void setHandlerForInterrupt(InterruptVector vector, InterruptHandler handler);
        InterruptHandler getHandlerForInterrupt(InterruptVector vector);
        void enableInterrupt(InterruptVector vector);
        void disableInterrupt(InterruptVector vector);
        bool isInterruptEnabled(InterruptVector vector);
        void setPriorityForInterrupt(InterruptVector vector, uint8_t priority);
        static InterruptManager& instance();
    private:
        SystemControl* scb;
        NVIC* nvic;

        struct HandlerLink {
            InterruptHandler handler;
            InterruptVector vector;
        };

        static constexpr uint16_t SystemVectorCount = 16;
        std::array<InterruptHandler, MAX_VECTOR + SystemVectorCount> handlers;

        friend void ::interruptDispatcher();

        static InterruptManager privateInstance;
    };

}

#endif /* InterruptManager_hpp */
