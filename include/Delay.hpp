//
//  Delay.hpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-17.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef Delay_h
#define Delay_h

#include "SysTick.hpp"
#include "InterruptManager.hpp"
/**
 General Delay functions
 */

namespace Bedrock {
    class Delay {
    public:

        /**
         Initialize Systick to provide the required behaviour
         */
        template <uint32_t sysclockFreq, SysTickProvider SysTick>
        static void init() {
            msCount = 0;
            SysTick().setReload(sysclockFreq/1000);
            SysTick().setClockSourceAHB();

            InterruptManager& manager = InterruptManager::instance();
            manager.setHandlerForInterrupt(InterruptVector::SysTick, timerUpdateEvent);
            SysTick().enableReloadInterrupt();
            SysTick().enable();
        }

        static uint32_t getMillis() {
            return msCount;
        }

        static void ms(uint32_t millis) {
            uint32_t target = msCount + millis;
            while(msCount < target);
        }
    private:
        static volatile uint32_t msCount;
        static void timerUpdateEvent();
    };
}

#endif /* Delay_h */
