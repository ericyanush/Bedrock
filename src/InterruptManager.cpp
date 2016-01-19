//
//  InterruptManager.cpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-15.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "InterruptManager.hpp"
#include "Interrupts.hpp"

SystemControl* InterruptManager::SCB;
NVIC* InterruptManager::NVIC;

InterruptHandler InterruptManager::handlers[MAX_VECTOR];

extern "C" {
    void interruptHandler() {
        InterruptVector activeVect = InterruptManager::SCB->getActiveVector();
        int32_t vecNum = static_cast<int32_t>(activeVect); //Need to subtract 16 to get vector number
        if (InterruptManager::handlers[vecNum]) {
            InterruptManager::handlers[vecNum]();
        }
    }
}
