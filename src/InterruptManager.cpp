//
//  InterruptManager.cpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-15.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "InterruptManager.hpp"
#include "Interrupts.hpp"

static SystemControl* SCB;
static NVIC* NVIC;

static InterruptHandler handlers[MAX_VECTOR];

extern "C" {
    void interruptHandler() {
        InterruptVector activeVect = SCB->getActiveVector();
        int32_t vecNum = static_cast<int32_t>(activeVect); //Need to subtract 16 to get vector number
        if (handlers[vecNum]) {
            handlers[vecNum]();
        }
    }
}

void InterruptManager::init(SystemControlProvider sysCtl, NVICProvider nvic) {
    SCB = &sysCtl();
    NVIC = &nvic();
}
void InterruptManager::setHandlerForInterrupt(InterruptVector vector, InterruptHandler handler) {
    handlers[static_cast<uint32_t>(vector)] = handler;
}
void InterruptManager::enableInterrupt(InterruptVector vector) {
    NVIC->enableIrq(vector);
}
void InterruptManager::disableInterrupt(InterruptVector vector) {
    NVIC->disableIrq(vector);
}
void InterruptManager::setPriorityForInterrupt(InterruptVector vector, uint8_t priority) {
    NVIC->setIrqPriority(vector, (uint8_t)(priority << 4)); // The processor only uses the upper 4 bits of the byte
}