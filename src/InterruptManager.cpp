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

static void init(SystemControlProvider sysCtl, NVICProvider nvic) {
    SCB = &sysCtl();
    NVIC = &nvic();
}
static void setHandlerForInterrupt(InterruptVector vector, InterruptHandler handler) {
    handlers[static_cast<uint32_t>(vector)] = handler;
}
static void enableInterrupt(InterruptVector vector) {
    NVIC->enableIrq(vector);
}
static void disableInterrupt(InterruptVector vector) {
    NVIC->disableIrq(vector);
}
static void setPriorityForInterrupt(InterruptVector vector, uint8_t priority) {
    NVIC->setIrqPriority(vector, (uint8_t)(priority << 4)); // The processor only uses the upper 4 bits of the byte
}