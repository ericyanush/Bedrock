//
//  InterruptManager.cpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-15.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "InterruptManager.hpp"
#include "Interrupts.hpp"

static Bedrock::SystemControl* scb;
static Bedrock::NVIC* nvic;

static Bedrock::InterruptHandler handlers[MAX_VECTOR];

extern "C" {
    void interruptDispatcher() {
        InterruptVector activeVect = scb->getActiveVector();
        int32_t vecNum = static_cast<int32_t>(activeVect); //Need to subtract 16 to get vector number
        if (handlers[vecNum]) {
            handlers[vecNum]();
        }
    }
}

void Bedrock::InterruptManager::init(SystemControlProvider sysCtl, NVICProvider intController) {
    scb = &sysCtl();
    nvic = &intController();
}
void Bedrock::InterruptManager::setHandlerForInterrupt(InterruptVector vector, InterruptHandler handler) {
    handlers[static_cast<uint32_t>(vector)] = handler;
}
void Bedrock::InterruptManager::enableInterrupt(InterruptVector vector) {
    nvic->enableIrq(vector);
}
void Bedrock::InterruptManager::disableInterrupt(InterruptVector vector) {
    nvic->disableIrq(vector);
}
void Bedrock::InterruptManager::setPriorityForInterrupt(InterruptVector vector, uint8_t priority) {
    nvic->setIrqPriority(vector, (uint8_t)(priority << 4)); // The processor only uses the upper 4 bits of the byte
}