//
//  InterruptManager.cpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-15.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "InterruptManager.hpp"
#include "Interrupts.hpp"

Bedrock::InterruptManager Bedrock::InterruptManager::privateInstance;

extern "C" {
    void interruptDispatcher() {
        using IntMan = Bedrock::InterruptManager;
        IntMan& manager = IntMan::instance();
        InterruptVector activeVect = manager.scb->getActiveVector();
        uint16_t vecIdx = static_cast<uint16_t>(activeVect) + manager.SystemVectorCount;

        if (manager.handlers[vecIdx] != nullptr) {
            manager.handlers[vecIdx]();
        }
    }
}

void Bedrock::InterruptManager::init(SystemControlProvider sysCtl, NVICProvider intController) {
    scb = &sysCtl();
    nvic = &intController();
}
void Bedrock::InterruptManager::setHandlerForInterrupt(InterruptVector vector, InterruptHandler handler) {
    uint16_t vecPos = static_cast<uint16_t>(vector) + SystemVectorCount;
    handlers[vecPos] = handler;
}

Bedrock::InterruptHandler Bedrock::InterruptManager::getHandlerForInterrupt(InterruptVector vector) {
    uint16_t vecPos = static_cast<uint16_t>(vector) + SystemVectorCount;
    return handlers[vecPos];
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

Bedrock::InterruptManager& Bedrock::InterruptManager::instance() {
    return privateInstance;
}