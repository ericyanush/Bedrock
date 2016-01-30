//
//  InterruptManager.cpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-15.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "InterruptManager.hpp"
#include "Interrupts.hpp"

Bedrock::SystemControl* Bedrock::InterruptManager::scb;
Bedrock::NVIC*  Bedrock::InterruptManager::nvic;

//Bedrock::InterruptHandler Bedrock::InterruptManager::handlers[MAX_VECTOR + InterruptManager::SystemVectorsOffset];
std::map<InterruptVector, Bedrock::InterruptHandler> Bedrock::InterruptManager::handlers;

extern "C" {
    void interruptDispatcher() {
        InterruptVector activeVect = Bedrock::InterruptManager::scb->getActiveVector();

        auto target = Bedrock::InterruptManager::handlers.find(activeVect);
        if (target != Bedrock::InterruptManager::handlers.end()) {
            target->second();
        }
    }
}

void Bedrock::InterruptManager::init(SystemControlProvider sysCtl, NVICProvider intController) {
    scb = &sysCtl();
    nvic = &intController();
}
void Bedrock::InterruptManager::setHandlerForInterrupt(InterruptVector vector, InterruptHandler handler) {
    handlers.emplace(vector, handler);
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