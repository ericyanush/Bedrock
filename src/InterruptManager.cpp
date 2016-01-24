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

Bedrock::InterruptHandler Bedrock::InterruptManager::handlers[MAX_VECTOR + InterruptManager::SystemVectorsOffset];

extern "C" {
    void interruptDispatcher() {
        InterruptVector activeVect = Bedrock::InterruptManager::scb->getActiveVector();
        int32_t vecNum = static_cast<int32_t>(activeVect) + Bedrock::InterruptManager::SystemVectorsOffset; //Need to add 14 to account for system vectors
        if (Bedrock::InterruptManager::handlers[vecNum]) {
            Bedrock::InterruptManager::handlers[vecNum]();
        }
    }
}

void Bedrock::InterruptManager::init(SystemControlProvider sysCtl, NVICProvider intController) {
    scb = &sysCtl();
    nvic = &intController();
}
void Bedrock::InterruptManager::setHandlerForInterrupt(InterruptVector vector, InterruptHandler handler) {
    handlers[static_cast<uint32_t>(vector) + SystemVectorsOffset] = handler;
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