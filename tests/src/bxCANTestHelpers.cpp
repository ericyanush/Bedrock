//
//  bxCANTestHelpers.cpp
//  Bedrock-xcode
//
//  Created by Eric Yanush on 2016-01-31.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "bxCANTestHelpers.hpp"
#include <thread>
#include <chrono>

std::thread simulateCAN_enter_exit_init(Bedrock::bxCAN::CANPort& port) {
    
    auto asyncHW = [&port]() {
        using namespace std::literals;
        
        //Wait for an init mode request
        while ((port.MCR & 0x1) != 0x1) {
            std::this_thread::sleep_for(100us);
        }
        port.MSR |= 0x1; // set the INAK bit
        //Wait for request to leave init mode
        while ((port.MCR & 0x1) == 0x1) {
            std::this_thread::sleep_for(100us);
        }
        port.MSR &= ~(0x1);
    };
    return std::thread(asyncHW);
}



std::thread simulateCAN_enter_init(Bedrock::bxCAN::CANPort& port) {
   
    auto asyncHW = [&port]() {
        using namespace std::literals;
        
        //Wait for an init mode request
        while ((port.MCR & 0x1) != 0x1) {
            std::this_thread::sleep_for(100us);
        }
        port.MSR |= 0x1; // set the INAK bit
    };
    return std::thread(asyncHW);
}

std::thread simulateCAN_exit_init(Bedrock::bxCAN::CANPort& port) {
    auto asyncHW = [&port]() {
        using namespace std::literals;
        
        //Wait for request to leave init mode
        while ((port.MCR & 0x1) == 0x1) {
            std::this_thread::sleep_for(100us);
        }
        port.MSR &= ~(0x1);
    };
    return std::thread(asyncHW);
}