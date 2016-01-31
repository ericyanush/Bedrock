//
//  bxCANTestHelpers.hpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-31.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef bxCANTestHelpers_h
#define bxCANTestHelpers_h

#include "bxCAN.hpp"
#include <thread>

std::thread simulateCAN_enter_exit_init(Bedrock::bxCAN::CANPort& port);
std::thread simulateCAN_enter_init(Bedrock::bxCAN::CANPort& port);
std::thread simulateCAN_exit_init(Bedrock::bxCAN::CANPort& port);

#endif /* bxCANTestHelpers_h */