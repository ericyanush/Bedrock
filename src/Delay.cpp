//
//  Delay.cpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-17.
//  Copyright © 2016 EricYanush. All rights reserved.
//
#include "Delay.hpp"

using namespace Bedrock;

volatile uint32_t Delay::msCount = 0;

void Delay::timerUpdateEvent() {
    msCount++;
}
