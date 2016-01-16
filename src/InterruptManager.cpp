//
//  InterruptManager.cpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-15.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "InterruptManager.hpp"

extern "C" {
    void InterruptManager::interruptHandler() {
        int i = 12;
        i ++;
        return;
    }
}