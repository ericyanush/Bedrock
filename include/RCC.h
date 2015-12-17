//
//  RCC.h
//  Bedrock RCC object for STM32F3
//
//  Created by Eric Yanush on 2015-12-17.
//  Copyright © 2015 EricYanush. All rights reserved.
//

#ifndef RCC_h
#define RCC_h

#include "types.h"

class RCC {
public:
    
    void enableGPIOA() {
        AHBENR |= (ENABLE << 17);
    }
    
    void enableGPIOB() {
        AHBENR |= (ENABLE << 18);
    }
    
    void enableGPIOC() {
        AHBENR |= (ENABLE << 19);
    }
    
    void enableGPIOD() {
        AHBENR |= (ENABLE << 20);
    }
    
    void enableGPIOE() {
        AHBENR |= (ENABLE << 21);
    }
    
    void enableGPIOF() {
        AHBENR |= (ENABLE << 22);
    }
    
    void enableGPIOG() {
        AHBENR |= (ENABLE << 23);
    }
    
    void enableGPIOH() {
        AHBENR |= (ENABLE << 16);
    }
    
    dev_reg CR;
    dev_reg CFG;
    dev_reg CIR;
    dev_reg APB2RST;
    dev_reg APB1RST;
    dev_reg AHBENR;
    dev_reg APB2ENR;
    dev_reg APB1ENR;
    dev_reg BDC;
    dev_reg CSR;
    dev_reg AHBRST;
    dev_reg CFG2;
    dev_reg CFG3;
};


#endif /* RCC_h */
