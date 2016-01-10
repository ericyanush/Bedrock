//
//  types.h
//  Bedrock Types
//
//  Created by Eric Yanush on 2015-12-17.
//  Copyright © 2015 EricYanush. All rights reserved.
//

#ifndef types_h
#define types_h

#include <stdint.h>


using dev_reg32_t =  volatile uint32_t;
using dev_reg16_t = volatile uint16_t;
using dev_reg8_t = volatile uint8_t;

constexpr uint32_t ENABLE = 0x1;
constexpr uint32_t DISABLE = 0x0;

enum class Irq : int32_t;

#endif /* types_h */
