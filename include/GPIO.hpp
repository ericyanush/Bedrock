//
//  GPIO.hpp
//  Bedrock GPIO object for STM32F3
//
//  Created by Eric Yanush on 2015-12-19.
//  Copyright © 2015 EricYanush. All rights reserved.
//

#ifndef GPIO_h
#define GPIO_h

#include <stdint.h>
#include "types.h"


class GPIOPort {
public:
    dev_reg MODE;
    dev_reg OTYPE;
    dev_reg OSPEED;
    dev_reg PUPD;
    dev_reg IDR;
    dev_reg ODR;
    dev_reg BSR;
    dev_reg LCK;
    dev_reg AFL;
    dev_reg AFH;
    dev_reg BR;
    
private:
};


enum class PinMode : uint32_t {
    input         = 0b00,
    output        = 0b01,
    alternateFunc = 0b10,
    analog        = 0b11
};

enum class OutputSpeed : uint32_t {
    low    = 0b00,
    medium = 0b01,
    high   = 0b11
};

enum class OutputType : uint32_t {
    pushpull  = 0,
    opendrain = 1
};

enum class IOPullType : uint32_t {
    none = 0b00,
    up   = 0b01,
    down = 0b10
};


/**
 * Typdef a function type to return instance of GPIOPort
 * We add a level of indirection here to allow for easy mock testing
 */

typedef GPIOPort* (GPIOPortProvider)(void);

template <GPIOPortProvider port, uint8_t pin>
class GPIOPin {
private:
    static_assert(pin >= 0 && pin <= 15, "Pin Number out of range!");
public:
    static void setMode(PinMode mode) {
        constexpr uint32_t modeShift = pin * 2;
        
        //clear the current config
        port()->MODE &= ~(0b11 << modeShift);
        port()->MODE |= (static_cast<uint32_t>(mode) << modeShift);
    }
    static void setOutSpeed(OutputSpeed speed) {
        constexpr uint32_t speedShift = pin * 2;
        port()->OSPEED &= ~(0b11 << speedShift);
        port()->OSPEED |= static_cast<uint32_t>(speed) << speedShift;
    }
    static void setOutputType(OutputType type) {
        port()->OTYPE &= ~(1 << pin);
        port()->OTYPE |= static_cast<uint32_t>(type) << pin;
    }
    static void setPullType(IOPullType type) {
        constexpr uint32_t pullShift = pin * 2;
        port()->PUPD &= ~(0b11 << pullShift);
        port()->PUPD |= static_cast<uint32_t>(type) << pullShift;
    }

    static bool isOn() {
        return (port()->ODR >> pin) & 0x1;
    }
    
    static void on() {
        port()->ODR |= (1 << pin);
    }
    static void off() {
        port()->ODR &= ~(1 << pin);
    }
    static void toggle() {
        port()->ODR ^= (1 << pin);
    }
    
};


#endif /* GPIO_h */
