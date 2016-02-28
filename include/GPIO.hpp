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
#include "types.hpp"


namespace Bedrock {
    namespace GPIO {
        class GPIOPort {
        public:
            dev_reg32_t MODE;
            dev_reg32_t OTYPE;
            dev_reg32_t OSPEED;
            dev_reg32_t PUPD;
            dev_reg32_t IDR;
            dev_reg32_t ODR;
            dev_reg32_t BSR;
            dev_reg32_t LCK;
            dev_reg32_t AFL;
            dev_reg32_t AFH;
            dev_reg32_t BR;
            
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
        
        enum class AlternateFunction : uint32_t {
            AF0,  AF1,  AF2,  AF3,
            AF4,  AF5,  AF6,  AF7,
            AF8,  AF9,  AF10, AF11,
            AF12, AF13, AF14, AF15
        };
        
        
        
        /**
         * Typdef a function type to return instance of GPIOPort
         * We add a level of indirection here to allow for easy mock testing
         */
        using GPIOPortProvider = GPIOPort& (*)(void);
        
        class GPIOPin {
        private:
            GPIOPortProvider port;
            uint8_t pin;
            
        public:
            
            GPIOPin(GPIOPortProvider port, uint8_t pin) :
                    port(port), pin(pin) {  }
            
            void setAlternateFunction(AlternateFunction af) {
                //Setup the pin's alternate function
                dev_reg32_t& afReg = (pin > 7) ? port().AFH : port().AFL;
                const uint8_t regShift = (pin % 8) * 4;
                afReg &= ~(static_cast<uint32_t>(AlternateFunction::AF15) << regShift); // clear the current AF
                afReg |= static_cast<uint32_t>(af) << regShift;
            }
            
            void setMode(PinMode mode) {
                const uint32_t modeShift = pin * 2;
                
                //clear the current config
                port().MODE &= ~(0b11 << modeShift);
                port().MODE |= (static_cast<uint32_t>(mode) << modeShift);
            }
            void setOutSpeed(OutputSpeed speed) {
                const uint32_t speedShift = pin * 2;
                port().OSPEED &= ~(0b11 << speedShift);
                port().OSPEED |= static_cast<uint32_t>(speed) << speedShift;
            }
            void setOutputType(OutputType type) {
                port().OTYPE &= ~(1 << pin);
                port().OTYPE |= static_cast<uint32_t>(type) << pin;
            }
            void setPullType(IOPullType type) {
                const uint32_t pullShift = pin * 2;
                port().PUPD &= ~(0b11 << pullShift);
                port().PUPD |= static_cast<uint32_t>(type) << pullShift;
            }
            
            bool isOn() {
                return (port().ODR >> pin) & 0x1;
            }
            
            void on() {
                port().ODR |= (1 << pin);
            }
            void off() {
                port().ODR &= ~(1 << pin);
            }
            void toggle() {
                port().ODR ^= (1 << pin);
            }
        };
    }
}


#endif /* GPIO_h */
