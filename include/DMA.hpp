//
//  DMA.hpp
//  Bedrock-xcode
//
//  Created by Eric Yanush on 2016-03-02.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef DMA_h
#define DMA_h

#include "types.hpp"

namespace Bedrock
{
    class DMAController {
    public:
        dev_reg32_t ISR;
        dev_reg32_t IFCR;
        
        struct ChannelConfig {
            dev_reg32_t CCR;
            dev_reg32_t CNDTR;
            dev_reg32_t CPAR;
            dev_reg32_t CMAR;
            dev_reg32_t _padding;
        };
        
        ChannelConfig channels[7];
    };
    
    using DMAControllerProvider = DMAController& (*)(void);
    
    class DMAChannel
    {
        
        DMAController& controller;
        const uint8_t channel;
        
    public:
        DMAChannel(DMAControllerProvider provider, uint8_t channelNum) :
        controller(provider()),
        channel(channelNum - 1)
        { }
        
        bool transferError()
        {
            return (controller.ISR >> ((channel * 4) + 3)) & 0x1;
        }
        bool halfTransferComplete()
        {
            return (controller.ISR >> ((channel * 4) + 2)) & 0x1;
        }
        bool transferComplete()
        {
            return (controller.ISR >> ((channel * 4) + 1)) & 0x1;
        }
        
        void ackTransferError()
        {
            controller.IFCR |= (0x1 << ((channel * 4) + 3));
        }
        void acklHalfTransferComplete()
        {
            controller.IFCR |= (0x1 << ((channel * 4) + 2));
        }
        void ackTransferComplete()
        {
            controller.IFCR |= (0x1 << ((channel * 4) + 1));
        }
        
        enum class ChannelPriority
        {
            Low      = 0b00,
            Medium   = 0b01,
            High     = 0b10,
            VeryHigh = 0b11
        };
        
        void setChannelPriorityLevel(ChannelPriority prio)
        {
            controller.channels[channel].CCR &= ~(0b11 << 12); //Clear current Val
            controller.channels[channel].CCR |= static_cast<uint32_t>(prio) << 12;
        }
        
        ChannelPriority getChannelPriority()
        {
            return static_cast<ChannelPriority>(
                (controller.channels[channel].CCR >> 12) & 0b11
            );
        }
        
        enum class TransferSize
        {
            Bits_8  = 0b00,
            Bits_16 = 0b01,
            Bits_32 = 0b10
        };
        void setChannelMemorySize(TransferSize size)
        {
            controller.channels[channel].CCR &= ~(0b11 << 10);
            controller.channels[channel].CCR |= static_cast<uint32_t>(size) << 10;
        }
        void setChannelPeripheralSize(TransferSize size)
        {
            controller.channels[channel].CCR &= ~(0b11 << 8);
            controller.channels[channel].CCR |= static_cast<uint32_t>(size) << 8;
        }
        
        void enableMemoryIncrementMode()
        {
            controller.channels[channel].CCR |= ENABLE << 7;
        }
        void disableMemoryIncrementMode()
        {
            controller.channels[channel].CCR &= ~(ENABLE << 7);
        }
        void enablePeripheralIncrementMode()
        {
            controller.channels[channel].CCR |= ENABLE << 6;
        }
        void disablePeripheralIncrementMode()
        {
            controller.channels[channel].CCR &= ~(ENABLE << 6);
        }
        
        void enableCircularMode()
        {
            controller.channels[channel].CCR |= ENABLE << 5;
        }
        void disableCircularMode()
        {
            controller.channels[channel].CCR &= ~(ENABLE << 5);
        }
        
        enum class TransferSource {
            Peripheral = 0,
            Memeory    = 1
        };
        void setTransferSource(TransferSource source)
        {
            if (source == TransferSource::Peripheral) {
                controller.channels[channel].CCR &= ~(ENABLE << 4);
            }
            else {
                controller.channels[channel].CCR |= ENABLE << 4;
            }
        }
        
        TransferSource getTransferSource()
        {
            return static_cast<TransferSource>(
                    (controller.channels[channel].CCR >> 4) & 0b1
            );
        }
        
        void enableHalfTransferInterrupt()
        {
            controller.channels[channel].CCR |= ENABLE << 2;
        }
        void disableHalfTransferInterrupt()
        {
            controller.channels[channel].CCR &= ~(ENABLE << 2);
        }
        
        void enableTransferCompleteInterrupt()
        {
            controller.channels[channel].CCR |= ENABLE << 1;
        }
        void disableTransferCompleteInterrupt()
        {
            controller.channels[channel].CCR &= ~(ENABLE << 1);
        }
        
        void enableTransferErrorInterrupt()
        {
            controller.channels[channel].CCR |= ENABLE << 3;
        }
        void disableTransferErrorInterrupt()
        {
            controller.channels[channel].CCR &= ~(ENABLE << 3);
        }
        
        void setTransferCount(uint16_t count)
        {
            controller.channels[channel].CNDTR = ((uint32_t)count);
        }
        
        uint16_t getTransferCount()
        {
            return (uint16_t)controller.channels[channel].CNDTR;
        }
        
        void setPeripheralRegisterAddress(void* regAddress)
        {
            //Cast to uintptr_t to avoid pointer type narrowing errors on
            // 64bit test host machines, on 32bit hardware this is irrelevant
            controller.channels[channel].CPAR = (uint32_t)reinterpret_cast<uintptr_t>(regAddress);
        }
        
        void setMemoryAddress(void* memAddress)
        {
            //Cast to uintptr_t to avoid pointer type narrowing errors on
            // 64bit test host machines, on 32bit hardware this is irrelevant
            controller.channels[channel].CMAR = (uint32_t)reinterpret_cast<uintptr_t>(memAddress);
        }
        
        void enable()
        {
            controller.channels[channel].CCR |= ENABLE;
        }
        void disable()
        {
            controller.channels[channel].CCR &= ~(ENABLE);
        }
    };
}


#endif /* DMA_h */
