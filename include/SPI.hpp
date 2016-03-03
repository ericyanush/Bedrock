//
//  SPI.hpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-02-28.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef SPI_h
#define SPI_h

#include "types.hpp"

namespace Bedrock {
    
    class SPI {
    public:
        dev_reg16_t CR1;
        dev_reg16_t __pad1; //anonymous padding
        dev_reg16_t CR2;
        dev_reg16_t __pad2; //anonymous padding
        dev_reg16_t SR;
        dev_reg16_t __pad3; //anonymous padding
        dev_reg16_t DR;
        dev_reg16_t __pad4; //anonymous padding
        dev_reg16_t CRCPR;
        dev_reg16_t __pad5; //anonymous padding
        dev_reg16_t RXCRCR;
        dev_reg16_t __pad6; //anonymous padding
        dev_reg16_t TXCRCR;
        dev_reg16_t __pad7; //anonymous padding
        dev_reg16_t I2SCFGR;
        dev_reg16_t __pad8; //anonymous padding
        dev_reg16_t I2SPR;
        dev_reg16_t __pad9; //anonymous padding
        
        void enableSoftareSlaveManagement()
        {
            CR1 |= ENABLE << 9;
        }
        
        void disableSoftwareSlaveManagement()
        {
            CR1 &= ~(ENABLE << 9);
        }
        
        void selectSlave()
        {
            //Note SS is active LOW
            CR1 &= ~(ENABLE << 8);
        }
        
        void deselectSlave()
        {
            //Note SS is active LOW
            CR1 |= ENABLE << 8;
        }
        
        enum class FrameFormat {
            MSBFirst = 0,
            LSBFirst = 1
        };
        
        void setFrameFormat(FrameFormat format) {
            if (format == FrameFormat::LSBFirst) {
                CR1 |= (ENABLE << 7);
            }
            else {
                CR1 &= ~(ENABLE << 7);
            }
        }
        
        FrameFormat getFrameFormat() {
            uint8_t f = (CR1 >> 7) & 0x1;
            return static_cast<FrameFormat>(f);
        }
        
        void enable() {
            CR1 |= (ENABLE << 6);
        }
        void disable() {
            CR1 &= ~(ENABLE << 6);
        }
        
        enum class BaudRatePrescaler
        {
            Div2   = 0b000,
            Div4   = 0b001,
            Div8   = 0b010,
            Div16  = 0b011,
            Div32  = 0b100,
            Div64  = 0b101,
            Div128 = 0b110,
            Div256 = 0b111
        };
        void setBaudRatePrescaler(BaudRatePrescaler pre)
        {
            CR1 &= ~(0b111 << 3); //clear current value
            CR1 |= static_cast<uint32_t>(pre) << 3;
        }
        BaudRatePrescaler getBaudRatePrescaler()
        {
            return static_cast<BaudRatePrescaler>((CR1 >> 3) & 0b111);
        }
        
        enum class DeviceMode
        {
            Slave  = 0,
            Master = 1
        };
        
        
        void setDeviceMode(DeviceMode mode)
        {
            if (mode == DeviceMode::Slave)
            {
                CR1 &= ~(ENABLE << 2);
            }
            else
            {
                CR1 |= (ENABLE << 2);
            }
        }
        
        DeviceMode getDeviceMode()
        {
            return static_cast<DeviceMode>((CR1 >> 2) & 0x1);
        }
        
        enum class SPIMode {
            Zero  = 0b00,
            One   = 0b01,
            Two   = 0b10,
            Three = 0b11
        };
        
        void setSPIMode(SPIMode mode) {
            CR1 &= ~(0b11); //Clear current value;
            CR1 |= static_cast<uint32_t>(mode);
        }
        SPIMode getSPIMode() {
            return static_cast<SPIMode>(CR1 & 0b11);
        }
        
        enum class TransmitSize {
            Bits4  = 0b0011,
            Bits5  = 0b0100,
            Bits6  = 0b0101,
            Bits7  = 0b0110,
            Bits8  = 0b0111,
            Bits9  = 0b1000,
            Bits10 = 0b1001,
            Bits11 = 0b1010,
            Bits12 = 0b1011,
            Bits13 = 0b1100,
            Bits14 = 0b1101,
            Bits15 = 0b1110,
            Bits16 = 0b1111
        };
        
        void setTransmitSize(TransmitSize size) {
            CR2 &= ~(0b1111 << 8);
            CR2 |= static_cast<uint32_t>(size) << 8;
        }
        
        TransmitSize getTransmitSize() {
            return static_cast<TransmitSize>((CR2 >> 8) & 0b1111);
        }
        
        void enableTXBufferEmptyInterrupt() {
            CR2 |= ENABLE << 7;
        }
        void disableTXBufferEmptyInterrupt() {
            CR2 &= ~(ENABLE << 7);
        }
        
        void enableRXBufferNotEmptyInterrupt() {
            CR2 |= ENABLE << 6;
        }
        void disableRXBufferNotEmptyInterrupt() {
            CR2 &= ~(ENABLE << 6);
        }
        
        void enableErrorInterrupt() {
            CR2 |= ENABLE << 5;
        }
        void disableErrorInterrupt() {
            CR2 &= ~(ENABLE << 5);
        }
        
        void enableTXDMA() {
            CR2 |= ENABLE << 1;
        }
        void disableTXDMA() {
            CR2 &= ~(ENABLE << 1);
        }
        
        void enableRXDMA() {
            CR2 |= (ENABLE << 0);
        }
        void disableRXDMA() {
            CR2 &= ~(ENABLE << 0);
        }
        
        enum class BufferLevel {
            Empty   = 0b00,
            Quarter = 0b01,
            Half    = 0b10,
            Full    = 0b11
        };
        
        BufferLevel getTXLevel() {
            return static_cast<BufferLevel>((SR >> 11) & 0b11);
        }
        
        BufferLevel getRXLevel() {
            return static_cast<BufferLevel>((SR >> 9) & 0b11);
        }
        
        bool isBusy() {
            return (SR >> 7) & 0x1;
        }
        
        bool txEmpty() {
            return (SR >> 1) & 0x1;
        }
        
        bool rxNotEmpty() {
            return (SR >> 0) & 0x1;
        }
        
    };
    using SPIProvider = SPI& (*)(void);
}

#endif /* SPI_h */
