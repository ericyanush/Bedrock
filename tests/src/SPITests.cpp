//
//  SPITests.cpp
//  Bedrock tests
//
//  Created by Eric Yanush on 2016-02-28.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "SPI.hpp"
#include "gtest/gtest.h"

class SPITests : public ::testing::Test {
protected:
    Bedrock::SPI spi{0};
    
    void SetUp()
    {
        //set register reset values that are not 0
        spi.CR2 = 0x700;
        spi.SR = 0x2;
        spi.CRCPR = 0x7;
        spi.I2SPR = 0x2;
    }
};

TEST_F(SPITests, TestRegisterLayout)
{
    ASSERT_EQ(0x00, offsetof(Bedrock::SPI, CR1));
    ASSERT_EQ(0x04, offsetof(Bedrock::SPI, CR2));
    ASSERT_EQ(0x08, offsetof(Bedrock::SPI, SR));
    ASSERT_EQ(0x0C, offsetof(Bedrock::SPI, DR));
    ASSERT_EQ(0x10, offsetof(Bedrock::SPI, CRCPR));
    ASSERT_EQ(0x14, offsetof(Bedrock::SPI, RXCRCR));
    ASSERT_EQ(0x18, offsetof(Bedrock::SPI, TXCRCR));
    ASSERT_EQ(0x1C, offsetof(Bedrock::SPI, I2SCFGR));
    ASSERT_EQ(0x20, offsetof(Bedrock::SPI, I2SPR));
    
    ASSERT_EQ(0x24, sizeof(Bedrock::SPI));
}

TEST_F(SPITests, TestEnableSoftwareSlaveManagement)
{
    spi.enableSoftareSlaveManagement();
    ASSERT_EQ(0x1 << 9, spi.CR1);
}

TEST_F(SPITests, TestDisableSoftwareSlaveManagement)
{
    spi.CR1 |= 0x1 << 9;
    spi.disableSoftwareSlaveManagement();
    ASSERT_EQ(0, spi.CR1);
}

TEST_F(SPITests, TestSelectSlave)
{
    spi.CR1 |= 0x1 << 8; //Slave is active LOW (idles high)
    spi.selectSlave();
    ASSERT_EQ(0, spi.CR1);
}

TEST_F(SPITests, TestDeselectSlave)
{
    spi.CR1 = 0; //Slave is active LOW (idles high)
    spi.deselectSlave();
    ASSERT_EQ(0x1 << 8, spi.CR1);
}

TEST_F(SPITests, TestSetFrameFormat)
{
    spi.setFrameFormat(Bedrock::SPI::FrameFormat::LSBFirst);
    ASSERT_EQ(0x1 << 7, spi.CR1);
    spi.CR1 &= ~(0x1 << 7);
    
    spi.setFrameFormat(Bedrock::SPI::FrameFormat::MSBFirst);
    ASSERT_EQ(0x0, spi.CR1);
}

TEST_F(SPITests, TestGetFrameFormat)
{
    using Format = Bedrock::SPI::FrameFormat;
    
    ASSERT_EQ(Format::MSBFirst, spi.getFrameFormat());
    spi.setFrameFormat(Format::LSBFirst);
    ASSERT_EQ(Format::LSBFirst, spi.getFrameFormat());
}

TEST_F(SPITests, TestEnable)
{
    spi.enable();
    ASSERT_EQ(0x1 << 6, spi.CR1);
}

TEST_F(SPITests, TestDisable)
{
    spi.CR1 |= 0x1 << 6;
    spi.disable();
    
    ASSERT_EQ(0, spi.CR1);
}

TEST_F(SPITests, TestSetPrescaler)
{
    using Pre = Bedrock::SPI::BaudRatePrescaler;
    
    spi.setBaudRatePrescaler(Pre::Div32);
    ASSERT_EQ(static_cast<uint16_t>(Pre::Div32) << 3, spi.CR1);
    spi.setBaudRatePrescaler(Pre::Div2);
    ASSERT_EQ(static_cast<uint16_t>(Pre::Div2) << 3, spi.CR1);
    spi.setBaudRatePrescaler(Pre::Div256);
    ASSERT_EQ(static_cast<uint16_t>(Pre::Div256) << 3, spi.CR1);
}

TEST_F(SPITests, TestGetPrescaler)
{
    using Pre = Bedrock::SPI::BaudRatePrescaler;
    
    ASSERT_EQ(Pre::Div2, spi.getBaudRatePrescaler());
    spi.setBaudRatePrescaler(Pre::Div64);
    ASSERT_EQ(Pre::Div64, spi.getBaudRatePrescaler());
}

TEST_F(SPITests, TestSetDeviceMode)
{
    spi.setDeviceMode(Bedrock::SPI::DeviceMode::Master);
    ASSERT_EQ(0x1 << 2, spi.CR1);
    spi.setDeviceMode(Bedrock::SPI::DeviceMode::Slave);
    ASSERT_EQ(0x0, spi.CR1);
}

TEST_F(SPITests, TestGetDeviceMode)
{
    using DevMode = Bedrock::SPI::DeviceMode;
    
    ASSERT_EQ(DevMode::Slave, spi.getDeviceMode());
    spi.setDeviceMode(DevMode::Master);
    ASSERT_EQ(DevMode::Master, spi.getDeviceMode());
}

TEST_F(SPITests, TestSetSPIMode)
{
    spi.setSPIMode(Bedrock::SPI::SPIMode::Two);
    ASSERT_EQ(0b10, spi.CR1);
    spi.setSPIMode(Bedrock::SPI::SPIMode::Zero);
    ASSERT_EQ(0b00, spi.CR1);
    spi.setSPIMode(Bedrock::SPI::SPIMode::Three);
    ASSERT_EQ(0b11, spi.CR1);
    spi.setSPIMode(Bedrock::SPI::SPIMode::One);
    ASSERT_EQ(0b01, spi.CR1);
}

TEST_F(SPITests, TestGetSPIMode)
{
    using Mode = Bedrock::SPI::SPIMode;
    
    ASSERT_EQ(Mode::Zero, spi.getSPIMode());
    spi.setSPIMode(Mode::Two);
    ASSERT_EQ(Mode::Two, spi.getSPIMode());
}

TEST_F(SPITests, TestSetTransmitSize)
{
    spi.setTransmitSize(Bedrock::SPI::TransmitSize::Bits14);
    ASSERT_EQ(0b1101 << 8, spi.CR2);
    spi.setTransmitSize(Bedrock::SPI::TransmitSize::Bits7);
    ASSERT_EQ(0b0110 << 8, spi.CR2);
}

TEST_F(SPITests, TestGetTransmitSize)
{
    using Size = Bedrock::SPI::TransmitSize;
    
    ASSERT_EQ(Size::Bits8, spi.getTransmitSize());
    spi.setTransmitSize(Size::Bits10);
    ASSERT_EQ(Size::Bits10, spi.getTransmitSize());
    spi.setTransmitSize(Size::Bits15);
    ASSERT_EQ(Size::Bits15, spi.getTransmitSize());
    spi.setTransmitSize(Size::Bits4);
    ASSERT_EQ(Size::Bits4, spi.getTransmitSize());
}

TEST_F(SPITests, TestEnableTXBufferEmptyInterrupt)
{
    spi.enableTXBufferEmptyInterrupt();
    ASSERT_EQ(0x1 << 7, spi.CR2 & (0x1 << 7));
}

TEST_F(SPITests, TestDisableTXBufferEmptyInterrupt)
{
    spi.CR2 |= (0x1 << 7);
    spi.disableTXBufferEmptyInterrupt();
    ASSERT_EQ(0x0, spi.CR2 & (0x1 << 7));
}

TEST_F(SPITests, TestEnableRXBufferNEInterrupt)
{
    spi.enableRXBufferNotEmptyInterrupt();
    ASSERT_EQ(0x1 << 6, spi.CR2 & (0x1 << 6));
}

TEST_F(SPITests, TestDisableRXBufferNEInterrupt)
{
    spi.CR2 |= 0x1 << 6;
    spi.disableRXBufferNotEmptyInterrupt();
    ASSERT_EQ(0x0, spi.CR2 & (0x1 << 6));
}

TEST_F(SPITests, TestEnableErrorInterrupt)
{
    spi.enableErrorInterrupt();
    ASSERT_EQ(0x1 << 5, spi.CR2 & (0x1 << 5));
}

TEST_F(SPITests, TestDisableErrorInterrupt)
{
    spi.CR2 |= 0x1 << 5;
    spi.disableErrorInterrupt();
    ASSERT_EQ(0x0, spi.CR2 & (0x1 << 5));
}

TEST_F(SPITests, TestEnableTXDMA)
{
    spi.enableTXDMA();
    ASSERT_EQ(0x1 << 1, spi.CR2 & (0x1 << 1));
}

TEST_F(SPITests, TestDisableTXDMA)
{
    spi.CR2 |= (0x1 << 1);
    spi.disableTXDMA();
    ASSERT_EQ(0x0, spi.CR2 & (0x1 << 1));
}

TEST_F(SPITests, TestEnableRXDMA)
{
    spi.enableRXDMA();
    ASSERT_EQ(0x1, spi.CR2 & 0x1);
}

TEST_F(SPITests, TestDisableRXDMA)
{
    spi.CR2 |= (0x1);
    spi.disableRXDMA();
    ASSERT_EQ(0x0, spi.CR2 & 0x1);
}

TEST_F(SPITests, TestGetTXBufferLevel)
{
    using Level = Bedrock::SPI::BufferLevel;
    ASSERT_EQ(Level::Empty, spi.getTXLevel());
    spi.SR = 0b11 << 11;
    ASSERT_EQ(Level::Full, spi.getTXLevel());
    spi.SR = 0b10 << 11;
    ASSERT_EQ(Level::Half, spi.getTXLevel());
}

TEST_F(SPITests, TestGetRXBufferLevel)
{
    using Level = Bedrock::SPI::BufferLevel;
    ASSERT_EQ(Level::Empty, spi.getRXLevel());
    spi.SR = 0b11 << 9;
    ASSERT_EQ(Level::Full, spi.getRXLevel());
    spi.SR = 0b01 << 9;
    ASSERT_EQ(Level::Quarter, spi.getRXLevel());
}

TEST_F(SPITests, TestBusy)
{
    ASSERT_FALSE(spi.isBusy());
    spi.SR = 0x1 << 7;
    ASSERT_TRUE(spi.isBusy());
}

TEST_F(SPITests, TestTXEmpty)
{
    ASSERT_TRUE(spi.txEmpty());
    spi.SR = 0;
    ASSERT_FALSE(spi.txEmpty());
}

TEST_F(SPITests, TestRXNotEmpty)
{
    ASSERT_FALSE(spi.rxNotEmpty());
    spi.SR = 0x1;
    ASSERT_TRUE(spi.rxNotEmpty());
}