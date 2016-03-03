//
//  DMATests.cpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-03-02.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "DMA.hpp"
#include "gtest/gtest.h"

class DMATests : public ::testing::Test
{
protected:
    using Controller = Bedrock::DMAController;
    using Channel = Bedrock::DMAChannel;
    
    static Controller controller;
    
    static Controller& controllerProv() {
        return controller;
    }
};

DMATests::Controller DMATests::controller{0};

TEST_F(DMATests, TestRegisterLayout)
{
    ASSERT_EQ(0x00, offsetof(Controller, ISR));
    ASSERT_EQ(0x04, offsetof(Controller, IFCR));
    
    //Test Channel configuration registers layout
    ASSERT_EQ(0x00, offsetof(Controller::ChannelConfig, CCR));
    ASSERT_EQ(0x04, offsetof(Controller::ChannelConfig, CNDTR));
    ASSERT_EQ(0x08, offsetof(Controller::ChannelConfig, CPAR));
    ASSERT_EQ(0x0C, offsetof(Controller::ChannelConfig, CMAR));
    ASSERT_EQ(20, sizeof(Controller::ChannelConfig));
    
    ASSERT_EQ(0x08, offsetof(Controller, channels));
    ASSERT_EQ(0x1C, offsetof(Controller, channels[1]));
    ASSERT_EQ(0x80, offsetof(Controller, channels[6]));
}

TEST_F(DMATests, TestChannelTransferErrorFlag)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.ISR = 0;
        ASSERT_FALSE(curr.transferError());
        controller.ISR |= 0x1 << (((i - 1) * 4) + 3); //set the trans err flag
        ASSERT_TRUE(curr.transferError());
    }
}

TEST_F(DMATests, TestChannelHalfTransferCompleteFlag)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.ISR = 0;
        ASSERT_FALSE(curr.halfTransferComplete());
        controller.ISR |= 0x1 << (((i - 1) * 4) + 2); //set the half trans flag
        ASSERT_TRUE(curr.halfTransferComplete());
    }
}

TEST_F(DMATests, TestChannelTransferCompleteFlag)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.ISR = 0;
        ASSERT_FALSE(curr.transferComplete());
        controller.ISR |= 0x1 << (((i - 1) * 4) + 1); //set the trans compl flag
        ASSERT_TRUE(curr.transferComplete());
    }
}

TEST_F(DMATests, TestChannelAckTransferError)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.IFCR = 0;
        curr.ackTransferError();
        ASSERT_EQ(0x1 << (((i - 1) * 4) + 3), controller.IFCR);
    }
}

TEST_F(DMATests, TestChannelAckHalfTransferComplete)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.IFCR = 0;
        curr.acklHalfTransferComplete();
        ASSERT_EQ(0x1 << (((i - 1) * 4) + 2), controller.IFCR);
    }
}

TEST_F(DMATests, TestChannelAckTransferComplete)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.IFCR = 0;
        curr.ackTransferComplete();
        ASSERT_EQ(0x1 << (((i - 1) * 4) + 1), controller.IFCR);
    }
}

TEST_F(DMATests, TestChannelSetPriority)
{
    using Prio = Channel::ChannelPriority;
    
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        curr.setChannelPriorityLevel(Prio::High);
        ASSERT_EQ(Prio::High,
                  static_cast<Prio>(controller.channels[i-1].CCR >> 12));
        curr.setChannelPriorityLevel(Prio::Low);
        ASSERT_EQ(Prio::Low,
                  static_cast<Prio>(controller.channels[i-1].CCR >> 12));
    }
}

TEST_F(DMATests, TestChannelGetPriority)
{
    using Prio = Channel::ChannelPriority;
    
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        curr.setChannelPriorityLevel(Prio::High);
        ASSERT_EQ(Prio::High, curr.getChannelPriority());
        curr.setChannelPriorityLevel(Prio::Low);
        ASSERT_EQ(Prio::Low, curr.getChannelPriority());
    }
}

TEST_F(DMATests, TestSetChannelMemorySize)
{
    using Size = Channel::TransferSize;
    
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        curr.setChannelMemorySize(Size::Bits_16);
        ASSERT_EQ(Size::Bits_16,
                  static_cast<Size>(controller.channels[i-1].CCR >> 10));
        curr.setChannelMemorySize(Size::Bits_8);
        ASSERT_EQ(Size::Bits_8,
                  static_cast<Size>(controller.channels[i-1].CCR >> 10));
    }
}

TEST_F(DMATests, TestSetChannelPeripheralSize)
{
    using Size = Channel::TransferSize;
    
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        curr.setChannelPeripheralSize(Size::Bits_16);
        ASSERT_EQ(Size::Bits_16,
                  static_cast<Size>(controller.channels[i-1].CCR >> 8));
        curr.setChannelPeripheralSize(Size::Bits_8);
        ASSERT_EQ(Size::Bits_8,
                  static_cast<Size>(controller.channels[i-1].CCR >> 8));
    }
}

TEST_F(DMATests, TestChannelEnableMemoryIncrement)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        curr.enableMemoryIncrementMode();
        ASSERT_EQ(0x1, controller.channels[i-1].CCR >> 7);
    }
}

TEST_F(DMATests, TestChannelDisableMemoryIncrement)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.channels[i-1].CCR = 0x1 << 7;
        curr.disableMemoryIncrementMode();
        ASSERT_EQ(0, controller.channels[i-1].CCR);
    }
}

TEST_F(DMATests, TestSetChannelEnablePeripheralIncrement)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        curr.enablePeripheralIncrementMode();
        ASSERT_EQ(0x1, controller.channels[i-1].CCR >> 6);
    }
}

TEST_F(DMATests, TestSetChannelDisablePeripheralIncrement)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.channels[i-1].CCR = 0x1 << 6;
        curr.disablePeripheralIncrementMode();
        ASSERT_EQ(0, controller.channels[i-1].CCR);
    }
}

TEST_F(DMATests, TestChannelEnableCircularMode)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.channels[i-1].CCR = 0;
        curr.enableCircularMode();
        ASSERT_EQ(0x1 << 5, controller.channels[i-1].CCR);
    }
}

TEST_F(DMATests, TestChannelDisableCircularMode)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.channels[i-1].CCR = 0x1 << 5;
        curr.disableCircularMode();
        ASSERT_EQ(0, controller.channels[i-1].CCR);
    }
}

TEST_F(DMATests, TestSetChannelTransferSource)
{
    using Source = Bedrock::DMAChannel::TransferSource;
    
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        curr.setTransferSource(Source::Memeory);
        ASSERT_EQ(0x1 << 4, controller.channels[i-1].CCR);
        
        curr.setTransferSource(Source::Peripheral);
        ASSERT_EQ(0, controller.channels[i-1].CCR);
    }
}

TEST_F(DMATests, TestGetChannelTransferSource)
{
    using Source = Bedrock::DMAChannel::TransferSource;
    
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        curr.setTransferSource(Source::Memeory);
        ASSERT_EQ(Source::Memeory, curr.getTransferSource());
        
        curr.setTransferSource(Source::Peripheral);
        ASSERT_EQ(Source::Peripheral, curr.getTransferSource());
    }
}

TEST_F(DMATests, TestChannelEnableHalfTransferInterrupt)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.channels[i-1].CCR = 0;
        curr.enableHalfTransferInterrupt();
        ASSERT_EQ(0x1 << 2, controller.channels[i-1].CCR);
    }
}

TEST_F(DMATests, TestChannelDisableHalfTransferInterrupt)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.channels[i-1].CCR = 0x1 << 2;
        curr.disableHalfTransferInterrupt();
        ASSERT_EQ(0, controller.channels[i-1].CCR);
    }
}

TEST_F(DMATests, TestChannelEnableTransferCompleteInterrupt)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.channels[i-1].CCR = 0;
        curr.enableTransferCompleteInterrupt();
        ASSERT_EQ(0x1 << 1, controller.channels[i-1].CCR);
    }
}

TEST_F(DMATests, TestChannelDisableTransferCompleteInterrupt)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.channels[i-1].CCR = 0x1 << 1;
        curr.disableTransferCompleteInterrupt();
        ASSERT_EQ(0, controller.channels[i-1].CCR);
    }
}

TEST_F(DMATests, TestChannelEnableTransferErrorInterrupt)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.channels[i-1].CCR = 0;
        curr.enableTransferErrorInterrupt();
        ASSERT_EQ(0x1 << 3, controller.channels[i-1].CCR);
    }
}

TEST_F(DMATests, TestChannelDisableTransferErrorInterrupt)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.channels[i-1].CCR = 0x1 << 3;
        curr.disableTransferErrorInterrupt();
        ASSERT_EQ(0, controller.channels[i-1].CCR);
    }
}

TEST_F(DMATests, TestChannelSetTransferCount)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        curr.setTransferCount(0xEA34);
        ASSERT_EQ(0xEA34, controller.channels[i-1].CNDTR);
    }
}

TEST_F(DMATests, TestChannelGetTransferCount)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.channels[i-1].CNDTR = 0xCF73;
        ASSERT_EQ(0xCF73, curr.getTransferCount());
    }
}

TEST_F(DMATests, TestChannelSetPeripheralRegisterAddress)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        uint32_t dummyAddr = 12;
        curr.setPeripheralRegisterAddress(&dummyAddr);
        ASSERT_EQ((uint32_t)reinterpret_cast<uintptr_t>(&dummyAddr),
                  controller.channels[i-1].CPAR);
    }
}

TEST_F(DMATests, TestChannelSetMemoryAddress)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        uint32_t dummyAddr = 44;
        curr.setMemoryAddress(&dummyAddr);
        ASSERT_EQ((uint32_t)reinterpret_cast<uintptr_t>(&dummyAddr),
                  controller.channels[i-1].CMAR);
    }
}


TEST_F(DMATests, TestChannelEnable)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.channels[i-1].CCR = 0;
        curr.enable();
        ASSERT_EQ(0x1, controller.channels[i-1].CCR);
    }
}

TEST_F(DMATests, TestChannelDisable)
{
    for (uint8_t i = 1; i < 8; i++)
    {
        Channel curr(controllerProv, i);
        
        controller.channels[i-1].CCR = 0x1;
        curr.disable();
        ASSERT_EQ(0, controller.channels[i-1].CCR);
    }
}

