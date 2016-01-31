//
//  bxCANTests.cpp
//  Bedrock Tests
//
//  Created by Eric Yanush on 2016-01-04.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "gtest/gtest.h"
#include "bxCAN.hpp"
#include "bxCANTestHelpers.hpp"

//Includes for async tests
#include <thread>
#include <future>
#include <chrono>

using namespace Bedrock;

/**
 Note: The tests which call enter/exit InitMode require they be run on a platform
        for which 32bit word accesses are atomic, or their behaviour will be undefined.
 */

class CANTests : public ::testing::Test {
    
protected:
    
    virtual void SetUp() {
        
        //Setup a frame with test data to be used for tests.
        testMessage.dataLen = 8;
        testMessage.id = 0x12345678;
        testMessage.format = CANMessage::Format::Extended;
        testMessage.type = CANMessage::Type::Data;
        testMessage.data[0] = 5;
        testMessage.data[1] = 10;
        testMessage.data[2] = 55;
        testMessage.data[3] = 201;
        testMessage.data[4] = 111;
        testMessage.data[5] = 42;
        testMessage.data[6] = 21;
        testMessage.data[7] = 69;
    }
    
    virtual void TearDown() {
    }
    
    bool canIsAllZeros() {
        return memcmp(&untouched, &can, sizeof(bxCAN::CANPort));
    }
    
    bxCAN::CANPort can{0};
    bxCAN::CANPort untouched{0};
    CANMessage testMessage;
};

/** Ensure layout of bxCAN object adheres with Register Map
 * as defined in the STM32F3 Programming Manual
 */
TEST_F(CANTests, TestCANRegLayout) {
    
    //Ensure base peripheral register offsets are correct
    ASSERT_EQ(0x000, offsetof(bxCAN::CANPort, MCR));
    ASSERT_EQ(0x004, offsetof(bxCAN::CANPort, MSR));
    ASSERT_EQ(0x008, offsetof(bxCAN::CANPort, TSR));
    ASSERT_EQ(0x00C, offsetof(bxCAN::CANPort, RFR[0]));
    ASSERT_EQ(0x010, offsetof(bxCAN::CANPort, RFR[1]));
    ASSERT_EQ(0x014, offsetof(bxCAN::CANPort, IER));
    ASSERT_EQ(0x018, offsetof(bxCAN::CANPort, ESR));
    ASSERT_EQ(0x01C, offsetof(bxCAN::CANPort, BTR));
    
    //Ensure Transmit Mailbox Register offsets are correct
    ASSERT_EQ(0x180, offsetof(bxCAN::CANPort, txMailbox[0]));
    ASSERT_EQ(0x190, offsetof(bxCAN::CANPort, txMailbox[1]));
    ASSERT_EQ(0x1A0, offsetof(bxCAN::CANPort, txMailbox[2]));
    //Test the Register layout of the TxMailbox
    ASSERT_EQ(0x0, offsetof(bxCAN::TxMailbox, TIR));
    ASSERT_EQ(0x4, offsetof(bxCAN::TxMailbox, TDTR));
    ASSERT_EQ(0x8, offsetof(bxCAN::TxMailbox, TDLR));
    ASSERT_EQ(0xC, offsetof(bxCAN::TxMailbox, TDHR));
    
    //Ensure RX FIFO Queue Register offsets are correct
    ASSERT_EQ(0x1B0, offsetof(bxCAN::CANPort, rxFIFO[0]));
    ASSERT_EQ(0x1C0, offsetof(bxCAN::CANPort, rxFIFO[1]));
    //Test the Register layout of the RX Queues
    ASSERT_EQ(0x0, offsetof(bxCAN::RxFIFO, RIR));
    ASSERT_EQ(0x4, offsetof(bxCAN::RxFIFO, RDTR));
    ASSERT_EQ(0x8, offsetof(bxCAN::RxFIFO, RDLR));
    ASSERT_EQ(0xC, offsetof(bxCAN::RxFIFO, RDHR));
    
    //Ensure Filter Bank Register offsets are correct
    ASSERT_EQ(0x200, offsetof(bxCAN::CANPort, filters));
    //Test the Register layout of the Filters
    ASSERT_EQ(0x00, offsetof(bxCAN::Filters, FMR));
    ASSERT_EQ(0x04, offsetof(bxCAN::Filters, FM1R));
    ASSERT_EQ(0x0C, offsetof(bxCAN::Filters, FS1R));
    ASSERT_EQ(0x14, offsetof(bxCAN::Filters, FFA1R));
    ASSERT_EQ(0x1C, offsetof(bxCAN::Filters, FA1R));
    ASSERT_EQ(0x40, offsetof(bxCAN::Filters, filterBank[0]));
    ASSERT_EQ(0x240, offsetof(bxCAN::CANPort, filters.filterBank));
    ASSERT_EQ(0x48, offsetof(bxCAN::Filters, filterBank[1]));
    //Test the register layout of the filter banks
    ASSERT_EQ(0x0, offsetof(bxCAN::Filters::FilterBank, FR1));
    ASSERT_EQ(0x4, offsetof(bxCAN::Filters::FilterBank, FR2));
}

TEST_F(CANTests, TestEnterInitMode) {
    using namespace std::literals;
    
    auto asyncHW = [this]() {
        //Check for Init mode request
        while ((can.MCR & 0x1) != 0x1) {
            std::this_thread::sleep_for(100us);
        }
        can.MSR |= 0x1; // Set the Init mode acknowledge bit
    };
    std::thread async(asyncHW);
    
    can.enterInitMode();
    async.join(); // Wait for async ops to be done
    
    ASSERT_TRUE((can.MCR & 0x1) == 0x1); // Ensure the INRQ bit was set
    ASSERT_TRUE((can.MSR & 0x1) == 0x1); // Ensure the INAK bit was set
}

TEST_F(CANTests, TestExitInitMode) {
    using namespace std::literals; //for chrono literals
    
    can.MCR |= 0x1; // Set the INRQ bit
    can.MSR |= 0x1; // Set the INAK bit
    
    auto asyncHW = [this]() {
        //wait for for INRQ clear
        while((can.MCR & 0x1) == 0x1) {
            std::this_thread::sleep_for(100us);
        }
        can.MSR &= ~(0x1); //Clear the init mode ack bit
    };
    std::thread async(asyncHW);
    
    can.exitInitMode();
    async.join(); // Wait for async ops to be done
    
    ASSERT_FALSE((can.MCR & 0x1) == 0x1); // Ensure the INRQ bit was cleared
    ASSERT_FALSE((can.MSR & 0x1) == 0x1); // Ensure the INAK bit was cleared
}

TEST_F(CANTests, TestInit_defaults) {
    using namespace std::literals;
    using BusFrequency = bxCAN::CANPort::BusFrequency;
    using Mode = bxCAN::CANPort::Mode;
    
    auto asyncHW = simulateCAN_enter_exit_init(can);
    
    can.init();
    asyncHW.join();
    
    ASSERT_FALSE((can.MSR & 0x1) == 0x1); //Ensure we are not still in init mode
    ASSERT_FALSE((can.MCR & 0x1) == 0x1);
    ASSERT_EQ(BusFrequency::KHz_125, static_cast<BusFrequency>(can.BTR & 0x1FF)); //Ensure we have set the prescaler value properly
    ASSERT_EQ(8, ((can.BTR & 0x000F0000) >> 16) + 1); //Ensure we have setup BS1 to have 8tq
    ASSERT_EQ(3, ((can.BTR & 0x00700000) >> 20) + 1); //Ensure we have setup BS2 to have 3tq
    ASSERT_EQ(1, ((can.BTR & 0x03000000) >> 24) + 1); //Ensure we have setup the Sync Jump to be 1tq
    ASSERT_EQ(Mode::Normal, static_cast<Mode>((can.BTR & 0xC0000000) >> 30)); //Ensure we have setup normal mode
    ASSERT_FALSE((can.MCR & 0x10) == 0x10); //Ensure we have Automatic Retransmission enabled (NART bit not set)
    ASSERT_TRUE((can.MCR & 0x40) == 0x40); // Ensure we have Automatic Bus-off management enabled (ABOM bit set)
    ASSERT_FALSE((can.MCR & 0x80) == 0x80); //Ensure we have Time Triggered Mode off (TTCM bit not set)
}

TEST_F(CANTests, TestInit) {
    using namespace std::literals;
    using BusFrequency = bxCAN::CANPort::BusFrequency;
    using Mode = bxCAN::CANPort::Mode;
    
    can.MCR |= 0x2; //Set the sleep bit, as this is done automatically by hw on reset
    
    auto asyncHW = simulateCAN_enter_exit_init(can);
    
    can.init(BusFrequency::MHz_1, Mode::SelfTest);
    asyncHW.join();
    
    ASSERT_FALSE((can.MSR & 0x1) == 0x1); //Ensure we are not still in init mode
    ASSERT_FALSE((can.MCR & 0x1) == 0x1);
    ASSERT_EQ(BusFrequency::MHz_1, static_cast<BusFrequency>(can.BTR & 0x1FF)); //Ensure we have set the prescaler value properly
    ASSERT_EQ(8, ((can.BTR & 0x000F0000) >> 16) + 1); //Ensure we have setup BS1 to have 8tq
    ASSERT_EQ(3, ((can.BTR & 0x00700000) >> 20) + 1); //Ensure we have setup BS2 to have 3tq
    ASSERT_EQ(1, ((can.BTR & 0x03000000) >> 24) + 1); //Ensure we have setup the Sync Jump to be 1tq
    ASSERT_EQ(Mode::SelfTest, static_cast<Mode>((can.BTR & 0xC0000000) >> 30)); //Ensure we have setup self-test mode
    ASSERT_FALSE((can.MCR & 0x10) == 0x10); //Ensure we have Automatic Retransmission enabled (NART bit not set)
    ASSERT_TRUE((can.MCR & 0x40) == 0x40); // Ensure we have Automatic Bus-off management enabled (ABOM bit set)
    ASSERT_FALSE((can.MCR & 0x80) == 0x80); //Ensure we have Time Triggered Mode disabled (TTCM bit not set)
    ASSERT_FALSE((can.MCR & 0x2) == 0x2); //Ensure we have cleared the sleep mode bit
}

TEST_F(CANTests, TestSetMode_unsafe) {
    using BusFrequency = bxCAN::CANPort::BusFrequency;
    using Mode = bxCAN::CANPort::Mode;
    
    can.setMode_unsafe(Mode::SelfTest);
    ASSERT_EQ(Mode::SelfTest, static_cast<Mode>((can.BTR & 0xC0000000) >> 30));
    ASSERT_FALSE((can.MCR & 0x1) == 0x1); //Ensure we didn't try to enter into init mode
}

TEST_F(CANTests, TestSetMode) {
    using namespace std::literals;
    using BusFrequency = bxCAN::CANPort::BusFrequency;
    using Mode = bxCAN::CANPort::Mode;
    
    auto asyncHW = simulateCAN_enter_exit_init(can);
    
    can.setMode(Mode::Silent);
    asyncHW.join();
    
    ASSERT_EQ(Mode::Silent, static_cast<Mode>((can.BTR & 0xC0000000) >> 30));
    ASSERT_FALSE((can.MSR & 0x1) == 0x1); //Ensure we are not still in init mode
    ASSERT_FALSE((can.MCR & 0x1) == 0x1);
}

TEST_F(CANTests, TestGetMode) {
    using Mode = bxCAN::CANPort::Mode;
    can.setMode_unsafe(Mode::Loopback);
    
    ASSERT_EQ(Mode::Loopback, can.getMode());
}

TEST_F(CANTests, TestSetFrequency_unsafe) {
    using BusFrequency = bxCAN::CANPort::BusFrequency;
    using Mode = bxCAN::CANPort::Mode;
    
    can.setFrequency_unsafe(BusFrequency::KHz_250);
    
    ASSERT_EQ(BusFrequency::KHz_250, static_cast<BusFrequency>(can.BTR & 0x1FF));
    ASSERT_FALSE((can.MCR & 0x1) == 0x1); //Ensure we didn't try to enter into init mode
}

TEST_F(CANTests, TestSetFrequency) {
    using namespace std::literals;
    using BusFrequency = bxCAN::CANPort::BusFrequency;
    using Mode = bxCAN::CANPort::Mode;
  
    auto asyncHW = simulateCAN_enter_exit_init(can);
    
    can.setFrequency(BusFrequency::KHz_20);
    asyncHW.join();
    
    ASSERT_FALSE((can.MSR & 0x1) == 0x1); //Ensure we aren't still in init mode
    ASSERT_FALSE((can.MCR & 0x1) == 0x1);
    ASSERT_EQ(BusFrequency::KHz_20, static_cast<BusFrequency>(can.BTR & 0x1FF));
}

TEST_F(CANTests, TestGetFrequency) {
    using BusFrequency = bxCAN::CANPort::BusFrequency;
    
    can.setFrequency_unsafe(BusFrequency::KHz_50);
    
    ASSERT_EQ(BusFrequency::KHz_50, can.getFrequency());
}

TEST_F(CANTests, TestAvailableTXMailbox) {
    //All registers should be 0 on setup, should be no mailbox available
    ASSERT_FALSE(can.availableTXMailbox());
    can.TSR |= (0x1 << 28); //Set the MB2 empty bit
    ASSERT_TRUE(can.availableTXMailbox());
    can.TSR = 0;
    can.TSR |= (0x1 << 27);
    ASSERT_TRUE(can.availableTXMailbox());
    can.TSR = 0;
    can.TSR |= (0x1 << 26);
    ASSERT_TRUE(can.availableTXMailbox());
}

TEST_F(CANTests, TestMessagesWaitingInFIFO) {
    //All registers inited to zero, should be no messages waiting
    ASSERT_EQ(0, can.rxMessagesWaitingInFIFO(0));
    ASSERT_EQ(0, can.rxMessagesWaitingInFIFO(1));
    
    
    for (uint8_t i = 0; i < 2; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            can.RFR[i] &= ~(0x3); //clear previous vals
            can.RFR[i] |= j;
            ASSERT_EQ(j, can.rxMessagesWaitingInFIFO(i));
        }
    }
}

TEST_F(CANTests, TestTransmitMessage_usesFirstAvailableMailbox) {
    //Set no mailboxes available
    can.TSR = 0;
    CANMessage testMessage;
    testMessage.dataLen = 4;
    testMessage.id = 0x12345678;
    testMessage.format = CANMessage::Format::Extended;
    testMessage.type = CANMessage::Type::Data;
    testMessage.data[0] = 5;
    testMessage.data[1] = 10;
    testMessage.data[2] = 55;
    testMessage.data[3] = 201;
    
    ASSERT_EQ(-1, can.transmitMessage(testMessage));
    ASSERT_EQ(0, memcmp(&can, &untouched, sizeof(bxCAN::CANPort))); //Ensure we didn't try to write anything out
    
    can.TSR |= 0x1 << 26; //Set mb 0 empty
    ASSERT_EQ(0, can.transmitMessage(testMessage));
    can.TSR = 0 | (0x1 << 27); // Set mb 1 empty
    ASSERT_EQ(1, can.transmitMessage(testMessage));
    can.TSR = 0 | (0x1 << 28); // Set mb 2 empty
    ASSERT_EQ(2, can.transmitMessage(testMessage));
    can.TSR = 0 | (0x1 << 27) | (0x1 << 28);
    ASSERT_EQ(1, can.transmitMessage(testMessage));
}

TEST_F(CANTests, TestTransmitMessage) {
    //Ensure we are using a data frame for this test
    testMessage.type = CANMessage::Type::Data;
    
    //set mb 1 empty
    can.TSR |= 0x1 << 27;
    //Send the message
    ASSERT_EQ(1, can.transmitMessage(testMessage));
    ASSERT_EQ(testMessage.id, can.txMailbox[1].TIR >> 3);
    ASSERT_TRUE((can.txMailbox[1].TIR & 0x1) == 0x1); //Ensure The TX was requested
    ASSERT_FALSE((can.txMailbox[1].TIR & 0x2) == 0x2); //Ensure the RTR bit was not set
    ASSERT_TRUE((can.txMailbox[1].TIR & 0x4) == 0x4); //Ensure the IDE (extended id) bit was set
    ASSERT_EQ(testMessage.dataLen, can.txMailbox[1].TDTR & 0xF);
    ASSERT_EQ(testMessage.data[0], can.txMailbox[1].TDLR & 0xFF);
    ASSERT_EQ(testMessage.data[1], (can.txMailbox[1].TDLR >> 8) & 0xFF);
    ASSERT_EQ(testMessage.data[2], (can.txMailbox[1].TDLR >> 16) & 0xFF);
    ASSERT_EQ(testMessage.data[3], (can.txMailbox[1].TDLR >> 24) & 0xFF);
    ASSERT_EQ(testMessage.data[4], can.txMailbox[1].TDHR & 0xFF);
    ASSERT_EQ(testMessage.data[5], (can.txMailbox[1].TDHR >> 8) & 0xFF);
    ASSERT_EQ(testMessage.data[6], (can.txMailbox[1].TDHR >> 16) & 0xFF);
    ASSERT_EQ(testMessage.data[7], (can.txMailbox[1].TDHR >> 24) & 0xFF);
    
    //Set mb 0 empty
    can.TSR = 0x1 << 26;
    testMessage.type = CANMessage::Type::Remote; // Try sending a remote frame
    testMessage.format = CANMessage::Format::Standard; // with a standard ID
    testMessage.dataLen = 3; //With only 3 data bytes
    ASSERT_EQ(0, can.transmitMessage(testMessage));
    ASSERT_EQ(testMessage.id & 0x7FF, can.txMailbox[0].TIR >> 21); //Check that the id was set correctly
    ASSERT_TRUE((can.txMailbox[0].TIR & 0x1) == 0x1); //Ensure The TX was requested
    ASSERT_TRUE((can.txMailbox[0].TIR & 0x2) == 0x2); //Ensure the RTR bit was set
    ASSERT_FALSE((can.txMailbox[0].TIR & 0x4) == 0x4); //Ensure the IDE (extended id) bit was not set
    ASSERT_EQ(3, can.txMailbox[0].TDTR & 0xF); //Ensure the data length was set correctly
}

TEST_F(CANTests, TestRecieveMessageFromFIFO) {
    //Copy Test Message into RXFIFO1
    can.rxFIFO[1].RIR = (testMessage.id << 3) | (1 << 2); //Using an extended ID
    can.rxFIFO[1].RDTR = testMessage.dataLen;
    uint32_t* msgData;
    msgData = reinterpret_cast<uint32_t*>(testMessage.data);
    can.rxFIFO[1].RDLR = msgData[0];
    can.rxFIFO[1].RDHR = msgData[1];
    
    CANMessage retMessage;
    can.recieveMessageFromFIFO(1, retMessage);
    ASSERT_TRUE((can.RFR[1] & 0x20) == 0x20); //Ensure the fifo was released
    ASSERT_EQ(testMessage.id, retMessage.id);
    ASSERT_EQ(testMessage.format, retMessage.format);
    ASSERT_EQ(testMessage.type, retMessage.type);
    ASSERT_EQ(testMessage.dataLen, retMessage.dataLen);
    for (uint8_t i = 0; i < 8; i++) {
        ASSERT_EQ(testMessage.data[i], retMessage.data[i]);
    }
}

TEST_F(CANTests, Test_tx0Complete) {
    //All Zeros, should be false
    ASSERT_FALSE(can.tx0Complete());
    
    can.TSR |= 1; //Set the completion flag
    ASSERT_TRUE(can.tx0Complete());
}

TEST_F(CANTests, Test_ackTX0Complete) {
    //Ensure the completion bit is zero; it is cleared by writing a 1
    // In reality, it would already by 1, but we can't test for something that doesn't change
    can.TSR = 0;
    can.ackTX0Complete();
    
    ASSERT_TRUE((can.TSR & 0x1) == 0x1);
}

TEST_F(CANTests, Test_tx1Complete) {
    //All Zeros, should be false
    ASSERT_FALSE(can.tx1Complete());
    
    can.TSR |= 1 << 8; //Set the completion flag
    ASSERT_TRUE(can.tx1Complete());
}

TEST_F(CANTests, Test_ackTX1Complete) {
    //Ensure the completion bit is zero; it is cleared by writing a 1
    // In reality, it would already by 1, but we can't test for something that doesn't change
    can.TSR = 0;
    can.ackTX1Complete();
    
    ASSERT_TRUE((can.TSR & (1 << 8)) == (1 << 8));
}

TEST_F(CANTests, Test_tx2Complete) {
    //All Zeros, should be false
    ASSERT_FALSE(can.tx2Complete());
    
    can.TSR |= 1 << 16; //Set the completion flag
    ASSERT_TRUE(can.tx2Complete());
}

TEST_F(CANTests, Test_ackTX2Complete) {
    //Ensure the completion bit is zero; it is cleared by writing a 1
    // In reality, it would already by 1, but we can't test for something that doesn't change
    can.TSR = 0;
    can.ackTX2Complete();
    
    ASSERT_TRUE((can.TSR & (1 << 16)) == (1 << 16));
}

TEST_F(CANTests, Test_SleepInterruptOnOff) {
    const uint32_t bitMask = 1 << 17;
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Ensure the interrupt is off
    can.enableSleepInterrupt();
    ASSERT_TRUE((can.IER & bitMask) == bitMask); //Should be on now
    can.disableSleepInterrupt();
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Should be off again
}

TEST_F(CANTests, Test_WakeupInterruptOnOff) {
    const uint32_t bitMask = 1 << 16;
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Ensure the interrupt is off
    can.enableWakupInterrupt();
    ASSERT_TRUE((can.IER & bitMask) == bitMask); //Should be on now
    can.disableWakupInterrupt();
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Should be off again
}

TEST_F(CANTests, Test_ErrorInterruptOnOff) {
    const uint32_t bitMask = 1 << 15;
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Ensure the interrupt is off
    can.enableErrorInterrupt();
    ASSERT_TRUE((can.IER & bitMask) == bitMask); //Should be on now
    can.disableErrorInterrupt();
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Should be off again
}

TEST_F(CANTests, Test_LastErrorCodeInterruptOnOff) {
    const uint32_t bitMask = 1 << 11;
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Ensure the interrupt is off
    can.enableLastErrorCodeInterrupt();
    ASSERT_TRUE((can.IER & bitMask) == bitMask); //Should be on now
    can.disableLastErrorCodeInterrupt();
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Should be off again
}

TEST_F(CANTests, Test_BusOffInterruptOnOff) {
    const uint32_t bitMask = 1 << 10;
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Ensure the interrupt is off
    can.enableBusOffInterrupt();
    ASSERT_TRUE((can.IER & bitMask) == bitMask); //Should be on now
    can.disableBusOffInterrupt();
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Should be off again
}

TEST_F(CANTests, Test_ErrorPassiveInterruptOnOff) {
    const uint32_t bitMask = 1 << 9;
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Ensure the interrupt is off
    can.enableErrorPassiveInterrupt();
    ASSERT_TRUE((can.IER & bitMask) == bitMask); //Should be on now
    can.disableErrorPassiveInterrupt();
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Should be off again
}

TEST_F(CANTests, Test_ErrorWarningInterruptOnOff) {
    const uint32_t bitMask = 1 << 8;
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Ensure the interrupt is off
    can.enableErrorWarnInterrupt();
    ASSERT_TRUE((can.IER & bitMask) == bitMask); //Should be on now
    can.disableErrorWarnInterrupt();
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Should be off again
}

TEST_F(CANTests, Test_FIFO1OverrunInterruptOnOff) {
    const uint32_t bitMask = 1 << 6;
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Ensure the interrupt is off
    can.enableFIFO1OverrunInterrupt();
    ASSERT_TRUE((can.IER & bitMask) == bitMask); //Should be on now
    can.disableFIFO1OverrunInterrupt();
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Should be off again
}

TEST_F(CANTests, Test_FIFO1FullInterruptOnOff) {
    const uint32_t bitMask = 1 << 5;
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Ensure the interrupt is off
    can.enableFIFO1FullInterrupt();
    ASSERT_TRUE((can.IER & bitMask) == bitMask); //Should be on now
    can.disableFIFO1FullInterrupt();
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Should be off again
}

TEST_F(CANTests, Test_FIFO1MsgPendingInterruptOnOff) {
    const uint32_t bitMask = 1 << 4;
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Ensure the interrupt is off
    can.enableFIFO1MessagePendingInterrupt();
    ASSERT_TRUE((can.IER & bitMask) == bitMask); //Should be on now
    can.disableFIFO1MessagePendingInterrupt();
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Should be off again
}

TEST_F(CANTests, Test_FIFO0OverrunInterruptOnOff) {
    const uint32_t bitMask = 1 << 3;
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Ensure the interrupt is off
    can.enableFIFO0OverrunInterrupt();
    ASSERT_TRUE((can.IER & bitMask) == bitMask); //Should be on now
    can.disableFIFO0OverrunInterrupt();
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Should be off again
}

TEST_F(CANTests, Test_FIFO0FullInterruptOnOff) {
    const uint32_t bitMask = 1 << 2;
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Ensure the interrupt is off
    can.enableFIFO0FullInterrupt();
    ASSERT_TRUE((can.IER & bitMask) == bitMask); //Should be on now
    can.disableFIFO0FullInterrupt();
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Should be off again
}

TEST_F(CANTests, Test_FIFO0MsgPendingInterruptOnOff) {
    const uint32_t bitMask = 1 << 1;
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Ensure the interrupt is off
    can.enableFIFO0MessagePendingInterrupt();
    ASSERT_TRUE((can.IER & bitMask) == bitMask); //Should be on now
    can.disableFIFO0MessagePendingInterrupt();
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Should be off again
}

TEST_F(CANTests, Test_TXMailboxEmptyInterruptOnOff) {
    const uint32_t bitMask = 1;
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Ensure the interrupt is off
    can.enableTXMailboxEmptyInterrupt();
    ASSERT_TRUE((can.IER & bitMask) == bitMask); //Should be on now
    can.disableTXMailboxEmptyInterrupt();
    ASSERT_FALSE((can.IER & bitMask) == bitMask); //Should be off again
}

TEST_F(CANTests, TestGetRXErrorCount) {
    ASSERT_EQ(0, can.getCurrentRXErrorCount());
    can.ESR = (101 << 24);
    ASSERT_EQ(101, can.getCurrentRXErrorCount());
}

TEST_F(CANTests, TestGetTXErrorCount) {
    ASSERT_EQ(0, can.getCurrentTXErrorCount());
    can.ESR = (88 << 16);
    ASSERT_EQ(88, can.getCurrentTXErrorCount());
}

TEST_F(CANTests, TestGetLastErrorCode) {
    using ErrorCode = bxCAN::CANPort::ErrorCode;
    
    ASSERT_EQ(ErrorCode::NoError, can.getLastErrorCode());
    can.ESR = static_cast<uint8_t>(ErrorCode::BitDominantError) << 4;
    ASSERT_EQ(ErrorCode::BitDominantError, can.getLastErrorCode());
}

TEST_F(CANTests, TestIsBusOffState) {
    ASSERT_FALSE(can.isInBusOffState());
    can.ESR = (1 << 2);
    ASSERT_TRUE(can.isInBusOffState());
}

TEST_F(CANTests, TestIsInErrorWarningState) {
    ASSERT_FALSE(can.isInErrorWarningState());
    can.ESR = 1;
    ASSERT_TRUE(can.isInErrorWarningState());
}

TEST_F(CANTests, TestIsInErrorPassiveState) {
    ASSERT_FALSE(can.isInErrorPassiveState());
    can.ESR = (1 << 1);
    ASSERT_TRUE(can.isInErrorPassiveState());
}

TEST_F(CANTests, TestFilterStructs) {
    //Test the single id struct
    ASSERT_EQ(4, sizeof(bxCAN::Filters::SingleIDFilter_t));
    bxCAN::Filters::SingleIDFilter_t singleId{0};
    uint32_t& rawSingleId = *reinterpret_cast<uint32_t*>(&singleId);
    singleId.ide = 1;
    ASSERT_TRUE((rawSingleId & 0x4) == 0x4);
    singleId.ide = 0;
    singleId.rtr = 1;
    ASSERT_TRUE((rawSingleId & 0x2) == 0x2);
    singleId.rtr = 0;
    singleId.id = 0x12345678;
    ASSERT_TRUE((rawSingleId >> 3) == 0x12345678);
    
    //Test the single mask struct
    ASSERT_EQ(4, sizeof(bxCAN::Filters::SingleMaskFilter_t));
    bxCAN::Filters::SingleMaskFilter_t singleMask{0};
    uint32_t& rawSingleMask = *reinterpret_cast<uint32_t*>(&singleMask);
    singleMask.id_mask = 0x12345678;
    ASSERT_EQ(0x12345678, rawSingleMask >> 3);
    singleMask.id_mask = 0;
    singleMask.ide_mask = 1;
    ASSERT_EQ(0x4, rawSingleMask & 0x4);
    singleMask.ide_mask = 0;
    singleMask.rtr_mask = 1;
    ASSERT_EQ(0x2, rawSingleMask & 0x2);
    
    //Test the Dual ID struct
    ASSERT_EQ(2, sizeof(bxCAN::Filters::DualIDFilter_t));
    bxCAN::Filters::DualIDFilter_t dualId{0};
    uint16_t& rawDualId = *reinterpret_cast<uint16_t*>(&dualId);
    dualId.id = 0x7A8;
    ASSERT_EQ(0x7A8, rawDualId >> 5);
    dualId.id = 0;
    dualId.ide = 1;
    ASSERT_EQ(0x8, rawDualId & 0x8);
    dualId.ide = 0;
    dualId.rtr = 1;
    ASSERT_EQ(0x10, rawDualId & 0x10);
    dualId.rtr = 0;
    dualId.extID = 0x5;
    ASSERT_EQ(0x5, rawDualId);
    
    //Test the Dual Mask struct
    ASSERT_EQ(2, sizeof(bxCAN::Filters::DualMaskFilter_t));
    bxCAN::Filters::DualMaskFilter_t dualMask{0};
    uint16_t& rawDualMask = *reinterpret_cast<uint16_t*>(&dualMask);
    dualMask.id_mask = 0x7A8;
    ASSERT_EQ(0x7A8, rawDualMask >> 5);
    dualMask.id_mask = 0;
    dualMask.ide_mask = 1;
    ASSERT_EQ(0x8, rawDualMask & 0x8);
    dualMask.ide_mask = 0;
    dualMask.rtr_mask = 1;
    ASSERT_EQ(0x10, rawDualMask & 0x10);
    dualMask.rtr_mask = 0;
    dualMask.extID_mask = 0x5;
    ASSERT_EQ(0x5, rawDualMask);
}

TEST_F(CANTests, TestEnableFilter) {
    for (uint8_t i = 0; i < 14; i++) {
        can.filters.enableFilterBank(i);
        ASSERT_EQ(1 << i, can.filters.FA1R);
        can.filters.FA1R = 0;
    }
}

TEST_F(CANTests, TestDisableFilter) {
    for (uint8_t i = 0; i < 14; i++) {
        can.filters.FA1R |= (1 << i);
        can.filters.disableFilterBank(i);
        ASSERT_EQ(0, can.filters.FA1R);
    }
}

TEST_F(CANTests, TestAssignFilterToFIFO) {
    //note: we cannot test for FINIT bit being set (required by hw)!
    for (uint8_t i = 0; i < 14; i++) {
        can.filters.assignFilterToFIFO(i, 1);
        ASSERT_EQ((1 << i), can.filters.FFA1R & (1 << i));
        can.filters.assignFilterToFIFO(i, 0);
        ASSERT_EQ(0, can.filters.FFA1R);
    }
}

TEST_F(CANTests, TestConfigureSingleMaskFilter) {
    using Filters = bxCAN::Filters;
    Filters::SingleIDFilter_t id{0};
    Filters::SingleMaskFilter_t mask{0};
    //Set some values
    id.id = 0x12345678;
    id.ide = 1;
    id.rtr = 0;
    mask.id_mask = 0x12348765;
    mask.ide_mask = 1;
    id.rtr = 1;
    
    uint32_t& rawId = *reinterpret_cast<uint32_t*>(&id);
    uint32_t& rawMask = *reinterpret_cast<uint32_t*>(&mask);
    
    //Test all the filter banks
    for (uint8_t i = 0; i < 14; i++) {
        can.filters.configureSingleMaskFilter(i, id, mask);
        ASSERT_EQ(rawId, can.filters.filterBank[i].FR1);
        ASSERT_EQ(rawMask, can.filters.filterBank[i].FR2);
        ASSERT_TRUE((can.filters.FM1R & (1 << i)) == 0); // Ensure we are in mask mode
        ASSERT_TRUE((can.filters.FS1R & (1 << i)) == (1 << i)); //Ensure we are in Single scale mode
        ASSERT_EQ(0, can.filters.FMR); //Ensure the FINIT bit is cleared
    }
}

TEST_F(CANTests, TestConfigureSingleIDListFilter) {
    using Filters = bxCAN::Filters;
    Filters::SingleIDFilter_t id{0};
    Filters::SingleIDFilter_t id2{0};
    //Set some values
    id.id = 0x12345678;
    id.ide = 1;
    id.rtr = 0;
    id2.id = 0x12348765;
    id2.ide = 1;
    id.rtr = 1;
    
    uint32_t& rawId = *reinterpret_cast<uint32_t*>(&id);
    uint32_t& rawId2 = *reinterpret_cast<uint32_t*>(&id2);
    
    //Test all the filter banks
    for (uint8_t i = 0; i < 14; i++) {
        can.filters.configureSingleIDListFilter(i, id, id2);
        ASSERT_EQ(rawId, can.filters.filterBank[i].FR1);
        ASSERT_EQ(rawId2, can.filters.filterBank[i].FR2);
        ASSERT_TRUE((can.filters.FM1R & (1 << i)) == (1 << i)); // Ensure we are in ID mode
        ASSERT_TRUE((can.filters.FS1R & (1 << i)) == (1 << i)); //Ensure we are in Single scale mode
        ASSERT_EQ(0, can.filters.FMR); //Ensure the FINIT bit is cleared
    }
}

TEST_F(CANTests, TestConfigureDualMaskFilter) {
    using Filters = bxCAN::Filters;
    Filters::DualIDFilter_t id1{0};
    Filters::DualMaskFilter_t mask1{0};
    Filters::DualIDFilter_t id2{0};
    Filters::DualMaskFilter_t mask2{0};
    
    //Set some values
    id1.id = 0x7AF;
    id1.ide = 1;
    id1.rtr = 0;
    mask1.id_mask = 0x111;
    mask1.ide_mask = 1;
    mask1.rtr_mask = 1;
    id2.id = 0x500;
    id2.ide = 1;
    id2.rtr = 1;
    mask2.id_mask = 0x555;
    mask2.ide_mask = 0;
    mask2.rtr_mask = 0;
    
    
    uint16_t& rawId1 = *reinterpret_cast<uint16_t*>(&id1);
    uint16_t& rawMask1 = *reinterpret_cast<uint16_t*>(&mask1);
    uint16_t& rawId2 = *reinterpret_cast<uint16_t*>(&id2);
    uint16_t& rawMask2 = *reinterpret_cast<uint16_t*>(&mask2);
    
    //Test all the filter banks
    for (uint8_t i = 0; i < 14; i++) {
        can.filters.configureDualMaskFilter(i, id1, mask1, id2, mask2);
        ASSERT_EQ((rawId1 << 16) | rawId2, can.filters.filterBank[i].FR1);
        ASSERT_EQ((rawMask1 << 16) | rawMask2, can.filters.filterBank[i].FR2);
        ASSERT_TRUE((can.filters.FM1R & (1 << i)) == 0); // Ensure we are in mask mode
        ASSERT_TRUE((can.filters.FS1R & (1 << i)) == 0); //Ensure we are in Dual scale
        ASSERT_EQ(0, can.filters.FMR); //Ensure the FINIT bit is cleared
    }
}

TEST_F(CANTests, TestConfigureDualIDListFilter) {
    using Filters = bxCAN::Filters;
    Filters::DualIDFilter_t id1{0};
    Filters::DualIDFilter_t id2{0};
    Filters::DualIDFilter_t id3{0};
    Filters::DualIDFilter_t id4{0};
    
    //Set some values
    id1.id = 0x7AF;
    id1.ide = 1;
    id1.rtr = 0;
    id2.id = 0x111;
    id2.ide = 1;
    id2.rtr = 1;
    id2.extID = 0x2;
    id3.id = 0x500;
    id3.ide = 1;
    id3.rtr = 1;
    id4.id = 0x555;
    id4.ide = 0;
    id4.rtr = 0;
    
    
    uint16_t& rawId1 = *reinterpret_cast<uint16_t*>(&id1);
    uint16_t& rawId2 = *reinterpret_cast<uint16_t*>(&id2);
    uint16_t& rawId3 = *reinterpret_cast<uint16_t*>(&id3);
    uint16_t& rawId4 = *reinterpret_cast<uint16_t*>(&id4);
    
    //Test all the filter banks
    for (uint8_t i = 0; i < 14; i++) {
        can.filters.configureDualIDListFilter(i, id1, id2, id3, id4);
        ASSERT_EQ((rawId1 << 16) | rawId2, can.filters.filterBank[i].FR1);
        ASSERT_EQ((rawId3 << 16) | rawId4, can.filters.filterBank[i].FR2);
        ASSERT_TRUE((can.filters.FM1R & (1 << i)) == (1 << i)); // Ensure we are in ID mode
        ASSERT_TRUE((can.filters.FS1R & (1 << i)) == 0); //Ensure we are in Dual scale
        ASSERT_EQ(0, can.filters.FMR); //Ensure the FINIT bit is cleared
    }
}
