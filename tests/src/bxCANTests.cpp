//
//  bxCANTests.cpp
//  Bedrock Tests
//
//  Created by Eric Yanush on 2016-01-04.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "gtest/gtest.h"
#include "bxCAN.hpp"

//Includes for async tests
#include <thread>
#include <future>
#include <chrono>


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
    
    CAN::CAN can{0};
    CAN::CAN untouched{0};
    CANMessage testMessage;
};

/** Ensure layout of bxCAN object adheres with Register Map
 * as defined in the STM32F3 Programming Manual
 */
TEST_F(CANTests, TestCANRegLayout) {
    
    //Ensure base peripheral register offsets are correct
    ASSERT_EQ(0x000, offsetof(CAN::CAN, MCR));
    ASSERT_EQ(0x004, offsetof(CAN::CAN, MSR));
    ASSERT_EQ(0x008, offsetof(CAN::CAN, TSR));
    ASSERT_EQ(0x00C, offsetof(CAN::CAN, RFR[0]));
    ASSERT_EQ(0x010, offsetof(CAN::CAN, RFR[1]));
    ASSERT_EQ(0x014, offsetof(CAN::CAN, IER));
    ASSERT_EQ(0x018, offsetof(CAN::CAN, ESR));
    ASSERT_EQ(0x01C, offsetof(CAN::CAN, BTR));
    
    //Ensure Transmit Mailbox Register offsets are correct
    ASSERT_EQ(0x180, offsetof(CAN::CAN, txMailbox[0]));
    ASSERT_EQ(0x190, offsetof(CAN::CAN, txMailbox[1]));
    ASSERT_EQ(0x1A0, offsetof(CAN::CAN, txMailbox[2]));
    //Test the Register layout of the TxMailbox
    ASSERT_EQ(0x0, offsetof(CAN::TxMailbox, TIR));
    ASSERT_EQ(0x4, offsetof(CAN::TxMailbox, TDTR));
    ASSERT_EQ(0x8, offsetof(CAN::TxMailbox, TDLR));
    ASSERT_EQ(0xC, offsetof(CAN::TxMailbox, TDHR));
    
    //Ensure RX FIFO Queue Register offsets are correct
    ASSERT_EQ(0x1B0, offsetof(CAN::CAN, rxFIFO[0]));
    ASSERT_EQ(0x1C0, offsetof(CAN::CAN, rxFIFO[1]));
    //Test the Register layout of the RX Queues
    ASSERT_EQ(0x0, offsetof(CAN::RxFIFO, RIR));
    ASSERT_EQ(0x4, offsetof(CAN::RxFIFO, RDTR));
    ASSERT_EQ(0x8, offsetof(CAN::RxFIFO, RDLR));
    ASSERT_EQ(0xC, offsetof(CAN::RxFIFO, RDHR));
    
    //Ensure Filter Bank Register offsets are correct
    ASSERT_EQ(0x200, offsetof(CAN::CAN, filters));
    //Test the Register layout of the Filters
    ASSERT_EQ(0x00, offsetof(CAN::Filters, FMR));
    ASSERT_EQ(0x04, offsetof(CAN::Filters, FM1R));
    ASSERT_EQ(0x0C, offsetof(CAN::Filters, FS1R));
    ASSERT_EQ(0x14, offsetof(CAN::Filters, FF1AR));
    ASSERT_EQ(0x1C, offsetof(CAN::Filters, FA1R));
    ASSERT_EQ(0x40, offsetof(CAN::Filters, filterBank[0]));
    ASSERT_EQ(0x240, offsetof(CAN::CAN, filters.filterBank));
    ASSERT_EQ(0x48, offsetof(CAN::Filters, filterBank[1]));
    //Test the register layout of the filter banks
    ASSERT_EQ(0x0, offsetof(CAN::Filters::FilterBank, FR1));
    ASSERT_EQ(0x4, offsetof(CAN::Filters::FilterBank, FR2));
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
    using CAN = CAN::CAN;
    
    auto asyncHW = [this]() {
        //Wait for an init mode request
        while ((can.MCR & 0x1) != 0x1) {
            std::this_thread::sleep_for(100us);
        }
        can.MSR |= 0x1; // set the INAK bit
        //Wait for request to leave init mode
        while ((can.MCR & 0x1) == 0x1) {
            std::this_thread::sleep_for(100us);
        }
        can.MSR &= ~(0x1);
    };
    std::thread async(asyncHW);
    
    can.init();
    async.join();
    
    ASSERT_FALSE((can.MSR & 0x1) == 0x1); //Ensure we are not still in init mode
    ASSERT_FALSE((can.MCR & 0x1) == 0x1);
    ASSERT_EQ(CAN::BusFrequency::KHz_125, static_cast<CAN::BusFrequency>(can.BTR & 0x1FF)); //Ensure we have set the prescaler value properly
    ASSERT_EQ(8, ((can.BTR & 0x000F0000) >> 16) + 1); //Ensure we have setup BS1 to have 8tq
    ASSERT_EQ(3, ((can.BTR & 0x00700000) >> 20) + 1); //Ensure we have setup BS2 to have 3tq
    ASSERT_EQ(1, ((can.BTR & 0x03000000) >> 24) + 1); //Ensure we have setup the Sync Jump to be 1tq
    ASSERT_EQ(CAN::Mode::Normal, static_cast<CAN::Mode>((can.BTR & 0xC0000000) >> 30)); //Ensure we have setup normal mode
    ASSERT_FALSE((can.MCR & 0x10) == 0x10); //Ensure we have Automatic Retransmission enabled (NART bit not set)
    ASSERT_TRUE((can.MCR & 0x40) == 0x40); // Ensure we have Automatic Bus-off management enabled (ABOM bit set)
    ASSERT_FALSE((can.MCR & 0x80) == 0x80); //Ensure we have Time Triggered Mode off (TTCM bit not set)
}

TEST_F(CANTests, TestInit) {
    using namespace std::literals;
    using CAN = CAN::CAN;
    
    auto asyncHW = [this]() {
        //Wait for an init mode request
        while ((can.MCR & 0x1) != 0x1) {
            std::this_thread::sleep_for(100us);
        }
        can.MSR |= 0x1; // set the INAK bit
        //Wait for request to leave init mode
        while ((can.MCR & 0x1) == 0x1) {
            std::this_thread::sleep_for(100us);
        }
        can.MSR &= ~(0x1);
    };
    std::thread async(asyncHW);
    
    can.init(CAN::BusFrequency::MHz_1, CAN::Mode::SelfTest);
    async.join();
    
    ASSERT_FALSE((can.MSR & 0x1) == 0x1); //Ensure we are not still in init mode
    ASSERT_FALSE((can.MCR & 0x1) == 0x1);
    ASSERT_EQ(CAN::BusFrequency::MHz_1, static_cast<CAN::BusFrequency>(can.BTR & 0x1FF)); //Ensure we have set the prescaler value properly
    ASSERT_EQ(8, ((can.BTR & 0x000F0000) >> 16) + 1); //Ensure we have setup BS1 to have 8tq
    ASSERT_EQ(3, ((can.BTR & 0x00700000) >> 20) + 1); //Ensure we have setup BS2 to have 3tq
    ASSERT_EQ(1, ((can.BTR & 0x03000000) >> 24) + 1); //Ensure we have setup the Sync Jump to be 1tq
    ASSERT_EQ(CAN::Mode::SelfTest, static_cast<CAN::Mode>((can.BTR & 0xC0000000) >> 30)); //Ensure we have setup self-test mode
    ASSERT_FALSE((can.MCR & 0x10) == 0x10); //Ensure we have Automatic Retransmission enabled (NART bit not set)
    ASSERT_TRUE((can.MCR & 0x40) == 0x40); // Ensure we have Automatic Bus-off management enabled (ABOM bit set)
    ASSERT_FALSE((can.MCR & 0x80) == 0x80); //Ensure we have Time Triggered Mode disabled (TTCM bit not set)
}

TEST_F(CANTests, TestSetMode_unsafe) {
    using CAN = CAN::CAN;
    can.setMode_unsafe(CAN::Mode::SelfTest);
    ASSERT_EQ(CAN::Mode::SelfTest, static_cast<CAN::Mode>((can.BTR & 0xC0000000) >> 30));
    ASSERT_FALSE((can.MCR & 0x1) == 0x1); //Ensure we didn't try to enter into init mode
}

TEST_F(CANTests, TestSetMode) {
    using namespace std::literals;
    using CAN = CAN::CAN;
    
    auto asyncHW = [this]() {
        //wait for an init request
        while ((can.MCR & 0x1) != 0x1) {
            std::this_thread::sleep_for(100us);
        }
        can.MSR |= 0x1; // Set the INAK bit
        //wait for request to leave init mode
        while ((can.MCR & 0x1) == 0x1) {
            std::this_thread::sleep_for(100us);
        }
        can.MSR &= ~(0x1); // Clear the INAK bit
    };
    
    std::thread async(asyncHW);
    
    can.setMode(CAN::Mode::Silent);
    async.join();
    
    ASSERT_EQ(CAN::Mode::Silent, static_cast<CAN::Mode>((can.BTR & 0xC0000000) >> 30));
    ASSERT_FALSE((can.MSR & 0x1) == 0x1); //Ensure we are not still in init mode
    ASSERT_FALSE((can.MCR & 0x1) == 0x1);
}

TEST_F(CANTests, TestSetFrequency_unsafe) {
    using CAN = CAN::CAN;
    can.setFrequency_unsafe(CAN::BusFrequency::KHz_250);
    
    ASSERT_EQ(CAN::BusFrequency::KHz_250, static_cast<CAN::BusFrequency>(can.BTR & 0x1FF));
    ASSERT_FALSE((can.MCR & 0x1) == 0x1); //Ensure we didn't try to enter into init mode
}

TEST_F(CANTests, TestSetFrequency) {
    using namespace std::literals;
    using CAN = CAN::CAN;
    
    auto asyncHW = [this]() {
        //Wait for an init mode request
        while ((can.MCR & 0x1) != 0x1) {
            std::this_thread::sleep_for(100us);
        }
        can.MSR |= 0x1; //Set the INAK bit
        //Wait for an exit init mode request
        while ((can.MCR & 0x1) == 0x1) {
            std::this_thread::sleep_for(100us);
        }
        can.MSR &= ~(0x1);
    };
    std::thread async(asyncHW);
    
    can.setFrequency(CAN::BusFrequency::KHz_20);
    async.join();
    
    ASSERT_FALSE((can.MSR & 0x1) == 0x1); //Ensure we aren't still in init mode
    ASSERT_FALSE((can.MCR & 0x1) == 0x1);
    ASSERT_EQ(CAN::BusFrequency::KHz_20, static_cast<CAN::BusFrequency>(can.BTR & 0x1FF));
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
    ASSERT_EQ(0, memcmp(&can, &untouched, sizeof(CAN::CAN))); //Ensure we didn't try to write anything out
    
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
