//
//  bxCANTests.cpp
//  Bedrock Tests
//
//  Created by Eric Yanush on 2016-01-04.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "gtest/gtest.h"
#include "bxCAN.hpp"

class bxCANTests : public ::testing::Test {
    
protected:
    
    virtual void SetUp() {
        can = (CAN::bxCAN*) malloc(sizeof(CAN::bxCAN));
    }
    
    virtual void TearDown() {
        free(can);
    }
    
    CAN::bxCAN* can;
};

/** Ensure layout of bxCAN object adheres with Register Map
 * as defined in the STM32F3 Programming Manual
 */
TEST_F(bxCANTests, TestCANRegLayout) {
    
    //Ensure base peripheral register offsets are correct
    ASSERT_EQ(0x000, offsetof(CAN::bxCAN, MCR));
    ASSERT_EQ(0x004, offsetof(CAN::bxCAN, MSR));
    ASSERT_EQ(0x008, offsetof(CAN::bxCAN, TSR));
    ASSERT_EQ(0x00C, offsetof(CAN::bxCAN, RF0R));
    ASSERT_EQ(0x010, offsetof(CAN::bxCAN, RF1R));
    ASSERT_EQ(0x014, offsetof(CAN::bxCAN, IER));
    ASSERT_EQ(0x018, offsetof(CAN::bxCAN, ESR));
    ASSERT_EQ(0x01C, offsetof(CAN::bxCAN, BTR));
    
    //Ensure Transmit Mailbox Register offsets are correct
    ASSERT_EQ(0x180, offsetof(CAN::bxCAN, txMailbox[0]));
    ASSERT_EQ(0x190, offsetof(CAN::bxCAN, txMailbox[1]));
    ASSERT_EQ(0x1A0, offsetof(CAN::bxCAN, txMailbox[2]));
    //Test the Register layout of the TxMailbox
    ASSERT_EQ(0x0, offsetof(CAN::TxMailbox, TIR));
    ASSERT_EQ(0x4, offsetof(CAN::TxMailbox, TDTR));
    ASSERT_EQ(0x8, offsetof(CAN::TxMailbox, TDLR));
    ASSERT_EQ(0xC, offsetof(CAN::TxMailbox, TDHR));
    
    //Ensure RX FIFO Queue Register offsets are correct
    ASSERT_EQ(0x1B0, offsetof(CAN::bxCAN, rxFIFO[0]));
    ASSERT_EQ(0x1C0, offsetof(CAN::bxCAN, rxFIFO[1]));
    //Test the Register layout of the RX Queues
    ASSERT_EQ(0x0, offsetof(CAN::RxFIFO, RIR));
    ASSERT_EQ(0x4, offsetof(CAN::RxFIFO, RDTR));
    ASSERT_EQ(0x8, offsetof(CAN::RxFIFO, RDLR));
    ASSERT_EQ(0xC, offsetof(CAN::RxFIFO, RDHR));
    
    //Ensure Filter Bank Register offsets are correct
    ASSERT_EQ(0x200, offsetof(CAN::bxCAN, filters));
    //Test the Register layout of the Filters
    ASSERT_EQ(0x00, offsetof(CAN::Filters, FMR));
    ASSERT_EQ(0x04, offsetof(CAN::Filters, FM1R));
    ASSERT_EQ(0x0C, offsetof(CAN::Filters, FS1R));
    ASSERT_EQ(0x14, offsetof(CAN::Filters, FF1AR));
    ASSERT_EQ(0x1C, offsetof(CAN::Filters, FA1R));
    ASSERT_EQ(0x40, offsetof(CAN::Filters, filterBank[0]));
    ASSERT_EQ(0x240, offsetof(CAN::bxCAN, filters.filterBank));
    ASSERT_EQ(0x48, offsetof(CAN::Filters, filterBank[1]));
    //Test the register layout of the filter banks
    ASSERT_EQ(0x0, offsetof(CAN::Filters::FilterBank, FR1));
    ASSERT_EQ(0x4, offsetof(CAN::Filters::FilterBank, FR2));
}

