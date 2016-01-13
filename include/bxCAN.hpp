//
//  bxCan.hpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-04.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef bxCan_h
#define bxCan_h

#include <stdint.h>
#include "types.hpp"
#include "CANMessage.hpp"
#include "RCC.hpp"

namespace CAN {
    
    class TxMailbox {
    public:
        dev_reg32_t TIR;
        dev_reg32_t TDTR;
        dev_reg32_t TDLR;
        dev_reg32_t TDHR;
    };
    
    class RxFIFO {
    public:
        dev_reg32_t RIR;
        dev_reg32_t RDTR;
        dev_reg32_t RDLR;
        dev_reg32_t RDHR;
    };
    
    class Filters {
    public:
        struct FilterBank {
            dev_reg32_t FR1;
            dev_reg32_t FR2;
        };
        
        dev_reg32_t FMR;
        dev_reg32_t FM1R;
        //Padding
        uint32_t _pad_1;
        dev_reg32_t FS1R;
        //Padding
        uint32_t _pad_2;
        dev_reg32_t FF1AR;
        //Padding
        uint32_t _pad_3;
        dev_reg32_t FA1R;
        //Padding
        uint32_t _pad_4[8];
        FilterBank filterBank[14];
    };
    
    
    class CAN {
    public:
        
        /**
         enterInitMode: Method to put CAN peripheral in initialization mode
         Note: This method will BLOCK until the hardware has signaled that it hase
               successfully entered init mode. (INAK bit in MSR is 1)
         */
        void enterInitMode() {
            //Write 1 to INRQ (Init Mode Request)
            MCR |= (ENABLE << 0);
            //Wait until the init mode flag has been set by hw
            while ((MSR & 0x1) != 0x1) {}
        }
        
        /**
         exitInitMode: Method to put CAN peripheral in normal op mode
         Note: This method will BLOCK until the hardware has signaled that it hase
         successfully left init mode. (INAK bit in MSR is cleared)
         */
        void exitInitMode() {
            //Clear Init Mode Request
            MCR &= ~(ENABLE << 0);
            //Wait until the init mode flag has been cleared by hw
            while ((MSR & 0x1) == 0x1) {}
        }
        
        enum class Mode : uint32_t {
            Normal   = 0b00,
            //Silent Mode: Does not TX anything. (TX Loopback)
            Silent   = 0b10,
            //Loopback Mode: Does not RX anything. (RX Loopback)
            Loopback = 0b01,
            //Selftest: Does not RX or TX anything. (RX + TX Loopback)
            SelfTest = 0b11,
        };
        
        /**
         Warning: Does not enter or exit init mode; needs to be done manually by caller
         */
        void setMode_unsafe(const Mode newOpMode) {
            constexpr uint8_t modeBitShift = 30;
            BTR &= ~(0b11 << modeBitShift);
            BTR |= static_cast<uint32_t>(newOpMode) << modeBitShift;
        }
        void setMode(const Mode newOpMode) {
            enterInitMode(); //BTR RO outside of init mode
            setMode_unsafe(newOpMode);
            exitInitMode();
        }
        
        
        enum class BusFrequency : uint16_t {
            //Values are prescaler values using 12TQ
            //  They are PRE_VAL - 1 as the bxCAN adds 1 in hw
            MHz_1   = (3 - 1),
            KHz_500 = (6 - 1),
            KHz_250 = (12 - 1),
            KHz_125 = (24 - 1),
            KHz_100 = (30 - 1),
            KHz_50  = (60 - 1),
            KHz_20  = (150 - 1),
            KHz_10  = (300 - 1)
        };
        /**
         Note: this method is currently hardcoded to use an AHB1 clock of 36MHz
         */
        void setFrequency(const BusFrequency freq) {
            enterInitMode();
            setFrequency_unsafe(freq);
            exitInitMode();
        }
        
        /**
         Note: this method is currently hardcoded to use an AHB1 clock of 36MHz
         Warning: This method does not enter or exit init mode, this must be done by the caller!
         */
        void setFrequency_unsafe(const BusFrequency freq) {
            BTR &= ~(0x1FF); //Clear the prescaler value ls 9-bits
            BTR |= static_cast<uint32_t>(freq);
        }
        
        /**
         init: Method to setup the CAN peripheral, sets up bit timings
         */
        void init(const BusFrequency freq = BusFrequency::KHz_125, const Mode opMode = Mode::Normal) {
            enterInitMode();
            
            constexpr uint32_t PRE_RMASK = 0x1FF;
            constexpr uint32_t MODE_RMASK = 0b11 << 30;
            // We are using a hardcoded 12TQ, with a sample pos of 75%
            constexpr uint32_t bs1 = 8; // 8tq in BS1
            constexpr uint32_t bs2 = 3; // 3tq in BS1
            constexpr uint32_t TS1 = (bs1 - 1) << 16;
            constexpr uint32_t TS1_RMASK = 0b1111 << 16;
            constexpr uint32_t TS2 = (bs2 - 1) << 20;
            constexpr uint32_t TS2_RMASK = 0b111 << 20;
            constexpr uint32_t SJW = (1 - 1) << 24;
            constexpr uint32_t SJW_RMASK = 0b11 << 24;
            
            uint32_t pre_val = static_cast<uint32_t>(freq);
            uint32_t mod_val = static_cast<uint32_t>(opMode) << 30;
            BTR &= ~(SJW_RMASK | TS2_RMASK | TS1_RMASK | PRE_RMASK | MODE_RMASK); // clear current vals
            BTR |= (TS1 | TS2 | SJW | pre_val | mod_val); // Set the new values
            
            //Enable Automatic Retransmission, Automatic Bus-off, and disable Time-triggered mode
            MCR &= ~((ENABLE << 7) | (ENABLE << 4)); //Disable TTCM and NART
            MCR |= (ENABLE << 6); // Enable ABOM
            
            exitInitMode();
        }
        
        
        bool availableTXMailbox() {
            //Check bits 26, 27, 28 (Mailbox Empty 0, 1, 2)
            return (TSR & 0x1C000000) != 0x0;
        }
        
        uint8_t rxMessagesWaitingInFIFO(const uint8_t fifo) {
            //The bottom two bits of RFR register indicate message in FIFO count
            return (RFR[fifo] & 0x3);
        }
        
        /**
         transmitMessage: Method to queue CANMessage for transmission
         parameter msg: A reference to a CANMessage to be transmitted on the bus
         returns: if message was successfully queued the mailbox number the message was queued into is returned,
                  otherwise -1 is returned if the message was not queued for transmission.
         */
        int8_t transmitMessage(const CANMessage& msg) {
            uint8_t targetMB = 0;
            //Get first available tx mailbox
            uint8_t mbEmptyFlags = (TSR & 0x1C000000) >> 26;
            if ((mbEmptyFlags & 0x1) == 0x1) {
                targetMB = 0;
            }
            else if ((mbEmptyFlags & 0x2) == 0x2) {
                targetMB = 1;
            }
            else if ((mbEmptyFlags & 0x4) == 0x4) {
                targetMB = 2;
            }
            else {
                //No available transmit mailbox, return false
                return -1;
            }
            
            uint32_t idr;
            if (msg.format == CANMessage::Format::Extended) {
                idr = msg.id << 3;
                idr |= 0x4; //Set the extended id flag
            }
            else {
                idr = msg.id << 21;
            }
            if (msg.type == CANMessage::Type::Remote) {
                idr |= 0x2; //Set the RTR bit
            }
            idr |= 0x1; //Set the TXRQ bit
            //Set the DLC
            txMailbox[targetMB].TDTR |= (msg.dataLen & 0xF); //DLC is 4 bits wide
            //Copy data into data registers
            txMailbox[targetMB].TDLR = msg.data[0] | (msg.data[1] << 8) |
                                      (msg.data[2] << 16) | (msg.data[3] << 24);
            txMailbox[targetMB].TDHR = msg.data[4] | (msg.data[5] << 8) |
                                      (msg.data[6] << 16) | (msg.data[7] << 24);
            
            //Copy the id into the registers, and request transmission
            txMailbox[targetMB].TIR = idr;
            
            return targetMB;
        }
        void recieveMessageFromFIFO(const int8_t fifo, CANMessage& msg) {
            //Copy The Identifier from the mailbox
            const uint32_t idReg = rxFIFO[fifo].RIR;
            if ((idReg & 0x4) == 0x4) {
                msg.format = CANMessage::Format::Extended;
                msg.id = idReg >> 3;
            }
            else {
                msg.format = CANMessage::Format::Standard;
                msg.id = idReg >> 21;
            }
            //Set the frame type
            msg.type = ((idReg & 0x2) == 0x2) ? CANMessage::Type::Remote : CANMessage::Type::Data;
            //Copy the data length into the message
            msg.dataLen = rxFIFO[fifo].RDTR & 0xF;
            //Copy the data into the message
            uint32_t* msgData;
            msgData = reinterpret_cast<uint32_t*>(msg.data);
            msgData[0] = rxFIFO[fifo].RDLR;
            msgData[1] = rxFIFO[fifo].RDHR;
            
            //release the fifo
            RFR[fifo] |= 1 << 5;
        }
        
        //Peripheral Interrupt methods
        bool tx0Complete();
        void ackTX0Complete();
        bool tx1Complete();
        void ackTX1Complete();
        bool tx2Complete();
        void ackTX2Complete();
        
        void releaseFIFO0();
        void releaseFIFO1();
        
        void enableSleepInterrupt();
        void enableWakupInterrupt();
        void enableErrorInterrupt();
        void enableLastErrorCodeInterrupt();
        void enableBusOffInterrupt();
        void enableErrorPassiveInterrupt();
        void enableErrorWarnInterrupt();
        void enableFIFO0OverrunInterrupt();
        void enableFIFO0FullInerrupt();
        void enableFIFO0MessagePendingInterrupt();
        void enableFIFO1OverrunInterrupt();
        void enableFIFO1FullInerrupt();
        void enableFIFO1MessagePendingInterrupt();
        void enableTXMailboxEmptyInterrupt();
        
        void disableSleepInterrupt();
        void disableWakupInterrupt();
        void disableErrorInterrupt();
        void disableLastErrorCodeInterrupt();
        void disableBusOffInterrupt();
        void disableErrorPassiveInterrupt();
        void disableErrorWarnInterrupt();
        void disableFIFO0OverrunInterrupt();
        void disableFIFO0FullInerrupt();
        void disableFIFO0MessagePendingInterrupt();
        void disableFIFO1OverrunInterrupt();
        void disableFIFO1FullInerrupt();
        void disableFIFO1MessagePendingInterrupt();
        void disableTXMailboxEmptyInterrupt();
        
        //Error handling methods
        enum class ErrorCode : uint8_t {
            NoError           = 0b000,
            StuffError        = 0b001,
            FormError         = 0b010,
            AcknowledgeError  = 0b011,
            BitRecessiveError = 0b100,
            BitDominantError  = 0b101,
            CRCError          = 0b110,
            SetBySoftware     = 0b111
        };
        uint8_t getCurrentRXErrorCount();
        uint8_t getCurrentTXErrorCount();
        ErrorCode getLastErrorCode();
        bool isInBusOffMode();
        bool isInErrorWarningMode();
        bool isInErrorPassiveMode();
        
        dev_reg32_t MCR;
        dev_reg32_t MSR;
        dev_reg32_t TSR;
        dev_reg32_t RFR[2];
        dev_reg32_t IER;
        dev_reg32_t ESR;
        dev_reg32_t BTR;
        
        //anonymous padding between base registers and mailbox registers
        uint32_t _pad_1[88];
        
        TxMailbox txMailbox[3];
        
        RxFIFO rxFIFO[2];
        
        //anonymous padding between RX Queues and Filter Registers
        uint32_t _pad_2[12];
        
        Filters filters;
    };
}

#endif /* bxCan_h */
