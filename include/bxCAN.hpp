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
#include "Delay.hpp"

namespace Bedrock {
    namespace bxCAN {
        
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
            
            /**
             Note: These filter type structs assume little-endian hardware
             which both armv7(e)-m and x86(-64) are.
             */
            struct SingleIDFilter_t {
                uint32_t     : 1,
                rtr : 1,
                ide : 1,
                id  : 29;
            };
            
            struct SingleMaskFilter_t {
                uint32_t          : 1,
                rtr_mask : 1,
                ide_mask : 1,
                id_mask  : 29;
            };
            
            struct DualIDFilter_t {
                uint16_t extID : 3,
                ide   : 1,
                rtr   : 1,
                id    : 11;
            };
            
            struct DualMaskFilter_t {
                uint16_t extID_mask : 3,
                ide_mask   : 1,
                rtr_mask   : 1,
                id_mask    : 11;
            };
            
            /**
             Enable a filter bank
             
             - Parameter filterIndex: The index of the filter bank to enable
             */
            void enableFilterBank(uint8_t filterIndex) {
                FA1R |= (1 << filterIndex);
            }
            
            /**
             Disable a filter bank
             
             - Parameter filterIndex: The index of the filter bank to disable
             */
            void disableFilterBank(uint8_t filterIndex) {
                FA1R &= ~(1 << filterIndex);
            }
            
            /**
             Check if a filter bank is currently enabled
             
             - Paramter filterIndex: The index of the filter bank to check
             - Returns bool: true if enabled, false if not
             */
            bool isFilterBankEnabled(uint8_t filterIndex) {
                return (FA1R >> filterIndex) & 0x1;
            }
            
            enum class FilterConfig {
                //Bit 1 = Mode, Bit 0 = Scale
                DualMask = 0b00,
                SingleMask = 0b01,
                DualID = 0b10,
                SingleID = 0b11
            };
            
            /**
             Retrieve the current configuration for a filter bank
             
             - Parameter filterIndex: The index of the filter bank to check
             - Returns FilterConfig: The configuration type of the bank
             */
            FilterConfig getFilterConfig(uint8_t filterIndex) {
                uint8_t conf = (FM1R >> filterIndex) & 0x1; //extract the mode bit
                conf = (uint8_t)(conf << 1) | ((FS1R >> filterIndex) & 0x1); // put scale into bit 0 and mode into bit 1
                return static_cast<FilterConfig>(conf);
            }
            
            /**
             Configure a filter bank in Single-Width Mask mode; A frame may match the masked filter.
             
             - Parameter filterIndex: The filter bank index to configure
             - Parameter filter: The CAN Identifier to match against
             - Parameter mask: The mask to use with the CAN Identifier to match
             */
            void configureSingleMaskFilter(uint8_t filterIndex,
                                           SingleIDFilter_t& filter,
                                           SingleMaskFilter_t& mask)
            {
                FMR |= 0x1; //Set FINIT bit
                
                //Set the filter scale and mode
                FM1R &= ~(1 << filterIndex); //Set mode to 0 (mask mode)
                FS1R |= (1 << filterIndex); //Set scale to 1 (single width)
                
                filterBank[filterIndex].FR1 = *reinterpret_cast<uint32_t*>(&filter);
                filterBank[filterIndex].FR2 = *reinterpret_cast<uint32_t*>(&mask);
                
                FMR ^= 0x1; //Clear FINIT bit
            }
            /**
             Configure a filter bank in Single-Width ID List mode; A frame
             may exactly match any one of the two configured filters.
             
             - Parameter filterIndex: The filter bank index to configure
             - Parameter filter1: The first CAN Identifier to match against
             - Parameter filter2: The second CAN Identifier to match against
             */
            void configureSingleIDListFilter(uint8_t filterIndex,
                                             SingleIDFilter_t& filter1,
                                             SingleIDFilter_t& filter2)
            {
                FMR |= 1; //Set the FINIT bit
                
                //Set the filter scale and mode
                FM1R |= (1 << filterIndex); //Set mode to 1 (id mode)
                FS1R |= (1 << filterIndex); //Set scale to 1 (single width)
                
                filterBank[filterIndex].FR1 = *reinterpret_cast<uint32_t*>(&filter1);
                filterBank[filterIndex].FR2 = *reinterpret_cast<uint32_t*>(&filter2);
                
                FMR ^= 1; //Clear the FINIT bit
            }
            /**
             Configure a filter bank in Dual-Width Mask mode; A frame may match any one of the two masked filters.
             
             - Parameter filterIndex: The filter bank index to configure
             - Parameter filter1: The CAN Identifier to match against
             - Parameter mask1: The mask to use with filter1
             - Parameter filter2: A second CAN Identifier to match against
             - Paramter mask2: The mask to use with filter2
             */
            void configureDualMaskFilter(uint8_t filterIndex,
                                         DualIDFilter_t& filter1,
                                         DualMaskFilter_t& mask1,
                                         DualIDFilter_t& filter2,
                                         DualMaskFilter_t& mask2)
            {
                FMR |= 1; //Set the FINIT bit
                
                //Set the mode and scale
                FM1R &= ~(1 << filterIndex); //Set mode to 0 (mask mode)
                FS1R &= ~(1 << filterIndex); //Set the scale to 0 (Dual width)
                
                filterBank[filterIndex].FR1 = (*reinterpret_cast<uint16_t*>(&filter1) << 16) | (*reinterpret_cast<uint16_t*>(&filter2));
                filterBank[filterIndex].FR2 = (*reinterpret_cast<uint16_t*>(&mask1) << 16) | (*reinterpret_cast<uint16_t*>(&mask2));
                
                FMR ^= 1; //clear the FINIT bit
            }
            
            /**
             Configure a filter bank in Dual-Width ID List mode;  A frame may match exactly any one of four configured filters.
             
             - Parameter filterIndex: The filter bank index to configure
             - Parameter filter1: The first CAN Identifier to match against
             - Parameter mask1: The mask to use with filter1
             - Parameter filter2: The second CAN Identifier to match against
             - Parameter mask2: The mask to use with filter2
             - Parameter filter3: The third CAN Identifier to match against
             - Parameter mask3: The mask to use with filter3
             - Parameter filter4: The fourth CAN Identifier to match against
             - Parameter mask4: The mask to use with filter4
             */
            void configureDualIDListFilter(uint8_t filterIndex,
                                           DualIDFilter_t& filter1,
                                           DualIDFilter_t& filter2,
                                           DualIDFilter_t& filter3,
                                           DualIDFilter_t& filter4)
            {
                FMR |= 1; //Set FINIT bit
                
                //Set scale and mode
                FM1R |= (1 << filterIndex); //Set mode to 1 (ID mode)
                FS1R &= ~(1 << filterIndex); //Set the scale to 0 (Dual width)
                
                filterBank[filterIndex].FR1 = (*reinterpret_cast<uint16_t*>(&filter1) << 16) | (*reinterpret_cast<uint16_t*>(&filter2));
                filterBank[filterIndex].FR2 = (*reinterpret_cast<uint16_t*>(&filter3) << 16) | (*reinterpret_cast<uint16_t*>(&filter4));
                
                FMR ^= 1; //Clear the FINIT bit
            }
            
            /**
             Configure a filter bank to place matched frames into a specific FIFO queue
             
             - Parameter filter: The filter index of the filter to configure
             - Paramter fifo: The fifo index to place matched frames into
             
             */
            void assignFilterToFIFO(uint8_t filter, uint8_t fifo) {
                FMR |= 0x1; // Set the FINIT bit to allow changes
                
                if (fifo == 1) {
                    FFA1R |= (1 << filter);
                }
                else {
                    FFA1R &= ~(1 << filter);
                }
                
                FMR ^= 0x1; // Clear the FINIT bit
            }
            
            /**
             Get the configured fifo for a filter bank
             
             - Parameter filter: The filter index to retrieve the configured FIFO for
             - Returns uint8_t: The FIFO index assigned to the filter bank
             */
            uint8_t getAssignedFIFOForFilter(uint8_t filter) {
                return (FFA1R >> filter) & 0x1;
            }
            
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
            dev_reg32_t FFA1R;
            //Padding
            uint32_t _pad_3;
            dev_reg32_t FA1R;
            //Padding
            uint32_t _pad_4[8];
            FilterBank filterBank[14];
        };
        
        
        class CANPort {
        public:
            
            /**
             Method to put CAN peripheral in initialization mode
             
             Note: This method will BLOCK until the hardware has signaled that it has
             successfully entered init mode. (INAK bit in MSR is 1)
             */
            void enterInitMode() {
                //Write 1 to INRQ (Init Mode Request)
                MCR |= (ENABLE << 0);
                //Wait until the init mode flag has been set by hw
                while ((MSR & 0x1) != 0x1) { }
            }
            
            /**
             Method to put CAN peripheral into operational mode
             
             - parameter timeout: the number of milliseconds to wait for the bus to sync before giving up
             - returns bool: true if exit was successful, false if timeout was reached
             
             Note: This method will BLOCK until the hardware has signaled that it has
             successfully left init mode, or the timeout value was reached. (INAK bit in MSR is cleared)
             */
            bool exitInitMode(uint16_t timeout = UINT16_MAX) {
                //Clear Init Mode Request
                MCR &= ~(ENABLE << 0);
                //Wait until the init mode flag has been cleared by hw
                uint32_t start = Delay::getMillis();
                while ((MSR & 0x1) == 0x1) {
                    if ((Delay::getMillis() - start) > timeout) {
                        return false;
                    }
                }
                return true;
            }
            
            /**
             Collection of supported operational modes
             
             - Normal: Normal Operation Mode
             - Silent: Will not Transmit anything on the bus. (TX Loopback)
             - Loopback: Will not Recieve anything from the bus. (RX Loopback)
             - SelfTest: Will not Transmit or Recieve from the bus. (RX + TX Loopback)
             */
            enum class Mode : uint32_t {
                Normal   = 0b00,
                Silent   = 0b10,
                Loopback = 0b01,
                SelfTest = 0b11
            };
            
            /**
             Method to change the bus operation mode.
             
             - parameter newOpMode: The bus operation mode to switch to.
             
             Warning: Does not enter or exit init mode; needs to be done manually by caller
             */
            void setMode_unsafe(const Mode newOpMode) {
                constexpr uint8_t modeBitShift = 30;
                BTR &= ~(0b11 << modeBitShift);
                BTR |= static_cast<uint32_t>(newOpMode) << modeBitShift;
            }
            /**
             Method to change the bus operation mode.
             
             - parameter freq: The bus operation frequency to switch to.
             
             Note: This method is safe to call; it automatically handles the
             entrance to and exit from init mode.
             */
            void setMode(const Mode newOpMode) {
                enterInitMode(); //BTR RO outside of init mode
                setMode_unsafe(newOpMode);
                exitInitMode();
            }
            
            /**
             Retrieve the currently configured operation mode
             
             - returns: Mode
            */
            Mode getMode() {
                constexpr uint8_t modeBitShift = 30;
                return static_cast<Mode>((BTR >> modeBitShift) & 0b11);
            }
            
            /**
             Collection of supported bus frequencies
             */
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
             Method to change the bus operation frequency.
             
             - parameter freq: The bus operation frequency to switch to.
             - returns bool: true if the frequency was sucessfully changed, false if a timeout occurred
             Note: This method is safe to call; it automatically handles the
             entrance to and exit from init mode.
             
             Note: this method is currently hardcoded to use an AHB1 clock of 36MHz
             */
            bool setFrequency(const BusFrequency freq, uint16_t timeoutMS = UINT16_MAX) {
                enterInitMode();
                setFrequency_unsafe(freq);
                return exitInitMode(timeoutMS);
            }
            
            /**
             Method to change the bus operation frequency.
             
             - parameter freq: The bus operation frequency to switch to
             
             Note: this method is currently hardcoded to use an AHB1 clock of 36MHz
             
             Warning: This method does not enter or exit init mode, this must be done by the caller!
             */
            void setFrequency_unsafe(const BusFrequency freq) {
                BTR &= ~(0x1FF); //Clear the prescaler value ls 9-bits
                BTR |= static_cast<uint32_t>(freq);
            }
            
            /**
             Method to retrieve the current bus frequency
             
             - returns: BusFrequency currently configured
            */
            BusFrequency getFrequency() {
                return static_cast<BusFrequency>(BTR & 0x1FF);
            }
            
            /**
            Method to setup the CAN peripheral
            
            This method performs preliminary configuration of the peripheral.
            
            Postcondition: The bus will be ready for application configuration
                           (frequency, mode), and will be in init mode.
            
            Note: This doesn't configure a bus frequency or mode, it just configures
                  the peripheral for future use
            */
            void init() {
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
            
                BTR &= ~(SJW_RMASK | TS2_RMASK | TS1_RMASK | PRE_RMASK | MODE_RMASK); // clear current vals
                BTR |= (TS1 | TS2 | SJW); // Set the new values
                
                //Enable Automatic Retransmission, Automatic Bus-off, and disable Time-triggered mode and Sleep mode
                MCR &= ~((ENABLE << 7) | (ENABLE << 4) | (ENABLE << 1)); //Disable TTCM and NART
                MCR |= (ENABLE << 6); // Enable ABOM
            }
            
            /**
             Check if there is any avaialble TX mailboxes
             
             - Returns: bool; True if there are any available mailboxes, false otherwise
             */
            bool availableTXMailbox() {
                //Check bits 26, 27, 28 (Mailbox Empty 0, 1, 2)
                return (TSR & 0x1C000000) != 0x0;
            }
            
            /**
             Check if there are any messages waiting in a RX queue
             
             - Returns: uint8_t; The current count of messages pending in the queue.
             */
            uint8_t rxMessagesWaitingInFIFO(const uint8_t fifo) {
                //The bottom two bits of RFR register indicate message in FIFO count
                return (RFR[fifo] & 0x3);
            }
            
            /**
             Method to queue CANMessage for transmission
             
             - Parameter msg: A reference to a CANMessage to be transmitted on the bus
             
             - Returns: int8_t; if message was successfully queued the mailbox number the message was queued into is
             returned, otherwise -1 is returned if the message was not queued for transmission.
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
            
            /**
             Method to recieve a message from a RX queue
             
             - Parameter fifo: The queue number (0 or 1) to recieve a message from.
             
             - Parameter msg&: A reference to copy the message into.
             */
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
            /**
             Check if TX mailbox 0 has completed transmission
             
             - Returns: bool; True if it has, false otherwise
             */
            bool tx0Complete() {
                return (TSR & 0x1) == 0x1;
            }
            /**
             Acknowledge TX mailbox 0 has completed transmission; clear interrupt
             */
            void ackTX0Complete() {
                TSR |= 0x1; //Clear by writing 1
            }
            /**
             Check if TX mailbox 1 has completed transmission
             
             - Returns: bool; True if it has, false otherwise
             */
            bool tx1Complete() {
                return (TSR & (1 << 8)) == (1 << 8);
            }
            /**
             Acknowledge TX mailbox 1 has completed transmission; clear interrupt
             */
            void ackTX1Complete() {
                TSR |= (1 << 8); //Clear by writing 1
            }
            /**
             Check if TX mailbox 1 has completed transmission
             
             - Returns: bool; True if it has, false otherwise
             */
            bool tx2Complete() {
                return (TSR & (1 << 16)) == (1 << 16);
            }
            /**
             Acknowledge TX mailbox 0 has completed transmission; clear interrupt
             */
            void ackTX2Complete() {
                TSR |= 1 << 16; //Clear by writing 1
            }
            
            /**
             Enable generation of interrupt on the bus entering sleep mode
             */
            void enableSleepInterrupt() {
                IER |= (ENABLE << 17);
            }
            /**
             Enable generation of interrupt on detection of SOF while in sleep mode
             */
            void enableWakupInterrupt() {
                IER |= (ENABLE << 16);
            }
            /**
             Enable interrupt generation when there is a pending error condition
             */
            void enableErrorInterrupt() {
                IER |= (ENABLE << 15);
            }
            /**
             Enable interrupt generation when an error code has been set
             */
            void enableLastErrorCodeInterrupt() {
                IER |= (ENABLE << 11);
            }
            /**
             Enable interrupt generation when the bus enters Bus-Off state
             */
            void enableBusOffInterrupt() {
                IER |= (ENABLE << 10);
            }
            /**
             Enable interrupt generation when the bus enters Error-Passive state
             */
            void enableErrorPassiveInterrupt() {
                IER |= (ENABLE << 9);
            }
            /**
             Enable interrrupt generation when the bus enters Error-Warning state
             */
            void enableErrorWarnInterrupt() {
                IER |= (ENABLE << 8);
            }
            /**
             Enable interrupt generation when FIFO 1 overruns
             */
            void enableFIFO1OverrunInterrupt() {
                IER |= (ENABLE << 6);
            }
            /**
             Enable interrupt generation when FIFO 1 is full
             */
            void enableFIFO1FullInterrupt() {
                IER |= (ENABLE << 5);
            }
            /**
             Enable interrupt generation when FIFO 1 has a message pending in its queue
             */
            void enableFIFO1MessagePendingInterrupt() {
                IER |= (ENABLE << 4);
            }
            /**
             Enable interrupt generation when FIFO 0 overruns
             */
            void enableFIFO0OverrunInterrupt() {
                IER |= (ENABLE << 3);
            }
            /**
             Enable interrupt generation when FIFO 0  is full
             */
            void enableFIFO0FullInterrupt() {
                IER |= (ENABLE << 2);
            }
            /**
             Enable interrupt generation when FIFO 0 has a message pending in its queue
             */
            void enableFIFO0MessagePendingInterrupt() {
                IER |= (ENABLE << 1);
            }
            /**
             Enable interrupt generation when a TX mailbox becomes empty (Completes transmission)
             */
            void enableTXMailboxEmptyInterrupt() {
                IER |= (ENABLE);
            }
            
            /**
             Disable interrupt generation when bus enters sleep mode
             */
            void disableSleepInterrupt() {
                IER &= ~(ENABLE << 17);
            }
            /**
             Disable interrupt generation when SOF detected in sleep mode
             */
            void disableWakupInterrupt() {
                IER &= ~(ENABLE << 16);
            }
            /**
             Disable interrupt generation when there is a pending error condition
             */
            void disableErrorInterrupt() {
                IER &= ~(ENABLE << 15);
            }
            /**
             Disable interrupt generation when an error code is set by hardware
             */
            void disableLastErrorCodeInterrupt() {
                IER &= ~(ENABLE << 11);
            }
            /**
             Disable interrupt generation when the bus enters Bus-off state
             */
            void disableBusOffInterrupt() {
                IER &= ~(ENABLE << 10);
            }
            /**
             Disable interrupt generation when the bus enters Passive-Error state
             */
            void disableErrorPassiveInterrupt() {
                IER &= ~(ENABLE << 9);
            }
            /**
             Disable interrupt generation when the bus enters Error-Warning state
             */
            void disableErrorWarnInterrupt() {
                IER &= ~(ENABLE << 8);
            }
            /**
             Disable interrupt generation when FIFO 1 overruns
             */
            void disableFIFO1OverrunInterrupt() {
                IER &= ~(ENABLE << 6);
            }
            /**
             Disable interrupt generation when FIFO 1 is full
             */
            void disableFIFO1FullInterrupt() {
                IER &= ~(ENABLE << 5);
            }
            /**
             Disable interrupt generation when FIFO 1 has a pending message
             */
            void disableFIFO1MessagePendingInterrupt() {
                IER &= ~(ENABLE << 4);
            }
            /**
             Disable interrupt generation when FIFO 0 overruns
             */
            void disableFIFO0OverrunInterrupt() {
                IER &= ~(ENABLE << 3);
            }
            /**
             Disable interrupt generation when FIFO 0 is full
             */
            void disableFIFO0FullInterrupt() {
                IER &= ~(ENABLE << 2);
            }
            /**
             Disable interrupt generation when FIFO 0 has a message pending.
             */
            void disableFIFO0MessagePendingInterrupt() {
                IER &= ~(ENABLE << 1);
            }
            /**
             Disable interrupt generation when a TX mailbox becomes empty (Completes transmission)
             */
            void disableTXMailboxEmptyInterrupt() {
                IER &= ~(ENABLE);
            }
            
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
            
            /**
             Retrieve the current Recieve Error count from hardware
             
             - Returns: uint8_t; The current recieve error count
             */
            uint8_t getCurrentRXErrorCount() {
                return (ESR >> 24);
            }
            /**
             Retrieve the current Transmite Error count from hardware
             
             - Returns: uint8_t; The current transmit error count
             */
            uint8_t getCurrentTXErrorCount() {
                return (ESR >> 16) & 0xFF;
            }
            /**
             Retrieve the last Error Code set by the hardware
             
             - Returns: ErrorCode; The last error code type set by the hardware
             */
            ErrorCode getLastErrorCode() {
                return static_cast<ErrorCode>((ESR >> 4) & 0x7);
            }
            /**
             Check if the bus is currently in Bus-Off state
             
             - Returns: bool; True if bus is in Bus-Off state, false otherwise
             */
            bool isInBusOffState() {
                return (ESR & 0x4) == 0x4;
            }
            /**
             Check if the bus is currently in Error-Warning state
             
             - Returns: bool; True if the bus is in Error-Warning state, false otherwise
             */
            bool isInErrorWarningState() {
                return (ESR & 0x1) == 0x1;
            }
            /**
             Check if the bus is currently in Error-Passive state
             
             - Returns: bool; True if the bus is in Error-Passive state, false otherwise
             */
            bool isInErrorPassiveState() {
                return (ESR & 0x2) == 0x2;
            }
            
            /**
             Clear the pending Error Interrupt
             */
            void ackErrorInterrupt() {
                MSR &= ~(0x1 << 2);
            }
            
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
        
        using CANPortProvider = CANPort& (*)(void);
    }
}

#endif /* bxCan_h */
