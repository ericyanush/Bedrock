/**
  ******************************************************************************
  * @file    stm32f303xe.h
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    12-Sept-2014
  * @brief   CMSIS STM32F303xE Devices Peripheral Access Layer Header File.
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripheral�s registers hardware
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT= c; 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES = INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION; HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT = INCLUDING NEGLIGENCE OR OTHERWISE; ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS_Device
  * @{
  */

/** @addtogroup stm32f303xe
  * @{
  */

#ifndef __STM32F303xE_H
#define __STM32F303xE_H

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
 * @brief Configuration of the Cortex-M4 Processor and Core Peripherals
 */
#define __CM4_REV                 0x0001  /*!< Core revision r0p1                             */
#define __MPU_PRESENT             1       /*!< STM32F303xE devices provide an MPU */
#define __NVIC_PRIO_BITS          4       /*!< STM32F303xE devices use 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0       /*!< Set to 1 if different SysTick Config is used */
#define __FPU_PRESENT             1       /*!< STM32F303xE devices provide an FPU */

/**
  * @}
  */

/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief STM32F303xE devices Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line 19          */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line 20                     */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_TSC_IRQn              = 8,      /*!< EXTI Line2 Interrupt and Touch Sense Controller Interrupt         */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Channel1_IRQn          = 11,     /*!< DMA1 Channel 1 Interrupt                                          */
  DMA1_Channel2_IRQn          = 12,     /*!< DMA1 Channel 2 Interrupt                                          */
  DMA1_Channel3_IRQn          = 13,     /*!< DMA1 Channel 3 Interrupt                                          */
  DMA1_Channel4_IRQn          = 14,     /*!< DMA1 Channel 4 Interrupt                                          */
  DMA1_Channel5_IRQn          = 15,     /*!< DMA1 Channel 5 Interrupt                                          */
  DMA1_Channel6_IRQn          = 16,     /*!< DMA1 Channel 6 Interrupt                                          */
  DMA1_Channel7_IRQn          = 17,     /*!< DMA1 Channel 7 Interrupt                                          */
  ADC1_2_IRQn                 = 18,     /*!< ADC1 & ADC2 Interrupts                                            */
  USB_HP_CAN_TX_IRQn          = 19,     /*!< USB Device High Priority or CAN TX Interrupts                     */
  USB_LP_CAN_RX0_IRQn         = 20,     /*!< USB Device Low Priority or CAN RX0 Interrupts                     */
  CAN_RX1_IRQn                = 21,     /*!< CAN RX1 Interrupt                                                 */
  CAN_SCE_IRQn                = 22,     /*!< CAN SCE Interrupt                                                 */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break and TIM15 Interrupts                                   */
  TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update and TIM16 Interrupts                                  */
  TIM1_TRG_COM_TIM17_IRQn     = 26,     /*!< TIM1 Trigger and Commutation and TIM17 Interrupt                  */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt & EXTI Line23 Interrupt = I2C1 wakeup;        */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt & EXTI Line24 Interrupt = I2C2 wakeup;        */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt & EXTI Line25 Interrupt = USART1 wakeup;   */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt & EXTI Line26 Interrupt = USART2 wakeup;   */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt & EXTI Line28 Interrupt = USART3 wakeup;   */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm = A and B; through EXTI Line 17 Interrupt                 */
  USBWakeUp_IRQn              = 42,     /*!< USB Wakeup Interrupt                                              */
  TIM8_BRK_IRQn               = 43,     /*!< TIM8 Break Interrupt                                              */
  TIM8_UP_IRQn                = 44,     /*!< TIM8 Update Interrupt                                             */
  TIM8_TRG_COM_IRQn           = 45,     /*!< TIM8 Trigger and Commutation Interrupt                            */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                                             */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt & EXTI Line34 Interrupt = UART4 wakeup;     */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt & EXTI Line35 Interrupt = UART5 wakeup;     */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC channel 1&2 underrun error  interrupts        */
  TIM7_IRQn                   = 55,     /*!< TIM7 global Interrupt                                             */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                                   */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                                   */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                                   */
  DMA2_Channel4_IRQn          = 59,     /*!< DMA2 Channel 4 global Interrupt                                   */
  DMA2_Channel5_IRQn          = 60,     /*!< DMA2 Channel 5 global Interrupt                                   */
  ADC4_IRQn                   = 61,     /*!< ADC4  global Interrupt                                            */
  COMP1_2_3_IRQn              = 64,     /*!< COMP1, COMP2 and COMP3 global Interrupt via EXTI Line21, 22 and 29*/
  COMP4_5_6_IRQn              = 65,     /*!< COMP4, COMP5 and COMP6 global Interrupt via EXTI Line30, 31 and 32*/
  COMP7_IRQn                  = 66,     /*!< COMP7 global Interrupt via EXTI Line33                            */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  USB_HP_IRQn                 = 74,     /*!< USB High Priority global Interrupt remap                          */
  USB_LP_IRQn                 = 75,     /*!< USB Low Priority global Interrupt  remap                          */
  USBWakeUp_RMP_IRQn          = 76,     /*!< USB Wakeup Interrupt remap                                        */
  TIM20_BRK_IRQn              = 77,     /*!< TIM20 Break Interrupt                                              */
  TIM20_UP_IRQn               = 78,     /*!< TIM20 Update Interrupt                                             */
  TIM20_TRG_COM_IRQn          = 79,     /*!< TIM20 Trigger and Commutation Interrupt                            */
  TIM20_CC_IRQn               = 80,     /*!< TIM20 Capture Compare Interrupt                                    */
  FPU_IRQn                    = 81,     /*!< Floating point Interrupt                                          */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
} IRQn_Type;

/**
  * @}
  */

#include <stdint.h>

/** @addtogroup Peripheral_memory_map
  * @{
  */

constexpr uint32_t FLASH_BASE            = 0x08000000; /*!< FLASH= 512KB; base address in the alias region                                */
constexpr uint32_t CCMDATARAM_BASE       = 0x10000000; /*!< CCM= core coupled memory; data RAM= 16 KB; base address in the alias region    */
constexpr uint32_t SRAM_BASE             = 0x20000000; /*!< SRAM= 64KB; base address in the alias region                                  */
constexpr uint32_t PERIPH_BASE           = 0x40000000; /*!< Peripheral base address in the alias region                                  */
constexpr uint32_t FMC_R_BASE            = 0xA0000000; /*!< FMC registers base address                                                   */

constexpr uint32_t CCMDATARAM_BB_BASE    = 0x12000000; /*!< CCM= core coupled memory; data RAM= 16 KB; base address in the bit-band region */
constexpr uint32_t SRAM_BB_BASE          = 0x22000000; /*!< SRAM= 64KB; base address in the bit-band region                               */
constexpr uint32_t PERIPH_BB_BASE        = 0x42000000; /*!< Peripheral base address in the bit-band region                               */


/*!< Peripheral memory map */
constexpr uint32_t APB1PERIPH_BASE       = PERIPH_BASE;
constexpr uint32_t APB2PERIPH_BASE       = PERIPH_BASE + 0x00010000;
constexpr uint32_t AHB1PERIPH_BASE       = PERIPH_BASE + 0x00020000;
constexpr uint32_t AHB2PERIPH_BASE       = PERIPH_BASE + 0x08000000;
constexpr uint32_t AHB3PERIPH_BASE       = PERIPH_BASE + 0x10000000;

/*!< APB1 peripherals */
constexpr uint32_t TIM2_BASE             = APB1PERIPH_BASE + 0x00000000;
constexpr uint32_t TIM3_BASE             = APB1PERIPH_BASE + 0x00000400;
constexpr uint32_t TIM4_BASE             = APB1PERIPH_BASE + 0x00000800;
constexpr uint32_t TIM6_BASE             = APB1PERIPH_BASE + 0x00001000;
constexpr uint32_t TIM7_BASE             = APB1PERIPH_BASE + 0x00001400;
constexpr uint32_t RTC_BASE              = APB1PERIPH_BASE + 0x00002800;
constexpr uint32_t WWDG_BASE             = APB1PERIPH_BASE + 0x00002C00;
constexpr uint32_t IWDG_BASE             = APB1PERIPH_BASE + 0x00003000;
constexpr uint32_t I2S2ext_BASE          = APB1PERIPH_BASE + 0x00003400;
constexpr uint32_t SPI2_BASE             = APB1PERIPH_BASE + 0x00003800;
constexpr uint32_t SPI3_BASE             = APB1PERIPH_BASE + 0x00003C00;
constexpr uint32_t I2S3ext_BASE          = APB1PERIPH_BASE + 0x00004000;
constexpr uint32_t USART2_BASE           = APB1PERIPH_BASE + 0x00004400;
constexpr uint32_t USART3_BASE           = APB1PERIPH_BASE + 0x00004800;
constexpr uint32_t UART4_BASE            = APB1PERIPH_BASE + 0x00004C00;
constexpr uint32_t UART5_BASE            = APB1PERIPH_BASE + 0x00005000;
constexpr uint32_t I2C1_BASE             = APB1PERIPH_BASE + 0x00005400;
constexpr uint32_t I2C2_BASE             = APB1PERIPH_BASE + 0x00005800;
constexpr uint32_t USB_BASE              = APB1PERIPH_BASE + 0x00005C00; /*!< USB_IP Peripheral Registers base address */
constexpr uint32_t USB_PMAADDR           = APB1PERIPH_BASE + 0x00006000; /*!< USB_IP Packet Memory Area base address */
constexpr uint32_t CAN_BASE              = APB1PERIPH_BASE + 0x00006400;
constexpr uint32_t PWR_BASE              = APB1PERIPH_BASE + 0x00007000;
constexpr uint32_t DAC1_BASE             = APB1PERIPH_BASE + 0x00007400;
constexpr uint32_t DAC_BASE              = DAC1_BASE;
constexpr uint32_t I2C3_BASE             = APB1PERIPH_BASE + 0x00007800;

/*!< APB2 peripherals */
constexpr uint32_t SYSCFG_BASE           = APB2PERIPH_BASE + 0x00000000;
constexpr uint32_t COMP1_BASE            = APB2PERIPH_BASE + 0x0000001C;
constexpr uint32_t COMP2_BASE            = APB2PERIPH_BASE + 0x00000020;
constexpr uint32_t COMP3_BASE            = APB2PERIPH_BASE + 0x00000024;
constexpr uint32_t COMP4_BASE            = APB2PERIPH_BASE + 0x00000028;
constexpr uint32_t COMP5_BASE            = APB2PERIPH_BASE + 0x0000002C;
constexpr uint32_t COMP6_BASE            = APB2PERIPH_BASE + 0x00000030;
constexpr uint32_t COMP7_BASE            = APB2PERIPH_BASE + 0x00000034;
constexpr uint32_t COMP_BASE             = COMP1_BASE;
constexpr uint32_t OPAMP1_BASE           = APB2PERIPH_BASE + 0x00000038;
constexpr uint32_t OPAMP2_BASE           = APB2PERIPH_BASE + 0x0000003C;
constexpr uint32_t OPAMP3_BASE           = APB2PERIPH_BASE + 0x00000040;
constexpr uint32_t OPAMP4_BASE           = APB2PERIPH_BASE + 0x00000044;
constexpr uint32_t OPAMP_BASE            = OPAMP1_BASE;
constexpr uint32_t EXTI_BASE             = APB2PERIPH_BASE + 0x00000400;
constexpr uint32_t TIM1_BASE             = APB2PERIPH_BASE + 0x00002C00;
constexpr uint32_t SPI1_BASE             = APB2PERIPH_BASE + 0x00003000;
constexpr uint32_t TIM8_BASE             = APB2PERIPH_BASE + 0x00003400;
constexpr uint32_t USART1_BASE           = APB2PERIPH_BASE + 0x00003800;
constexpr uint32_t SPI4_BASE             = APB2PERIPH_BASE + 0x00003C00;
constexpr uint32_t TIM15_BASE            = APB2PERIPH_BASE + 0x00004000;
constexpr uint32_t TIM16_BASE            = APB2PERIPH_BASE + 0x00004400;
constexpr uint32_t TIM17_BASE            = APB2PERIPH_BASE + 0x00004800;
constexpr uint32_t TIM20_BASE            = APB2PERIPH_BASE + 0x00005000;

/*!< AHB1 peripherals */
constexpr uint32_t DMA1_BASE             = AHB1PERIPH_BASE + 0x00000000;
constexpr uint32_t DMA1_Channel1_BASE    = AHB1PERIPH_BASE + 0x00000008;
constexpr uint32_t DMA1_Channel2_BASE    = AHB1PERIPH_BASE + 0x0000001C;
constexpr uint32_t DMA1_Channel3_BASE    = AHB1PERIPH_BASE + 0x00000030;
constexpr uint32_t DMA1_Channel4_BASE    = AHB1PERIPH_BASE + 0x00000044;
constexpr uint32_t DMA1_Channel5_BASE    = AHB1PERIPH_BASE + 0x00000058;
constexpr uint32_t DMA1_Channel6_BASE    = AHB1PERIPH_BASE + 0x0000006C;
constexpr uint32_t DMA1_Channel7_BASE    = AHB1PERIPH_BASE + 0x00000080;
constexpr uint32_t DMA2_BASE             = AHB1PERIPH_BASE + 0x00000400;
constexpr uint32_t DMA2_Channel1_BASE    = AHB1PERIPH_BASE + 0x00000408;
constexpr uint32_t DMA2_Channel2_BASE    = AHB1PERIPH_BASE + 0x0000041C;
constexpr uint32_t DMA2_Channel3_BASE    = AHB1PERIPH_BASE + 0x00000430;
constexpr uint32_t DMA2_Channel4_BASE    = AHB1PERIPH_BASE + 0x00000444;
constexpr uint32_t DMA2_Channel5_BASE    = AHB1PERIPH_BASE + 0x00000458;
constexpr uint32_t RCC_BASE              = AHB1PERIPH_BASE + 0x00001000;
constexpr uint32_t FLASH_R_BASE          = AHB1PERIPH_BASE + 0x00002000; /*!< Flash registers base address */
constexpr uint32_t OB_BASE               = 0x1FFFF800;         /*!< Flash Option Bytes base address */
constexpr uint32_t CRC_BASE              = AHB1PERIPH_BASE + 0x00003000;
constexpr uint32_t TSC_BASE              = AHB1PERIPH_BASE + 0x00004000;

/*!< AHB2 peripherals */
constexpr uint32_t GPIOA_BASE            = AHB2PERIPH_BASE + 0x00000000;
constexpr uint32_t GPIOB_BASE            = AHB2PERIPH_BASE + 0x00000400;
constexpr uint32_t GPIOC_BASE            = AHB2PERIPH_BASE + 0x00000800;
constexpr uint32_t GPIOD_BASE            = AHB2PERIPH_BASE + 0x00000C00;
constexpr uint32_t GPIOE_BASE            = AHB2PERIPH_BASE + 0x00001000;
constexpr uint32_t GPIOF_BASE            = AHB2PERIPH_BASE + 0x00001400;
constexpr uint32_t GPIOG_BASE            = AHB2PERIPH_BASE + 0x00001800;
constexpr uint32_t GPIOH_BASE            = AHB2PERIPH_BASE + 0x00001C00;

/*!< AHB3 peripherals */
constexpr uint32_t ADC1_BASE             = AHB3PERIPH_BASE + 0x00000000;
constexpr uint32_t ADC2_BASE             = AHB3PERIPH_BASE + 0x00000100;
constexpr uint32_t ADC1_2_COMMON_BASE    = AHB3PERIPH_BASE + 0x00000300;
constexpr uint32_t ADC3_BASE             = AHB3PERIPH_BASE + 0x00000400;
constexpr uint32_t ADC4_BASE             = AHB3PERIPH_BASE + 0x00000500;
constexpr uint32_t ADC3_4_COMMON_BASE    = AHB3PERIPH_BASE + 0x00000700;

/*!< FMC Bankx registers base address */
constexpr uint32_t FMC_Bank1_R_BASE      = FMC_R_BASE + 0x0000;
constexpr uint32_t FMC_Bank1E_R_BASE     = FMC_R_BASE + 0x0104;
constexpr uint32_t FMC_Bank2_3_R_BASE    = FMC_R_BASE + 0x0060;
constexpr uint32_t FMC_Bank4_R_BASE      = FMC_R_BASE + 0x00A0;

constexpr uint32_t DBGMCU_BASE           = 0xE0042000; /*!< Debug MCU registers base address */


/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter SAR = ADC;               */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for ADC_ISR register  ********************/
constexpr uint32_t ADC_ISR_ADRD          = 0x00000001; /*!< ADC Ready = ADRDY; flag  */
constexpr uint32_t ADC_ISR_EOSMP         = 0x00000002; /*!< ADC End of Sampling flag */
constexpr uint32_t ADC_ISR_EOC           = 0x00000004; /*!< ADC End of Regular Conversion flag */
constexpr uint32_t ADC_ISR_EOS           = 0x00000008; /*!< ADC End of Regular sequence of Conversions flag */
constexpr uint32_t ADC_ISR_OVR           = 0x00000010; /*!< ADC overrun flag */
constexpr uint32_t ADC_ISR_JEOC          = 0x00000020; /*!< ADC End of Injected Conversion flag */
constexpr uint32_t ADC_ISR_JEOS          = 0x00000040; /*!< ADC End of Injected sequence of Conversions flag */
constexpr uint32_t ADC_ISR_AWD1          = 0x00000080; /*!< ADC Analog watchdog 1 flag */
constexpr uint32_t ADC_ISR_AWD2          = 0x00000100; /*!< ADC Analog watchdog 2 flag */
constexpr uint32_t ADC_ISR_AWD3          = 0x00000200; /*!< ADC Analog watchdog 3 flag */
constexpr uint32_t ADC_ISR_JQOVF         = 0x00000400; /*!< ADC Injected Context Queue Overflow flag */

/********************  Bit definition for ADC_IER register  ********************/
constexpr uint32_t ADC_IER_RDY           = 0x00000001; /*!< ADC Ready = ADRDY; interrupt source */
constexpr uint32_t ADC_IER_EOSMP         = 0x00000002; /*!< ADC End of Sampling interrupt source */
constexpr uint32_t ADC_IER_EOC           = 0x00000004; /*!< ADC End of Regular Conversion interrupt source */
constexpr uint32_t ADC_IER_EOS           = 0x00000008; /*!< ADC End of Regular sequence of Conversions interrupt source */
constexpr uint32_t ADC_IER_OVR           = 0x00000010; /*!< ADC overrun interrupt source */
constexpr uint32_t ADC_IER_JEOC          = 0x00000020; /*!< ADC End of Injected Conversion interrupt source */
constexpr uint32_t ADC_IER_JEOS          = 0x00000040; /*!< ADC End of Injected sequence of Conversions interrupt source */
constexpr uint32_t ADC_IER_AWD1          = 0x00000080; /*!< ADC Analog watchdog 1 interrupt source */
constexpr uint32_t ADC_IER_AWD2          = 0x00000100; /*!< ADC Analog watchdog 2 interrupt source */
constexpr uint32_t ADC_IER_AWD3          = 0x00000200; /*!< ADC Analog watchdog 3 interrupt source */
constexpr uint32_t ADC_IER_JQOVF         = 0x00000400; /*!< ADC Injected Context Queue Overflow interrupt source */

/********************  Bit definition for ADC_CR register  ********************/
constexpr uint32_t ADC_CR_ADEN          = 0x00000001; /*!< ADC Enable control */
constexpr uint32_t ADC_CR_ADDIS         = 0x00000002; /*!< ADC Disable command */
constexpr uint32_t ADC_CR_ADSTART       = 0x00000004; /*!< ADC Start of Regular conversion */
constexpr uint32_t ADC_CR_JADSTART      = 0x00000008; /*!< ADC Start of injected conversion */
constexpr uint32_t ADC_CR_ADSTP         = 0x00000010; /*!< ADC Stop of Regular conversion */
constexpr uint32_t ADC_CR_JADSTP        = 0x00000020; /*!< ADC Stop of injected conversion */
constexpr uint32_t ADC_CR_ADVREGEN      = 0x30000000; /*!< ADC Voltage regulator Enable */
constexpr uint32_t ADC_CR_ADVREGEN_0    = 0x10000000; /*!< ADC ADVREGEN bit 0 */
constexpr uint32_t ADC_CR_ADVREGEN_1    = 0x20000000; /*!< ADC ADVREGEN bit 1 */
constexpr uint32_t ADC_CR_ADCALDIF      = 0x40000000; /*!< ADC Differential Mode for calibration */
constexpr uint32_t ADC_CR_ADCAL         = 0x80000000; /*!< ADC Calibration */

/********************  Bit definition for ADC_CFGR register  ********************/
constexpr uint32_t ADC_CFGR_DMAEN     = 0x00000001; /*!< ADC DMA Enable */
constexpr uint32_t ADC_CFGR_DMACFG    = 0x00000002; /*!< ADC DMA configuration */

constexpr uint32_t ADC_CFGR_RES       = 0x00000018; /*!< ADC Data resolution */
constexpr uint32_t ADC_CFGR_RES_0     = 0x00000008; /*!< ADC RES bit 0 */
constexpr uint32_t ADC_CFGR_RES_1     = 0x00000010; /*!< ADC RES bit 1 */

constexpr uint32_t ADC_CFGR_ALIGN     = 0x00000020; /*!< ADC Data Alignement */

constexpr uint32_t ADC_CFGR_EXTSEL   = 0x000003C0; /*!< ADC External trigger selection for regular group */
constexpr uint32_t ADC_CFGR_EXTSEL_0 = 0x00000040; /*!< ADC EXTSEL bit 0 */
constexpr uint32_t ADC_CFGR_EXTSEL_1 = 0x00000080; /*!< ADC EXTSEL bit 1 */
constexpr uint32_t ADC_CFGR_EXTSEL_2 = 0x00000100; /*!< ADC EXTSEL bit 2 */
constexpr uint32_t ADC_CFGR_EXTSEL_3 = 0x00000200; /*!< ADC EXTSEL bit 3 */

constexpr uint32_t ADC_CFGR_EXTEN     = 0x00000C00; /*!< ADC External trigger enable and polarity selection for regular channels */
constexpr uint32_t ADC_CFGR_EXTEN_0   = 0x00000400; /*!< ADC EXTEN bit 0 */
constexpr uint32_t ADC_CFGR_EXTEN_1   = 0x00000800; /*!< ADC EXTEN bit 1 */

constexpr uint32_t ADC_CFGR_OVRMOD    = 0x00001000; /*!< ADC overrun mode */
constexpr uint32_t ADC_CFGR_CONT      = 0x00002000; /*!< ADC Single/continuous conversion mode for regular conversion */
constexpr uint32_t ADC_CFGR_AUTDLY   = 0x00004000; /*!< ADC Delayed conversion mode */
constexpr uint32_t ADC_CFGR_AUTOFF    = 0x00008000; /*!< ADC Auto power OFF */
constexpr uint32_t ADC_CFGR_DISCEN    = 0x00010000; /*!< ADC Discontinuous mode for regular channels */

constexpr uint32_t ADC_CFGR_DISCNUM   = 0x000E0000; /*!< ADC Discontinuous mode channel count */
constexpr uint32_t ADC_CFGR_DISCNUM_0 = 0x00020000; /*!< ADC DISCNUM bit 0 */
constexpr uint32_t ADC_CFGR_DISCNUM_1 = 0x00040000; /*!< ADC DISCNUM bit 1 */
constexpr uint32_t ADC_CFGR_DISCNUM_2 = 0x00080000; /*!< ADC DISCNUM bit 2 */

constexpr uint32_t ADC_CFGR_JDISCEN   = 0x00100000; /*!< ADC Discontinous mode on injected channels */
constexpr uint32_t ADC_CFGR_JQM       = 0x00200000; /*!< ADC JSQR Queue mode */
constexpr uint32_t ADC_CFGR_AWD1SGL   = 0x00400000; /*!< Eanble the watchdog 1 on a single channel or on all channels */
constexpr uint32_t ADC_CFGR_AWD1EN    = 0x00800000; /*!< ADC Analog watchdog 1 enable on regular Channels */
constexpr uint32_t ADC_CFGR_JAWD1EN   = 0x01000000; /*!< ADC Analog watchdog 1 enable on injected Channels */
constexpr uint32_t ADC_CFGR_JAUTO     = 0x02000000; /*!< ADC Automatic injected group conversion */

constexpr uint32_t ADC_CFGR_AWD1CH    = 0x7C000000; /*!< ADC Analog watchdog 1 Channel selection */
constexpr uint32_t ADC_CFGR_AWD1CH_0  = 0x04000000; /*!< ADC AWD1CH bit 0 */
constexpr uint32_t ADC_CFGR_AWD1CH_1  = 0x08000000; /*!< ADC AWD1CH bit 1  */
constexpr uint32_t ADC_CFGR_AWD1CH_2  = 0x10000000; /*!< ADC AWD1CH bit 2  */
constexpr uint32_t ADC_CFGR_AWD1CH_3  = 0x20000000; /*!< ADC AWD1CH bit 3  */
constexpr uint32_t ADC_CFGR_AWD1CH_4  = 0x40000000; /*!< ADC AWD1CH bit 4  */

/********************  Bit definition for ADC_SMPR1 register  ********************/
constexpr uint32_t ADC_SMPR1_SMP0     = 0x00000007; /*!< ADC Channel 0 Sampling time selection  */
constexpr uint32_t ADC_SMPR1_SMP0_0   = 0x00000001; /*!< ADC SMP0 bit 0 */
constexpr uint32_t ADC_SMPR1_SMP0_1   = 0x00000002; /*!< ADC SMP0 bit 1 */
constexpr uint32_t ADC_SMPR1_SMP0_2   = 0x00000004; /*!< ADC SMP0 bit 2 */

constexpr uint32_t ADC_SMPR1_SMP1     = 0x00000038; /*!< ADC Channel 1 Sampling time selection  */
constexpr uint32_t ADC_SMPR1_SMP1_0   = 0x00000008; /*!< ADC SMP1 bit 0 */
constexpr uint32_t ADC_SMPR1_SMP1_1   = 0x00000010; /*!< ADC SMP1 bit 1 */
constexpr uint32_t ADC_SMPR1_SMP1_2   = 0x00000020; /*!< ADC SMP1 bit 2 */

constexpr uint32_t ADC_SMPR1_SMP2     = 0x000001C0; /*!< ADC Channel 2 Sampling time selection  */
constexpr uint32_t ADC_SMPR1_SMP2_0   = 0x00000040; /*!< ADC SMP2 bit 0 */
constexpr uint32_t ADC_SMPR1_SMP2_1   = 0x00000080; /*!< ADC SMP2 bit 1 */
constexpr uint32_t ADC_SMPR1_SMP2_2   = 0x00000100; /*!< ADC SMP2 bit 2 */

constexpr uint32_t ADC_SMPR1_SMP3     = 0x00000E00; /*!< ADC Channel 3 Sampling time selection  */
constexpr uint32_t ADC_SMPR1_SMP3_0   = 0x00000200; /*!< ADC SMP3 bit 0 */
constexpr uint32_t ADC_SMPR1_SMP3_1   = 0x00000400; /*!< ADC SMP3 bit 1 */
constexpr uint32_t ADC_SMPR1_SMP3_2   = 0x00000800; /*!< ADC SMP3 bit 2 */

constexpr uint32_t ADC_SMPR1_SMP4     = 0x00007000; /*!< ADC Channel 4 Sampling time selection  */
constexpr uint32_t ADC_SMPR1_SMP4_0   = 0x00001000; /*!< ADC SMP4 bit 0 */
constexpr uint32_t ADC_SMPR1_SMP4_1   = 0x00002000; /*!< ADC SMP4 bit 1 */
constexpr uint32_t ADC_SMPR1_SMP4_2   = 0x00004000; /*!< ADC SMP4 bit 2 */

constexpr uint32_t ADC_SMPR1_SMP5     = 0x00038000; /*!< ADC Channel 5 Sampling time selection  */
constexpr uint32_t ADC_SMPR1_SMP5_0   = 0x00008000; /*!< ADC SMP5 bit 0 */
constexpr uint32_t ADC_SMPR1_SMP5_1   = 0x00010000; /*!< ADC SMP5 bit 1 */
constexpr uint32_t ADC_SMPR1_SMP5_2   = 0x00020000; /*!< ADC SMP5 bit 2 */

constexpr uint32_t ADC_SMPR1_SMP6     = 0x001C0000; /*!< ADC Channel 6 Sampling time selection  */
constexpr uint32_t ADC_SMPR1_SMP6_0   = 0x00040000; /*!< ADC SMP6 bit 0 */
constexpr uint32_t ADC_SMPR1_SMP6_1   = 0x00080000; /*!< ADC SMP6 bit 1 */
constexpr uint32_t ADC_SMPR1_SMP6_2   = 0x00100000; /*!< ADC SMP6 bit 2 */

constexpr uint32_t ADC_SMPR1_SMP7     = 0x00E00000; /*!< ADC Channel 7 Sampling time selection  */
constexpr uint32_t ADC_SMPR1_SMP7_0   = 0x00200000; /*!< ADC SMP7 bit 0 */
constexpr uint32_t ADC_SMPR1_SMP7_1   = 0x00400000; /*!< ADC SMP7 bit 1 */
constexpr uint32_t ADC_SMPR1_SMP7_2   = 0x00800000; /*!< ADC SMP7 bit 2 */

constexpr uint32_t ADC_SMPR1_SMP8     = 0x07000000; /*!< ADC Channel 8 Sampling time selection  */
constexpr uint32_t ADC_SMPR1_SMP8_0   = 0x01000000; /*!< ADC SMP8 bit 0 */
constexpr uint32_t ADC_SMPR1_SMP8_1   = 0x02000000; /*!< ADC SMP8 bit 1 */
constexpr uint32_t ADC_SMPR1_SMP8_2   = 0x04000000; /*!< ADC SMP8 bit 2 */

constexpr uint32_t ADC_SMPR1_SMP9     = 0x38000000; /*!< ADC Channel 9 Sampling time selection  */
constexpr uint32_t ADC_SMPR1_SMP9_0   = 0x08000000; /*!< ADC SMP9 bit 0 */
constexpr uint32_t ADC_SMPR1_SMP9_1   = 0x10000000; /*!< ADC SMP9 bit 1 */
constexpr uint32_t ADC_SMPR1_SMP9_2   = 0x20000000; /*!< ADC SMP9 bit 2 */

/********************  Bit definition for ADC_SMPR2 register  ********************/
constexpr uint32_t ADC_SMPR2_SMP10     = 0x00000007; /*!< ADC Channel 10 Sampling time selection  */
constexpr uint32_t ADC_SMPR2_SMP10_0   = 0x00000001; /*!< ADC SMP10 bit 0 */
constexpr uint32_t ADC_SMPR2_SMP10_1   = 0x00000002; /*!< ADC SMP10 bit 1 */
constexpr uint32_t ADC_SMPR2_SMP10_2   = 0x00000004; /*!< ADC SMP10 bit 2 */

constexpr uint32_t ADC_SMPR2_SMP11     = 0x00000038; /*!< ADC Channel 11 Sampling time selection  */
constexpr uint32_t ADC_SMPR2_SMP11_0   = 0x00000008; /*!< ADC SMP11 bit 0 */
constexpr uint32_t ADC_SMPR2_SMP11_1   = 0x00000010; /*!< ADC SMP11 bit 1 */
constexpr uint32_t ADC_SMPR2_SMP11_2   = 0x00000020; /*!< ADC SMP11 bit 2 */

constexpr uint32_t ADC_SMPR2_SMP12     = 0x000001C0; /*!< ADC Channel 12 Sampling time selection  */
constexpr uint32_t ADC_SMPR2_SMP12_0   = 0x00000040; /*!< ADC SMP12 bit 0 */
constexpr uint32_t ADC_SMPR2_SMP12_1   = 0x00000080; /*!< ADC SMP12 bit 1 */
constexpr uint32_t ADC_SMPR2_SMP12_2   = 0x00000100; /*!< ADC SMP12 bit 2 */

constexpr uint32_t ADC_SMPR2_SMP13     = 0x00000E00; /*!< ADC Channel 13 Sampling time selection  */
constexpr uint32_t ADC_SMPR2_SMP13_0   = 0x00000200; /*!< ADC SMP13 bit 0 */
constexpr uint32_t ADC_SMPR2_SMP13_1   = 0x00000400; /*!< ADC SMP13 bit 1 */
constexpr uint32_t ADC_SMPR2_SMP13_2   = 0x00000800; /*!< ADC SMP13 bit 2 */

constexpr uint32_t ADC_SMPR2_SMP14     = 0x00007000; /*!< ADC Channel 14 Sampling time selection  */
constexpr uint32_t ADC_SMPR2_SMP14_0   = 0x00001000; /*!< ADC SMP14 bit 0 */
constexpr uint32_t ADC_SMPR2_SMP14_1   = 0x00002000; /*!< ADC SMP14 bit 1 */
constexpr uint32_t ADC_SMPR2_SMP14_2   = 0x00004000; /*!< ADC SMP14 bit 2 */

constexpr uint32_t ADC_SMPR2_SMP15     = 0x00038000; /*!< ADC Channel 15 Sampling time selection  */
constexpr uint32_t ADC_SMPR2_SMP15_0   = 0x00008000; /*!< ADC SMP15 bit 0 */
constexpr uint32_t ADC_SMPR2_SMP15_1   = 0x00010000; /*!< ADC SMP15 bit 1 */
constexpr uint32_t ADC_SMPR2_SMP15_2   = 0x00020000; /*!< ADC SMP15 bit 2 */

constexpr uint32_t ADC_SMPR2_SMP16     = 0x001C0000; /*!< ADC Channel 16 Sampling time selection  */
constexpr uint32_t ADC_SMPR2_SMP16_0   = 0x00040000; /*!< ADC SMP16 bit 0 */
constexpr uint32_t ADC_SMPR2_SMP16_1   = 0x00080000; /*!< ADC SMP16 bit 1 */
constexpr uint32_t ADC_SMPR2_SMP16_2   = 0x00100000; /*!< ADC SMP16 bit 2 */

constexpr uint32_t ADC_SMPR2_SMP17     = 0x00E00000; /*!< ADC Channel 17 Sampling time selection  */
constexpr uint32_t ADC_SMPR2_SMP17_0   = 0x00200000; /*!< ADC SMP17 bit 0 */
constexpr uint32_t ADC_SMPR2_SMP17_1   = 0x00400000; /*!< ADC SMP17 bit 1 */
constexpr uint32_t ADC_SMPR2_SMP17_2   = 0x00800000; /*!< ADC SMP17 bit 2 */

constexpr uint32_t ADC_SMPR2_SMP18     = 0x07000000; /*!< ADC Channel 18 Sampling time selection  */
constexpr uint32_t ADC_SMPR2_SMP18_0   = 0x01000000; /*!< ADC SMP18 bit 0 */
constexpr uint32_t ADC_SMPR2_SMP18_1   = 0x02000000; /*!< ADC SMP18 bit 1 */
constexpr uint32_t ADC_SMPR2_SMP18_2   = 0x04000000; /*!< ADC SMP18 bit 2 */

/********************  Bit definition for ADC_TR1 register  ********************/
constexpr uint32_t ADC_TR1_LT1         = 0x00000FFF; /*!< ADC Analog watchdog 1 lower threshold */
constexpr uint32_t ADC_TR1_LT1_0       = 0x00000001; /*!< ADC LT1 bit 0 */
constexpr uint32_t ADC_TR1_LT1_1       = 0x00000002; /*!< ADC LT1 bit 1 */
constexpr uint32_t ADC_TR1_LT1_2       = 0x00000004; /*!< ADC LT1 bit 2 */
constexpr uint32_t ADC_TR1_LT1_3       = 0x00000008; /*!< ADC LT1 bit 3 */
constexpr uint32_t ADC_TR1_LT1_4       = 0x00000010; /*!< ADC LT1 bit 4 */
constexpr uint32_t ADC_TR1_LT1_5       = 0x00000020; /*!< ADC LT1 bit 5 */
constexpr uint32_t ADC_TR1_LT1_6       = 0x00000040; /*!< ADC LT1 bit 6 */
constexpr uint32_t ADC_TR1_LT1_7       = 0x00000080; /*!< ADC LT1 bit 7 */
constexpr uint32_t ADC_TR1_LT1_8       = 0x00000100; /*!< ADC LT1 bit 8 */
constexpr uint32_t ADC_TR1_LT1_9       = 0x00000200; /*!< ADC LT1 bit 9 */
constexpr uint32_t ADC_TR1_LT1_10      = 0x00000400; /*!< ADC LT1 bit 10 */
constexpr uint32_t ADC_TR1_LT1_11      = 0x00000800; /*!< ADC LT1 bit 11 */

constexpr uint32_t ADC_TR1_HT1         = 0x0FFF0000; /*!< ADC Analog watchdog 1 higher threshold */
constexpr uint32_t ADC_TR1_HT1_0       = 0x00010000; /*!< ADC HT1 bit 0 */
constexpr uint32_t ADC_TR1_HT1_1       = 0x00020000; /*!< ADC HT1 bit 1 */
constexpr uint32_t ADC_TR1_HT1_2       = 0x00040000; /*!< ADC HT1 bit 2 */
constexpr uint32_t ADC_TR1_HT1_3       = 0x00080000; /*!< ADC HT1 bit 3 */
constexpr uint32_t ADC_TR1_HT1_4       = 0x00100000; /*!< ADC HT1 bit 4 */
constexpr uint32_t ADC_TR1_HT1_5       = 0x00200000; /*!< ADC HT1 bit 5 */
constexpr uint32_t ADC_TR1_HT1_6       = 0x00400000; /*!< ADC HT1 bit 6 */
constexpr uint32_t ADC_TR1_HT1_7       = 0x00800000; /*!< ADC HT1 bit 7 */
constexpr uint32_t ADC_TR1_HT1_8       = 0x01000000; /*!< ADC HT1 bit 8 */
constexpr uint32_t ADC_TR1_HT1_9       = 0x02000000; /*!< ADC HT1 bit 9 */
constexpr uint32_t ADC_TR1_HT1_10      = 0x04000000; /*!< ADC HT1 bit 10 */
constexpr uint32_t ADC_TR1_HT1_11      = 0x08000000; /*!< ADC HT1 bit 11 */

/********************  Bit definition for ADC_TR2 register  ********************/
constexpr uint32_t ADC_TR2_LT2         = 0x000000FF; /*!< ADC Analog watchdog 2 lower threshold */
constexpr uint32_t ADC_TR2_LT2_0       = 0x00000001; /*!< ADC LT2 bit 0 */
constexpr uint32_t ADC_TR2_LT2_1       = 0x00000002; /*!< ADC LT2 bit 1 */
constexpr uint32_t ADC_TR2_LT2_2       = 0x00000004; /*!< ADC LT2 bit 2 */
constexpr uint32_t ADC_TR2_LT2_3       = 0x00000008; /*!< ADC LT2 bit 3 */
constexpr uint32_t ADC_TR2_LT2_4       = 0x00000010; /*!< ADC LT2 bit 4 */
constexpr uint32_t ADC_TR2_LT2_5       = 0x00000020; /*!< ADC LT2 bit 5 */
constexpr uint32_t ADC_TR2_LT2_6       = 0x00000040; /*!< ADC LT2 bit 6 */
constexpr uint32_t ADC_TR2_LT2_7       = 0x00000080; /*!< ADC LT2 bit 7 */

constexpr uint32_t ADC_TR2_HT2         = 0x00FF0000; /*!< ADC Analog watchdog 2 higher threshold */
constexpr uint32_t ADC_TR2_HT2_0       = 0x00010000; /*!< ADC HT2 bit 0 */
constexpr uint32_t ADC_TR2_HT2_1       = 0x00020000; /*!< ADC HT2 bit 1 */
constexpr uint32_t ADC_TR2_HT2_2       = 0x00040000; /*!< ADC HT2 bit 2 */
constexpr uint32_t ADC_TR2_HT2_3       = 0x00080000; /*!< ADC HT2 bit 3 */
constexpr uint32_t ADC_TR2_HT2_4       = 0x00100000; /*!< ADC HT2 bit 4 */
constexpr uint32_t ADC_TR2_HT2_5       = 0x00200000; /*!< ADC HT2 bit 5 */
constexpr uint32_t ADC_TR2_HT2_6       = 0x00400000; /*!< ADC HT2 bit 6 */
constexpr uint32_t ADC_TR2_HT2_7       = 0x00800000; /*!< ADC HT2 bit 7 */

/********************  Bit definition for ADC_TR3 register  ********************/
constexpr uint32_t ADC_TR3_LT3         = 0x000000FF; /*!< ADC Analog watchdog 3 lower threshold */
constexpr uint32_t ADC_TR3_LT3_0       = 0x00000001; /*!< ADC LT3 bit 0 */
constexpr uint32_t ADC_TR3_LT3_1       = 0x00000002; /*!< ADC LT3 bit 1 */
constexpr uint32_t ADC_TR3_LT3_2       = 0x00000004; /*!< ADC LT3 bit 2 */
constexpr uint32_t ADC_TR3_LT3_3       = 0x00000008; /*!< ADC LT3 bit 3 */
constexpr uint32_t ADC_TR3_LT3_4       = 0x00000010; /*!< ADC LT3 bit 4 */
constexpr uint32_t ADC_TR3_LT3_5       = 0x00000020; /*!< ADC LT3 bit 5 */
constexpr uint32_t ADC_TR3_LT3_6       = 0x00000040; /*!< ADC LT3 bit 6 */
constexpr uint32_t ADC_TR3_LT3_7       = 0x00000080; /*!< ADC LT3 bit 7 */

constexpr uint32_t ADC_TR3_HT3         = 0x00FF0000; /*!< ADC Analog watchdog 3 higher threshold */
constexpr uint32_t ADC_TR3_HT3_0       = 0x00010000; /*!< ADC HT3 bit 0 */
constexpr uint32_t ADC_TR3_HT3_1       = 0x00020000; /*!< ADC HT3 bit 1 */
constexpr uint32_t ADC_TR3_HT3_2       = 0x00040000; /*!< ADC HT3 bit 2 */
constexpr uint32_t ADC_TR3_HT3_3       = 0x00080000; /*!< ADC HT3 bit 3 */
constexpr uint32_t ADC_TR3_HT3_4       = 0x00100000; /*!< ADC HT3 bit 4 */
constexpr uint32_t ADC_TR3_HT3_5       = 0x00200000; /*!< ADC HT3 bit 5 */
constexpr uint32_t ADC_TR3_HT3_6       = 0x00400000; /*!< ADC HT3 bit 6 */
constexpr uint32_t ADC_TR3_HT3_7       = 0x00800000; /*!< ADC HT3 bit 7 */

/********************  Bit definition for ADC_SQR1 register  ********************/
constexpr uint32_t ADC_SQR1_L          = 0x0000000F; /*!< ADC regular channel sequence lenght */
constexpr uint32_t ADC_SQR1_L_0        = 0x00000001; /*!< ADC L bit 0 */
constexpr uint32_t ADC_SQR1_L_1        = 0x00000002; /*!< ADC L bit 1 */
constexpr uint32_t ADC_SQR1_L_2        = 0x00000004; /*!< ADC L bit 2 */
constexpr uint32_t ADC_SQR1_L_3        = 0x00000008; /*!< ADC L bit 3 */

constexpr uint32_t ADC_SQR1_SQ1        = 0x000007C0; /*!< ADC 1st conversion in regular sequence */
constexpr uint32_t ADC_SQR1_SQ1_0      = 0x00000040; /*!< ADC SQ1 bit 0 */
constexpr uint32_t ADC_SQR1_SQ1_1      = 0x00000080; /*!< ADC SQ1 bit 1 */
constexpr uint32_t ADC_SQR1_SQ1_2      = 0x00000100; /*!< ADC SQ1 bit 2 */
constexpr uint32_t ADC_SQR1_SQ1_3      = 0x00000200; /*!< ADC SQ1 bit 3 */
constexpr uint32_t ADC_SQR1_SQ1_4      = 0x00000400; /*!< ADC SQ1 bit 4 */

constexpr uint32_t ADC_SQR1_SQ2        = 0x0001F000; /*!< ADC 2nd conversion in regular sequence */
constexpr uint32_t ADC_SQR1_SQ2_0      = 0x00001000; /*!< ADC SQ2 bit 0 */
constexpr uint32_t ADC_SQR1_SQ2_1      = 0x00002000; /*!< ADC SQ2 bit 1 */
constexpr uint32_t ADC_SQR1_SQ2_2      = 0x00004000; /*!< ADC SQ2 bit 2 */
constexpr uint32_t ADC_SQR1_SQ2_3      = 0x00008000; /*!< ADC SQ2 bit 3 */
constexpr uint32_t ADC_SQR1_SQ2_4      = 0x00010000; /*!< ADC SQ2 bit 4 */

constexpr uint32_t ADC_SQR1_SQ3        = 0x007C0000; /*!< ADC 3rd conversion in regular sequence */
constexpr uint32_t ADC_SQR1_SQ3_0      = 0x00040000; /*!< ADC SQ3 bit 0 */
constexpr uint32_t ADC_SQR1_SQ3_1      = 0x00080000; /*!< ADC SQ3 bit 1 */
constexpr uint32_t ADC_SQR1_SQ3_2      = 0x00100000; /*!< ADC SQ3 bit 2 */
constexpr uint32_t ADC_SQR1_SQ3_3      = 0x00200000; /*!< ADC SQ3 bit 3 */
constexpr uint32_t ADC_SQR1_SQ3_4      = 0x00400000; /*!< ADC SQ3 bit 4 */

constexpr uint32_t ADC_SQR1_SQ4        = 0x1F000000; /*!< ADC 4th conversion in regular sequence */
constexpr uint32_t ADC_SQR1_SQ4_0      = 0x01000000; /*!< ADC SQ4 bit 0 */
constexpr uint32_t ADC_SQR1_SQ4_1      = 0x02000000; /*!< ADC SQ4 bit 1 */
constexpr uint32_t ADC_SQR1_SQ4_2      = 0x04000000; /*!< ADC SQ4 bit 2 */
constexpr uint32_t ADC_SQR1_SQ4_3      = 0x08000000; /*!< ADC SQ4 bit 3 */
constexpr uint32_t ADC_SQR1_SQ4_4      = 0x10000000; /*!< ADC SQ4 bit 4 */

/********************  Bit definition for ADC_SQR2 register  ********************/
constexpr uint32_t ADC_SQR2_SQ5        = 0x0000001F; /*!< ADC 5th conversion in regular sequence */
constexpr uint32_t ADC_SQR2_SQ5_0      = 0x00000001; /*!< ADC SQ5 bit 0 */
constexpr uint32_t ADC_SQR2_SQ5_1      = 0x00000002; /*!< ADC SQ5 bit 1 */
constexpr uint32_t ADC_SQR2_SQ5_2      = 0x00000004; /*!< ADC SQ5 bit 2 */
constexpr uint32_t ADC_SQR2_SQ5_3      = 0x00000008; /*!< ADC SQ5 bit 3 */
constexpr uint32_t ADC_SQR2_SQ5_4      = 0x00000010; /*!< ADC SQ5 bit 4 */

constexpr uint32_t ADC_SQR2_SQ6        = 0x000007C0; /*!< ADC 6th conversion in regular sequence */
constexpr uint32_t ADC_SQR2_SQ6_0      = 0x00000040; /*!< ADC SQ6 bit 0 */
constexpr uint32_t ADC_SQR2_SQ6_1      = 0x00000080; /*!< ADC SQ6 bit 1 */
constexpr uint32_t ADC_SQR2_SQ6_2      = 0x00000100; /*!< ADC SQ6 bit 2 */
constexpr uint32_t ADC_SQR2_SQ6_3      = 0x00000200; /*!< ADC SQ6 bit 3 */
constexpr uint32_t ADC_SQR2_SQ6_4      = 0x00000400; /*!< ADC SQ6 bit 4 */

constexpr uint32_t ADC_SQR2_SQ7        = 0x0001F000; /*!< ADC 7th conversion in regular sequence */
constexpr uint32_t ADC_SQR2_SQ7_0      = 0x00001000; /*!< ADC SQ7 bit 0 */
constexpr uint32_t ADC_SQR2_SQ7_1      = 0x00002000; /*!< ADC SQ7 bit 1 */
constexpr uint32_t ADC_SQR2_SQ7_2      = 0x00004000; /*!< ADC SQ7 bit 2 */
constexpr uint32_t ADC_SQR2_SQ7_3      = 0x00008000; /*!< ADC SQ7 bit 3 */
constexpr uint32_t ADC_SQR2_SQ7_4      = 0x00010000; /*!< ADC SQ7 bit 4 */

constexpr uint32_t ADC_SQR2_SQ8        = 0x007C0000; /*!< ADC 8th conversion in regular sequence */
constexpr uint32_t ADC_SQR2_SQ8_0      = 0x00040000; /*!< ADC SQ8 bit 0 */
constexpr uint32_t ADC_SQR2_SQ8_1      = 0x00080000; /*!< ADC SQ8 bit 1 */
constexpr uint32_t ADC_SQR2_SQ8_2      = 0x00100000; /*!< ADC SQ8 bit 2 */
constexpr uint32_t ADC_SQR2_SQ8_3      = 0x00200000; /*!< ADC SQ8 bit 3 */
constexpr uint32_t ADC_SQR2_SQ8_4      = 0x00400000; /*!< ADC SQ8 bit 4 */

constexpr uint32_t ADC_SQR2_SQ9        = 0x1F000000; /*!< ADC 9th conversion in regular sequence */
constexpr uint32_t ADC_SQR2_SQ9_0      = 0x01000000; /*!< ADC SQ9 bit 0 */
constexpr uint32_t ADC_SQR2_SQ9_1      = 0x02000000; /*!< ADC SQ9 bit 1 */
constexpr uint32_t ADC_SQR2_SQ9_2      = 0x04000000; /*!< ADC SQ9 bit 2 */
constexpr uint32_t ADC_SQR2_SQ9_3      = 0x08000000; /*!< ADC SQ9 bit 3 */
constexpr uint32_t ADC_SQR2_SQ9_4      = 0x10000000; /*!< ADC SQ9 bit 4 */

/********************  Bit definition for ADC_SQR3 register  ********************/
constexpr uint32_t ADC_SQR3_SQ10       = 0x0000001F; /*!< ADC 10th conversion in regular sequence */
constexpr uint32_t ADC_SQR3_SQ10_0     = 0x00000001; /*!< ADC SQ10 bit 0 */
constexpr uint32_t ADC_SQR3_SQ10_1     = 0x00000002; /*!< ADC SQ10 bit 1 */
constexpr uint32_t ADC_SQR3_SQ10_2     = 0x00000004; /*!< ADC SQ10 bit 2 */
constexpr uint32_t ADC_SQR3_SQ10_3     = 0x00000008; /*!< ADC SQ10 bit 3 */
constexpr uint32_t ADC_SQR3_SQ10_4     = 0x00000010; /*!< ADC SQ10 bit 4 */

constexpr uint32_t ADC_SQR3_SQ11       = 0x000007C0; /*!< ADC 11th conversion in regular sequence */
constexpr uint32_t ADC_SQR3_SQ11_0     = 0x00000040; /*!< ADC SQ11 bit 0 */
constexpr uint32_t ADC_SQR3_SQ11_1     = 0x00000080; /*!< ADC SQ11 bit 1 */
constexpr uint32_t ADC_SQR3_SQ11_2     = 0x00000100; /*!< ADC SQ11 bit 2 */
constexpr uint32_t ADC_SQR3_SQ11_3     = 0x00000200; /*!< ADC SQ11 bit 3 */
constexpr uint32_t ADC_SQR3_SQ11_4     = 0x00000400; /*!< ADC SQ11 bit 4 */

constexpr uint32_t ADC_SQR3_SQ12       = 0x0001F000; /*!< ADC 12th conversion in regular sequence */
constexpr uint32_t ADC_SQR3_SQ12_0     = 0x00001000; /*!< ADC SQ12 bit 0 */
constexpr uint32_t ADC_SQR3_SQ12_1     = 0x00002000; /*!< ADC SQ12 bit 1 */
constexpr uint32_t ADC_SQR3_SQ12_2     = 0x00004000; /*!< ADC SQ12 bit 2 */
constexpr uint32_t ADC_SQR3_SQ12_3     = 0x00008000; /*!< ADC SQ12 bit 3 */
constexpr uint32_t ADC_SQR3_SQ12_4     = 0x00010000; /*!< ADC SQ12 bit 4 */

constexpr uint32_t ADC_SQR3_SQ13       = 0x007C0000; /*!< ADC 13th conversion in regular sequence */
constexpr uint32_t ADC_SQR3_SQ13_0     = 0x00040000; /*!< ADC SQ13 bit 0 */
constexpr uint32_t ADC_SQR3_SQ13_1     = 0x00080000; /*!< ADC SQ13 bit 1 */
constexpr uint32_t ADC_SQR3_SQ13_2     = 0x00100000; /*!< ADC SQ13 bit 2 */
constexpr uint32_t ADC_SQR3_SQ13_3     = 0x00200000; /*!< ADC SQ13 bit 3 */
constexpr uint32_t ADC_SQR3_SQ13_4     = 0x00400000; /*!< ADC SQ13 bit 4 */

constexpr uint32_t ADC_SQR3_SQ14       = 0x1F000000; /*!< ADC 14th conversion in regular sequence */
constexpr uint32_t ADC_SQR3_SQ14_0     = 0x01000000; /*!< ADC SQ14 bit 0 */
constexpr uint32_t ADC_SQR3_SQ14_1     = 0x02000000; /*!< ADC SQ14 bit 1 */
constexpr uint32_t ADC_SQR3_SQ14_2     = 0x04000000; /*!< ADC SQ14 bit 2 */
constexpr uint32_t ADC_SQR3_SQ14_3     = 0x08000000; /*!< ADC SQ14 bit 3 */
constexpr uint32_t ADC_SQR3_SQ14_4     = 0x10000000; /*!< ADC SQ14 bit 4 */

/********************  Bit definition for ADC_SQR4 register  ********************/
constexpr uint32_t ADC_SQR4_SQ15       = 0x0000001F; /*!< ADC 15th conversion in regular sequence */
constexpr uint32_t ADC_SQR4_SQ15_0     = 0x00000001; /*!< ADC SQ15 bit 0 */
constexpr uint32_t ADC_SQR4_SQ15_1     = 0x00000002; /*!< ADC SQ15 bit 1 */
constexpr uint32_t ADC_SQR4_SQ15_2     = 0x00000004; /*!< ADC SQ15 bit 2 */
constexpr uint32_t ADC_SQR4_SQ15_3     = 0x00000008; /*!< ADC SQ15 bit 3 */
constexpr uint32_t ADC_SQR4_SQ15_4     = 0x00000010; /*!< ADC SQ105 bit 4 */

constexpr uint32_t ADC_SQR4_SQ16       = 0x000007C0; /*!< ADC 16th conversion in regular sequence */
constexpr uint32_t ADC_SQR4_SQ16_0     = 0x00000040; /*!< ADC SQ16 bit 0 */
constexpr uint32_t ADC_SQR4_SQ16_1     = 0x00000080; /*!< ADC SQ16 bit 1 */
constexpr uint32_t ADC_SQR4_SQ16_2     = 0x00000100; /*!< ADC SQ16 bit 2 */
constexpr uint32_t ADC_SQR4_SQ16_3     = 0x00000200; /*!< ADC SQ16 bit 3 */
constexpr uint32_t ADC_SQR4_SQ16_4     = 0x00000400; /*!< ADC SQ16 bit 4 */
/********************  Bit definition for ADC_DR register  ********************/
constexpr uint32_t ADC_DR_RDATA        = 0x0000FFFF; /*!< ADC regular Data converted */
constexpr uint32_t ADC_DR_RDATA_0      = 0x00000001; /*!< ADC RDATA bit 0 */
constexpr uint32_t ADC_DR_RDATA_1      = 0x00000002; /*!< ADC RDATA bit 1 */
constexpr uint32_t ADC_DR_RDATA_2      = 0x00000004; /*!< ADC RDATA bit 2 */
constexpr uint32_t ADC_DR_RDATA_3      = 0x00000008; /*!< ADC RDATA bit 3 */
constexpr uint32_t ADC_DR_RDATA_4      = 0x00000010; /*!< ADC RDATA bit 4 */
constexpr uint32_t ADC_DR_RDATA_5      = 0x00000020; /*!< ADC RDATA bit 5 */
constexpr uint32_t ADC_DR_RDATA_6      = 0x00000040; /*!< ADC RDATA bit 6 */
constexpr uint32_t ADC_DR_RDATA_7      = 0x00000080; /*!< ADC RDATA bit 7 */
constexpr uint32_t ADC_DR_RDATA_8      = 0x00000100; /*!< ADC RDATA bit 8 */
constexpr uint32_t ADC_DR_RDATA_9      = 0x00000200; /*!< ADC RDATA bit 9 */
constexpr uint32_t ADC_DR_RDATA_10     = 0x00000400; /*!< ADC RDATA bit 10 */
constexpr uint32_t ADC_DR_RDATA_11     = 0x00000800; /*!< ADC RDATA bit 11 */
constexpr uint32_t ADC_DR_RDATA_12     = 0x00001000; /*!< ADC RDATA bit 12 */
constexpr uint32_t ADC_DR_RDATA_13     = 0x00002000; /*!< ADC RDATA bit 13 */
constexpr uint32_t ADC_DR_RDATA_14     = 0x00004000; /*!< ADC RDATA bit 14 */
constexpr uint32_t ADC_DR_RDATA_15     = 0x00008000; /*!< ADC RDATA bit 15 */

/********************  Bit definition for ADC_JSQR register  ********************/
constexpr uint32_t ADC_JSQR_JL         = 0x00000003; /*!< ADC injected channel sequence length */
constexpr uint32_t ADC_JSQR_JL_0       = 0x00000001; /*!< ADC JL bit 0 */
constexpr uint32_t ADC_JSQR_JL_1       = 0x00000002; /*!< ADC JL bit 1 */

constexpr uint32_t ADC_JSQR_JEXTSEL    = 0x0000003C; /*!< ADC external trigger selection for injected group */
constexpr uint32_t ADC_JSQR_JEXTSEL_0  = 0x00000004; /*!< ADC JEXTSEL bit 0 */
constexpr uint32_t ADC_JSQR_JEXTSEL_1  = 0x00000008; /*!< ADC JEXTSEL bit 1 */
constexpr uint32_t ADC_JSQR_JEXTSEL_2  = 0x00000010; /*!< ADC JEXTSEL bit 2 */
constexpr uint32_t ADC_JSQR_JEXTSEL_3  = 0x00000020; /*!< ADC JEXTSEL bit 3 */

constexpr uint32_t ADC_JSQR_JEXTEN     = 0x000000C0; /*!< ADC external trigger enable and polarity selection for injected channels */
constexpr uint32_t ADC_JSQR_JEXTEN_0   = 0x00000040; /*!< ADC JEXTEN bit 0 */
constexpr uint32_t ADC_JSQR_JEXTEN_1   = 0x00000080; /*!< ADC JEXTEN bit 1 */

constexpr uint32_t ADC_JSQR_JSQ1       = 0x00001F00; /*!< ADC 1st conversion in injected sequence */
constexpr uint32_t ADC_JSQR_JSQ1_0     = 0x00000100; /*!< ADC JSQ1 bit 0 */
constexpr uint32_t ADC_JSQR_JSQ1_1     = 0x00000200; /*!< ADC JSQ1 bit 1 */
constexpr uint32_t ADC_JSQR_JSQ1_2     = 0x00000400; /*!< ADC JSQ1 bit 2 */
constexpr uint32_t ADC_JSQR_JSQ1_3     = 0x00000800; /*!< ADC JSQ1 bit 3 */
constexpr uint32_t ADC_JSQR_JSQ1_4     = 0x00001000; /*!< ADC JSQ1 bit 4 */

constexpr uint32_t ADC_JSQR_JSQ2       = 0x0007C000; /*!< ADC 2nd conversion in injected sequence */
constexpr uint32_t ADC_JSQR_JSQ2_0     = 0x00004000; /*!< ADC JSQ2 bit 0 */
constexpr uint32_t ADC_JSQR_JSQ2_1     = 0x00008000; /*!< ADC JSQ2 bit 1 */
constexpr uint32_t ADC_JSQR_JSQ2_2     = 0x00010000; /*!< ADC JSQ2 bit 2 */
constexpr uint32_t ADC_JSQR_JSQ2_3     = 0x00020000; /*!< ADC JSQ2 bit 3 */
constexpr uint32_t ADC_JSQR_JSQ2_4     = 0x00040000; /*!< ADC JSQ2 bit 4 */

constexpr uint32_t ADC_JSQR_JSQ3       = 0x01F00000; /*!< ADC 3rd conversion in injected sequence */
constexpr uint32_t ADC_JSQR_JSQ3_0     = 0x00100000; /*!< ADC JSQ3 bit 0 */
constexpr uint32_t ADC_JSQR_JSQ3_1     = 0x00200000; /*!< ADC JSQ3 bit 1 */
constexpr uint32_t ADC_JSQR_JSQ3_2     = 0x00400000; /*!< ADC JSQ3 bit 2 */
constexpr uint32_t ADC_JSQR_JSQ3_3     = 0x00800000; /*!< ADC JSQ3 bit 3 */
constexpr uint32_t ADC_JSQR_JSQ3_4     = 0x01000000; /*!< ADC JSQ3 bit 4 */

constexpr uint32_t ADC_JSQR_JSQ4       = 0x7C000000; /*!< ADC 4th conversion in injected sequence */
constexpr uint32_t ADC_JSQR_JSQ4_0     = 0x04000000; /*!< ADC JSQ4 bit 0 */
constexpr uint32_t ADC_JSQR_JSQ4_1     = 0x08000000; /*!< ADC JSQ4 bit 1 */
constexpr uint32_t ADC_JSQR_JSQ4_2     = 0x10000000; /*!< ADC JSQ4 bit 2 */
constexpr uint32_t ADC_JSQR_JSQ4_3     = 0x20000000; /*!< ADC JSQ4 bit 3 */
constexpr uint32_t ADC_JSQR_JSQ4_4     = 0x40000000; /*!< ADC JSQ4 bit 4 */

/********************  Bit definition for ADC_OFR1 register  ********************/
constexpr uint32_t ADC_OFR1_OFFSET1    = 0x00000FFF; /*!< ADC data offset 1 for channel programmed into bits OFFSET1_CH[4:0] */
constexpr uint32_t ADC_OFR1_OFFSET1_0  = 0x00000001; /*!< ADC OFFSET1 bit 0 */
constexpr uint32_t ADC_OFR1_OFFSET1_1  = 0x00000002; /*!< ADC OFFSET1 bit 1 */
constexpr uint32_t ADC_OFR1_OFFSET1_2  = 0x00000004; /*!< ADC OFFSET1 bit 2 */
constexpr uint32_t ADC_OFR1_OFFSET1_3  = 0x00000008; /*!< ADC OFFSET1 bit 3 */
constexpr uint32_t ADC_OFR1_OFFSET1_4  = 0x00000010; /*!< ADC OFFSET1 bit 4 */
constexpr uint32_t ADC_OFR1_OFFSET1_5  = 0x00000020; /*!< ADC OFFSET1 bit 5 */
constexpr uint32_t ADC_OFR1_OFFSET1_6  = 0x00000040; /*!< ADC OFFSET1 bit 6 */
constexpr uint32_t ADC_OFR1_OFFSET1_7  = 0x00000080; /*!< ADC OFFSET1 bit 7 */
constexpr uint32_t ADC_OFR1_OFFSET1_8  = 0x00000100; /*!< ADC OFFSET1 bit 8 */
constexpr uint32_t ADC_OFR1_OFFSET1_9  = 0x00000200; /*!< ADC OFFSET1 bit 9 */
constexpr uint32_t ADC_OFR1_OFFSET1_10 = 0x00000400; /*!< ADC OFFSET1 bit 10 */
constexpr uint32_t ADC_OFR1_OFFSET1_11 = 0x00000800; /*!< ADC OFFSET1 bit 11 */

constexpr uint32_t ADC_OFR1_OFFSET1_CH     = 0x7C000000; /*!< ADC Channel selection for the data offset 1 */
constexpr uint32_t ADC_OFR1_OFFSET1_CH_0  = 0x04000000; /*!< ADC OFFSET1_CH bit 0 */
constexpr uint32_t ADC_OFR1_OFFSET1_CH_1  = 0x08000000; /*!< ADC OFFSET1_CH bit 1 */
constexpr uint32_t ADC_OFR1_OFFSET1_CH_2  = 0x10000000; /*!< ADC OFFSET1_CH bit 2 */
constexpr uint32_t ADC_OFR1_OFFSET1_CH_3  = 0x20000000; /*!< ADC OFFSET1_CH bit 3 */
constexpr uint32_t ADC_OFR1_OFFSET1_CH_4  = 0x40000000; /*!< ADC OFFSET1_CH bit 4 */

constexpr uint32_t ADC_OFR1_OFFSET1_EN = 0x80000000; /*!< ADC offset 1 enable */

/********************  Bit definition for ADC_OFR2 register  ********************/
constexpr uint32_t ADC_OFR2_OFFSET2    = 0x00000FFF; /*!< ADC data offset 2 for channel programmed into bits OFFSET2_CH[4:0] */
constexpr uint32_t ADC_OFR2_OFFSET2_0  = 0x00000001; /*!< ADC OFFSET2 bit 0 */
constexpr uint32_t ADC_OFR2_OFFSET2_1  = 0x00000002; /*!< ADC OFFSET2 bit 1 */
constexpr uint32_t ADC_OFR2_OFFSET2_2  = 0x00000004; /*!< ADC OFFSET2 bit 2 */
constexpr uint32_t ADC_OFR2_OFFSET2_3  = 0x00000008; /*!< ADC OFFSET2 bit 3 */
constexpr uint32_t ADC_OFR2_OFFSET2_4  = 0x00000010; /*!< ADC OFFSET2 bit 4 */
constexpr uint32_t ADC_OFR2_OFFSET2_5  = 0x00000020; /*!< ADC OFFSET2 bit 5 */
constexpr uint32_t ADC_OFR2_OFFSET2_6  = 0x00000040; /*!< ADC OFFSET2 bit 6 */
constexpr uint32_t ADC_OFR2_OFFSET2_7  = 0x00000080; /*!< ADC OFFSET2 bit 7 */
constexpr uint32_t ADC_OFR2_OFFSET2_8  = 0x00000100; /*!< ADC OFFSET2 bit 8 */
constexpr uint32_t ADC_OFR2_OFFSET2_9  = 0x00000200; /*!< ADC OFFSET2 bit 9 */
constexpr uint32_t ADC_OFR2_OFFSET2_10 = 0x00000400; /*!< ADC OFFSET2 bit 10 */
constexpr uint32_t ADC_OFR2_OFFSET2_11 = 0x00000800; /*!< ADC OFFSET2 bit 11 */

constexpr uint32_t ADC_OFR2_OFFSET2_CH     = 0x7C000000; /*!< ADC Channel selection for the data offset 2 */
constexpr uint32_t ADC_OFR2_OFFSET2_CH_0  = 0x04000000; /*!< ADC OFFSET2_CH bit 0 */
constexpr uint32_t ADC_OFR2_OFFSET2_CH_1  = 0x08000000; /*!< ADC OFFSET2_CH bit 1 */
constexpr uint32_t ADC_OFR2_OFFSET2_CH_2  = 0x10000000; /*!< ADC OFFSET2_CH bit 2 */
constexpr uint32_t ADC_OFR2_OFFSET2_CH_3  = 0x20000000; /*!< ADC OFFSET2_CH bit 3 */
constexpr uint32_t ADC_OFR2_OFFSET2_CH_4  = 0x40000000; /*!< ADC OFFSET2_CH bit 4 */

constexpr uint32_t ADC_OFR2_OFFSET2_EN = 0x80000000; /*!< ADC offset 2 enable */

/********************  Bit definition for ADC_OFR3 register  ********************/
constexpr uint32_t ADC_OFR3_OFFSET3    = 0x00000FFF; /*!< ADC data offset 3 for channel programmed into bits OFFSET3_CH[4:0] */
constexpr uint32_t ADC_OFR3_OFFSET3_0  = 0x00000001; /*!< ADC OFFSET3 bit 0 */
constexpr uint32_t ADC_OFR3_OFFSET3_1  = 0x00000002; /*!< ADC OFFSET3 bit 1 */
constexpr uint32_t ADC_OFR3_OFFSET3_2  = 0x00000004; /*!< ADC OFFSET3 bit 2 */
constexpr uint32_t ADC_OFR3_OFFSET3_3  = 0x00000008; /*!< ADC OFFSET3 bit 3 */
constexpr uint32_t ADC_OFR3_OFFSET3_4  = 0x00000010; /*!< ADC OFFSET3 bit 4 */
constexpr uint32_t ADC_OFR3_OFFSET3_5  = 0x00000020; /*!< ADC OFFSET3 bit 5 */
constexpr uint32_t ADC_OFR3_OFFSET3_6  = 0x00000040; /*!< ADC OFFSET3 bit 6 */
constexpr uint32_t ADC_OFR3_OFFSET3_7  = 0x00000080; /*!< ADC OFFSET3 bit 7 */
constexpr uint32_t ADC_OFR3_OFFSET3_8  = 0x00000100; /*!< ADC OFFSET3 bit 8 */
constexpr uint32_t ADC_OFR3_OFFSET3_9  = 0x00000200; /*!< ADC OFFSET3 bit 9 */
constexpr uint32_t ADC_OFR3_OFFSET3_10 = 0x00000400; /*!< ADC OFFSET3 bit 10 */
constexpr uint32_t ADC_OFR3_OFFSET3_11 = 0x00000800; /*!< ADC OFFSET3 bit 11 */

constexpr uint32_t ADC_OFR3_OFFSET3_CH     = 0x7C000000; /*!< ADC Channel selection for the data offset 3 */
constexpr uint32_t ADC_OFR3_OFFSET3_CH_0  = 0x04000000; /*!< ADC OFFSET3_CH bit 0 */
constexpr uint32_t ADC_OFR3_OFFSET3_CH_1  = 0x08000000; /*!< ADC OFFSET3_CH bit 1 */
constexpr uint32_t ADC_OFR3_OFFSET3_CH_2  = 0x10000000; /*!< ADC OFFSET3_CH bit 2 */
constexpr uint32_t ADC_OFR3_OFFSET3_CH_3  = 0x20000000; /*!< ADC OFFSET3_CH bit 3 */
constexpr uint32_t ADC_OFR3_OFFSET3_CH_4  = 0x40000000; /*!< ADC OFFSET3_CH bit 4 */

constexpr uint32_t ADC_OFR3_OFFSET3_EN = 0x80000000; /*!< ADC offset 3 enable */

/********************  Bit definition for ADC_OFR4 register  ********************/
constexpr uint32_t ADC_OFR4_OFFSET4    = 0x00000FFF; /*!< ADC data offset 4 for channel programmed into bits OFFSET4_CH[4:0] */
constexpr uint32_t ADC_OFR4_OFFSET4_0  = 0x00000001; /*!< ADC OFFSET4 bit 0 */
constexpr uint32_t ADC_OFR4_OFFSET4_1  = 0x00000002; /*!< ADC OFFSET4 bit 1 */
constexpr uint32_t ADC_OFR4_OFFSET4_2  = 0x00000004; /*!< ADC OFFSET4 bit 2 */
constexpr uint32_t ADC_OFR4_OFFSET4_3  = 0x00000008; /*!< ADC OFFSET4 bit 3 */
constexpr uint32_t ADC_OFR4_OFFSET4_4  = 0x00000010; /*!< ADC OFFSET4 bit 4 */
constexpr uint32_t ADC_OFR4_OFFSET4_5  = 0x00000020; /*!< ADC OFFSET4 bit 5 */
constexpr uint32_t ADC_OFR4_OFFSET4_6  = 0x00000040; /*!< ADC OFFSET4 bit 6 */
constexpr uint32_t ADC_OFR4_OFFSET4_7  = 0x00000080; /*!< ADC OFFSET4 bit 7 */
constexpr uint32_t ADC_OFR4_OFFSET4_8  = 0x00000100; /*!< ADC OFFSET4 bit 8 */
constexpr uint32_t ADC_OFR4_OFFSET4_9  = 0x00000200; /*!< ADC OFFSET4 bit 9 */
constexpr uint32_t ADC_OFR4_OFFSET4_10 = 0x00000400; /*!< ADC OFFSET4 bit 10 */
constexpr uint32_t ADC_OFR4_OFFSET4_11 = 0x00000800; /*!< ADC OFFSET4 bit 11 */

constexpr uint32_t ADC_OFR4_OFFSET4_CH     = 0x7C000000; /*!< ADC Channel selection for the data offset 4 */
constexpr uint32_t ADC_OFR4_OFFSET4_CH_0  = 0x04000000; /*!< ADC OFFSET4_CH bit 0 */
constexpr uint32_t ADC_OFR4_OFFSET4_CH_1  = 0x08000000; /*!< ADC OFFSET4_CH bit 1 */
constexpr uint32_t ADC_OFR4_OFFSET4_CH_2  = 0x10000000; /*!< ADC OFFSET4_CH bit 2 */
constexpr uint32_t ADC_OFR4_OFFSET4_CH_3  = 0x20000000; /*!< ADC OFFSET4_CH bit 3 */
constexpr uint32_t ADC_OFR4_OFFSET4_CH_4  = 0x40000000; /*!< ADC OFFSET4_CH bit 4 */

constexpr uint32_t ADC_OFR4_OFFSET4_EN = 0x80000000; /*!< ADC offset 4 enable */

/********************  Bit definition for ADC_JDR1 register  ********************/
constexpr uint32_t ADC_JDR1_JDATA      = 0x0000FFFF; /*!< ADC Injected DATA */
constexpr uint32_t ADC_JDR1_JDATA_0    = 0x00000001; /*!< ADC JDATA bit 0 */
constexpr uint32_t ADC_JDR1_JDATA_1    = 0x00000002; /*!< ADC JDATA bit 1 */
constexpr uint32_t ADC_JDR1_JDATA_2    = 0x00000004; /*!< ADC JDATA bit 2 */
constexpr uint32_t ADC_JDR1_JDATA_3    = 0x00000008; /*!< ADC JDATA bit 3 */
constexpr uint32_t ADC_JDR1_JDATA_4    = 0x00000010; /*!< ADC JDATA bit 4 */
constexpr uint32_t ADC_JDR1_JDATA_5    = 0x00000020; /*!< ADC JDATA bit 5 */
constexpr uint32_t ADC_JDR1_JDATA_6    = 0x00000040; /*!< ADC JDATA bit 6 */
constexpr uint32_t ADC_JDR1_JDATA_7    = 0x00000080; /*!< ADC JDATA bit 7 */
constexpr uint32_t ADC_JDR1_JDATA_8    = 0x00000100; /*!< ADC JDATA bit 8 */
constexpr uint32_t ADC_JDR1_JDATA_9    = 0x00000200; /*!< ADC JDATA bit 9 */
constexpr uint32_t ADC_JDR1_JDATA_10   = 0x00000400; /*!< ADC JDATA bit 10 */
constexpr uint32_t ADC_JDR1_JDATA_11   = 0x00000800; /*!< ADC JDATA bit 11 */
constexpr uint32_t ADC_JDR1_JDATA_12   = 0x00001000; /*!< ADC JDATA bit 12 */
constexpr uint32_t ADC_JDR1_JDATA_13   = 0x00002000; /*!< ADC JDATA bit 13 */
constexpr uint32_t ADC_JDR1_JDATA_14   = 0x00004000; /*!< ADC JDATA bit 14 */
constexpr uint32_t ADC_JDR1_JDATA_15   = 0x00008000; /*!< ADC JDATA bit 15 */

/********************  Bit definition for ADC_JDR2 register  ********************/
constexpr uint32_t ADC_JDR2_JDATA      = 0x0000FFFF; /*!< ADC Injected DATA */
constexpr uint32_t ADC_JDR2_JDATA_0    = 0x00000001; /*!< ADC JDATA bit 0 */
constexpr uint32_t ADC_JDR2_JDATA_1    = 0x00000002; /*!< ADC JDATA bit 1 */
constexpr uint32_t ADC_JDR2_JDATA_2    = 0x00000004; /*!< ADC JDATA bit 2 */
constexpr uint32_t ADC_JDR2_JDATA_3    = 0x00000008; /*!< ADC JDATA bit 3 */
constexpr uint32_t ADC_JDR2_JDATA_4    = 0x00000010; /*!< ADC JDATA bit 4 */
constexpr uint32_t ADC_JDR2_JDATA_5    = 0x00000020; /*!< ADC JDATA bit 5 */
constexpr uint32_t ADC_JDR2_JDATA_6    = 0x00000040; /*!< ADC JDATA bit 6 */
constexpr uint32_t ADC_JDR2_JDATA_7    = 0x00000080; /*!< ADC JDATA bit 7 */
constexpr uint32_t ADC_JDR2_JDATA_8    = 0x00000100; /*!< ADC JDATA bit 8 */
constexpr uint32_t ADC_JDR2_JDATA_9    = 0x00000200; /*!< ADC JDATA bit 9 */
constexpr uint32_t ADC_JDR2_JDATA_10   = 0x00000400; /*!< ADC JDATA bit 10 */
constexpr uint32_t ADC_JDR2_JDATA_11   = 0x00000800; /*!< ADC JDATA bit 11 */
constexpr uint32_t ADC_JDR2_JDATA_12   = 0x00001000; /*!< ADC JDATA bit 12 */
constexpr uint32_t ADC_JDR2_JDATA_13   = 0x00002000; /*!< ADC JDATA bit 13 */
constexpr uint32_t ADC_JDR2_JDATA_14   = 0x00004000; /*!< ADC JDATA bit 14 */
constexpr uint32_t ADC_JDR2_JDATA_15   = 0x00008000; /*!< ADC JDATA bit 15 */

/********************  Bit definition for ADC_JDR3 register  ********************/
constexpr uint32_t ADC_JDR3_JDATA      = 0x0000FFFF; /*!< ADC Injected DATA */
constexpr uint32_t ADC_JDR3_JDATA_0    = 0x00000001; /*!< ADC JDATA bit 0 */
constexpr uint32_t ADC_JDR3_JDATA_1    = 0x00000002; /*!< ADC JDATA bit 1 */
constexpr uint32_t ADC_JDR3_JDATA_2    = 0x00000004; /*!< ADC JDATA bit 2 */
constexpr uint32_t ADC_JDR3_JDATA_3    = 0x00000008; /*!< ADC JDATA bit 3 */
constexpr uint32_t ADC_JDR3_JDATA_4    = 0x00000010; /*!< ADC JDATA bit 4 */
constexpr uint32_t ADC_JDR3_JDATA_5    = 0x00000020; /*!< ADC JDATA bit 5 */
constexpr uint32_t ADC_JDR3_JDATA_6    = 0x00000040; /*!< ADC JDATA bit 6 */
constexpr uint32_t ADC_JDR3_JDATA_7    = 0x00000080; /*!< ADC JDATA bit 7 */
constexpr uint32_t ADC_JDR3_JDATA_8    = 0x00000100; /*!< ADC JDATA bit 8 */
constexpr uint32_t ADC_JDR3_JDATA_9    = 0x00000200; /*!< ADC JDATA bit 9 */
constexpr uint32_t ADC_JDR3_JDATA_10   = 0x00000400; /*!< ADC JDATA bit 10 */
constexpr uint32_t ADC_JDR3_JDATA_11   = 0x00000800; /*!< ADC JDATA bit 11 */
constexpr uint32_t ADC_JDR3_JDATA_12   = 0x00001000; /*!< ADC JDATA bit 12 */
constexpr uint32_t ADC_JDR3_JDATA_13   = 0x00002000; /*!< ADC JDATA bit 13 */
constexpr uint32_t ADC_JDR3_JDATA_14   = 0x00004000; /*!< ADC JDATA bit 14 */
constexpr uint32_t ADC_JDR3_JDATA_15   = 0x00008000; /*!< ADC JDATA bit 15 */

/********************  Bit definition for ADC_JDR4 register  ********************/
constexpr uint32_t ADC_JDR4_JDATA      = 0x0000FFFF; /*!< ADC Injected DATA */
constexpr uint32_t ADC_JDR4_JDATA_0    = 0x00000001; /*!< ADC JDATA bit 0 */
constexpr uint32_t ADC_JDR4_JDATA_1    = 0x00000002; /*!< ADC JDATA bit 1 */
constexpr uint32_t ADC_JDR4_JDATA_2    = 0x00000004; /*!< ADC JDATA bit 2 */
constexpr uint32_t ADC_JDR4_JDATA_3    = 0x00000008; /*!< ADC JDATA bit 3 */
constexpr uint32_t ADC_JDR4_JDATA_4    = 0x00000010; /*!< ADC JDATA bit 4 */
constexpr uint32_t ADC_JDR4_JDATA_5    = 0x00000020; /*!< ADC JDATA bit 5 */
constexpr uint32_t ADC_JDR4_JDATA_6    = 0x00000040; /*!< ADC JDATA bit 6 */
constexpr uint32_t ADC_JDR4_JDATA_7    = 0x00000080; /*!< ADC JDATA bit 7 */
constexpr uint32_t ADC_JDR4_JDATA_8    = 0x00000100; /*!< ADC JDATA bit 8 */
constexpr uint32_t ADC_JDR4_JDATA_9    = 0x00000200; /*!< ADC JDATA bit 9 */
constexpr uint32_t ADC_JDR4_JDATA_10   = 0x00000400; /*!< ADC JDATA bit 10 */
constexpr uint32_t ADC_JDR4_JDATA_11   = 0x00000800; /*!< ADC JDATA bit 11 */
constexpr uint32_t ADC_JDR4_JDATA_12   = 0x00001000; /*!< ADC JDATA bit 12 */
constexpr uint32_t ADC_JDR4_JDATA_13   = 0x00002000; /*!< ADC JDATA bit 13 */
constexpr uint32_t ADC_JDR4_JDATA_14   = 0x00004000; /*!< ADC JDATA bit 14 */
constexpr uint32_t ADC_JDR4_JDATA_15   = 0x00008000; /*!< ADC JDATA bit 15 */

/********************  Bit definition for ADC_AWD2CR register  ********************/
constexpr uint32_t ADC_AWD2CR_AWD2CH    = 0x0007FFFE; /*!< ADC Analog watchdog 2 channel selection */
constexpr uint32_t ADC_AWD2CR_AWD2CH_0  = 0x00000002; /*!< ADC AWD2CH bit 0 */
constexpr uint32_t ADC_AWD2CR_AWD2CH_1  = 0x00000004; /*!< ADC AWD2CH bit 1 */
constexpr uint32_t ADC_AWD2CR_AWD2CH_2  = 0x00000008; /*!< ADC AWD2CH bit 2 */
constexpr uint32_t ADC_AWD2CR_AWD2CH_3  = 0x00000010; /*!< ADC AWD2CH bit 3 */
constexpr uint32_t ADC_AWD2CR_AWD2CH_4  = 0x00000020; /*!< ADC AWD2CH bit 4 */
constexpr uint32_t ADC_AWD2CR_AWD2CH_5  = 0x00000040; /*!< ADC AWD2CH bit 5 */
constexpr uint32_t ADC_AWD2CR_AWD2CH_6  = 0x00000080; /*!< ADC AWD2CH bit 6 */
constexpr uint32_t ADC_AWD2CR_AWD2CH_7  = 0x00000100; /*!< ADC AWD2CH bit 7 */
constexpr uint32_t ADC_AWD2CR_AWD2CH_8  = 0x00000200; /*!< ADC AWD2CH bit 8 */
constexpr uint32_t ADC_AWD2CR_AWD2CH_9  = 0x00000400; /*!< ADC AWD2CH bit 9 */
constexpr uint32_t ADC_AWD2CR_AWD2CH_10 = 0x00000800; /*!< ADC AWD2CH bit 10 */
constexpr uint32_t ADC_AWD2CR_AWD2CH_11 = 0x00001000; /*!< ADC AWD2CH bit 11 */
constexpr uint32_t ADC_AWD2CR_AWD2CH_12 = 0x00002000; /*!< ADC AWD2CH bit 12 */
constexpr uint32_t ADC_AWD2CR_AWD2CH_13 = 0x00004000; /*!< ADC AWD2CH bit 13 */
constexpr uint32_t ADC_AWD2CR_AWD2CH_14 = 0x00008000; /*!< ADC AWD2CH bit 14 */
constexpr uint32_t ADC_AWD2CR_AWD2CH_15 = 0x00010000; /*!< ADC AWD2CH bit 15 */
constexpr uint32_t ADC_AWD2CR_AWD2CH_16 = 0x00020000; /*!< ADC AWD2CH bit 16 */
constexpr uint32_t ADC_AWD2CR_AWD2CH_17 = 0x00030000; /*!< ADC AWD2CH bit 17 */

/********************  Bit definition for ADC_AWD3CR register  ********************/
constexpr uint32_t ADC_AWD3CR_AWD3CH    = 0x0007FFFE; /*!< ADC Analog watchdog 2 channel selection */
constexpr uint32_t ADC_AWD3CR_AWD3CH_0  = 0x00000002; /*!< ADC AWD3CH bit 0 */
constexpr uint32_t ADC_AWD3CR_AWD3CH_1  = 0x00000004; /*!< ADC AWD3CH bit 1 */
constexpr uint32_t ADC_AWD3CR_AWD3CH_2  = 0x00000008; /*!< ADC AWD3CH bit 2 */
constexpr uint32_t ADC_AWD3CR_AWD3CH_3  = 0x00000010; /*!< ADC AWD3CH bit 3 */
constexpr uint32_t ADC_AWD3CR_AWD3CH_4  = 0x00000020; /*!< ADC AWD3CH bit 4 */
constexpr uint32_t ADC_AWD3CR_AWD3CH_5  = 0x00000040; /*!< ADC AWD3CH bit 5 */
constexpr uint32_t ADC_AWD3CR_AWD3CH_6  = 0x00000080; /*!< ADC AWD3CH bit 6 */
constexpr uint32_t ADC_AWD3CR_AWD3CH_7  = 0x00000100; /*!< ADC AWD3CH bit 7 */
constexpr uint32_t ADC_AWD3CR_AWD3CH_8  = 0x00000200; /*!< ADC AWD3CH bit 8 */
constexpr uint32_t ADC_AWD3CR_AWD3CH_9  = 0x00000400; /*!< ADC AWD3CH bit 9 */
constexpr uint32_t ADC_AWD3CR_AWD3CH_10 = 0x00000800; /*!< ADC AWD3CH bit 10 */
constexpr uint32_t ADC_AWD3CR_AWD3CH_11 = 0x00001000; /*!< ADC AWD3CH bit 11 */
constexpr uint32_t ADC_AWD3CR_AWD3CH_12 = 0x00002000; /*!< ADC AWD3CH bit 12 */
constexpr uint32_t ADC_AWD3CR_AWD3CH_13 = 0x00004000; /*!< ADC AWD3CH bit 13 */
constexpr uint32_t ADC_AWD3CR_AWD3CH_14 = 0x00008000; /*!< ADC AWD3CH bit 14 */
constexpr uint32_t ADC_AWD3CR_AWD3CH_15 = 0x00010000; /*!< ADC AWD3CH bit 15 */
constexpr uint32_t ADC_AWD3CR_AWD3CH_16 = 0x00020000; /*!< ADC AWD3CH bit 16 */
constexpr uint32_t ADC_AWD3CR_AWD3CH_17 = 0x00030000; /*!< ADC AWD3CH bit 17 */

/********************  Bit definition for ADC_DIFSEL register  ********************/
constexpr uint32_t ADC_DIFSEL_DIFSEL    = 0x0007FFFE; /*!< ADC differential modes for channels 1 to 18 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_0  = 0x00000002; /*!< ADC DIFSEL bit 0 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_1  = 0x00000004; /*!< ADC DIFSEL bit 1 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_2  = 0x00000008; /*!< ADC DIFSEL bit 2 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_3  = 0x00000010; /*!< ADC DIFSEL bit 3 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_4  = 0x00000020; /*!< ADC DIFSEL bit 4 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_5  = 0x00000040; /*!< ADC DIFSEL bit 5 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_6  = 0x00000080; /*!< ADC DIFSEL bit 6 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_7  = 0x00000100; /*!< ADC DIFSEL bit 7 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_8  = 0x00000200; /*!< ADC DIFSEL bit 8 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_9  = 0x00000400; /*!< ADC DIFSEL bit 9 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_10 = 0x00000800; /*!< ADC DIFSEL bit 10 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_11 = 0x00001000; /*!< ADC DIFSEL bit 11 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_12 = 0x00002000; /*!< ADC DIFSEL bit 12 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_13 = 0x00004000; /*!< ADC DIFSEL bit 13 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_14 = 0x00008000; /*!< ADC DIFSEL bit 14 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_15 = 0x00010000; /*!< ADC DIFSEL bit 15 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_16 = 0x00020000; /*!< ADC DIFSEL bit 16 */
constexpr uint32_t ADC_DIFSEL_DIFSEL_17 = 0x00030000; /*!< ADC DIFSEL bit 17 */

/********************  Bit definition for ADC_CALFACT register  ********************/
constexpr uint32_t ADC_CALFACT_CALFACT_S   = 0x0000007F; /*!< ADC calibration factors in single-ended mode */
constexpr uint32_t ADC_CALFACT_CALFACT_S_0 = 0x00000001; /*!< ADC CALFACT_S bit 0 */
constexpr uint32_t ADC_CALFACT_CALFACT_S_1 = 0x00000002; /*!< ADC CALFACT_S bit 1 */
constexpr uint32_t ADC_CALFACT_CALFACT_S_2 = 0x00000004; /*!< ADC CALFACT_S bit 2 */
constexpr uint32_t ADC_CALFACT_CALFACT_S_3 = 0x00000008; /*!< ADC CALFACT_S bit 3 */
constexpr uint32_t ADC_CALFACT_CALFACT_S_4 = 0x00000010; /*!< ADC CALFACT_S bit 4 */
constexpr uint32_t ADC_CALFACT_CALFACT_S_5 = 0x00000020; /*!< ADC CALFACT_S bit 5 */
constexpr uint32_t ADC_CALFACT_CALFACT_S_6 = 0x00000040; /*!< ADC CALFACT_S bit 6 */
constexpr uint32_t ADC_CALFACT_CALFACT_D   = 0x007F0000; /*!< ADC calibration factors in differential mode */
constexpr uint32_t ADC_CALFACT_CALFACT_D_0 = 0x00010000; /*!< ADC CALFACT_D bit 0 */
constexpr uint32_t ADC_CALFACT_CALFACT_D_1 = 0x00020000; /*!< ADC CALFACT_D bit 1 */
constexpr uint32_t ADC_CALFACT_CALFACT_D_2 = 0x00040000; /*!< ADC CALFACT_D bit 2 */
constexpr uint32_t ADC_CALFACT_CALFACT_D_3 = 0x00080000; /*!< ADC CALFACT_D bit 3 */
constexpr uint32_t ADC_CALFACT_CALFACT_D_4 = 0x00100000; /*!< ADC CALFACT_D bit 4 */
constexpr uint32_t ADC_CALFACT_CALFACT_D_5 = 0x00200000; /*!< ADC CALFACT_D bit 5 */
constexpr uint32_t ADC_CALFACT_CALFACT_D_6 = 0x00400000; /*!< ADC CALFACT_D bit 6 */

/*************************  ADC Common registers  *****************************/
/********************  Bit definition for ADC12_CSR register  ********************/
constexpr uint32_t ADC12_CSR_ADRDY_MST         = 0x00000001; /*!< Master ADC ready */
constexpr uint32_t ADC12_CSR_ADRDY_EOSMP_MST   = 0x00000002; /*!< End of sampling phase flag of the master ADC */
constexpr uint32_t ADC12_CSR_ADRDY_EOC_MST     = 0x00000004; /*!< End of regular conversion of the master ADC */
constexpr uint32_t ADC12_CSR_ADRDY_EOS_MST     = 0x00000008; /*!< End of regular sequence flag of the master ADC */
constexpr uint32_t ADC12_CSR_ADRDY_OVR_MST     = 0x00000010; /*!< Overrun flag of the master ADC */
constexpr uint32_t ADC12_CSR_ADRDY_JEOC_MST    = 0x00000020; /*!< End of injected conversion of the master ADC */
constexpr uint32_t ADC12_CSR_ADRDY_JEOS_MST    = 0x00000040; /*!< End of injected sequence flag of the master ADC */
constexpr uint32_t ADC12_CSR_AWD1_MST          = 0x00000080; /*!< Analog watchdog 1 flag of the master ADC */
constexpr uint32_t ADC12_CSR_AWD2_MST          = 0x00000100; /*!< Analog watchdog 2 flag of the master ADC */
constexpr uint32_t ADC12_CSR_AWD3_MST          = 0x00000200; /*!< Analog watchdog 3 flag of the master ADC */
constexpr uint32_t ADC12_CSR_JQOVF_MST         = 0x00000400; /*!< Injected context queue overflow flag of the master ADC */
constexpr uint32_t ADC12_CSR_ADRDY_SLV         = 0x00010000; /*!< Slave ADC ready */
constexpr uint32_t ADC12_CSR_ADRDY_EOSMP_SLV   = 0x00020000; /*!< End of sampling phase flag of the slave ADC */
constexpr uint32_t ADC12_CSR_ADRDY_EOC_SLV     = 0x00040000; /*!< End of regular conversion of the slave ADC */
constexpr uint32_t ADC12_CSR_ADRDY_EOS_SLV     = 0x00080000; /*!< End of regular sequence flag of the slave ADC */
constexpr uint32_t ADC12_CSR_ADRDY_OVR_SLV     = 0x00100000; /*!< Overrun flag of the slave ADC */
constexpr uint32_t ADC12_CSR_ADRDY_JEOC_SLV    = 0x00200000; /*!< End of injected conversion of the slave ADC */
constexpr uint32_t ADC12_CSR_ADRDY_JEOS_SLV    = 0x00400000; /*!< End of injected sequence flag of the slave ADC */
constexpr uint32_t ADC12_CSR_AWD1_SLV          = 0x00800000; /*!< Analog watchdog 1 flag of the slave ADC */
constexpr uint32_t ADC12_CSR_AWD2_SLV          = 0x01000000; /*!< Analog watchdog 2 flag of the slave ADC */
constexpr uint32_t ADC12_CSR_AWD3_SLV          = 0x02000000; /*!< Analog watchdog 3 flag of the slave ADC */
constexpr uint32_t ADC12_CSR_JQOVF_SLV         = 0x04000000; /*!< Injected context queue overflow flag of the slave ADC */

/********************  Bit definition for ADC34_CSR register  ********************/
constexpr uint32_t ADC34_CSR_ADRDY_MST         = 0x00000001; /*!< Master ADC ready */
constexpr uint32_t ADC34_CSR_ADRDY_EOSMP_MST   = 0x00000002; /*!< End of sampling phase flag of the master ADC */
constexpr uint32_t ADC34_CSR_ADRDY_EOC_MST     = 0x00000004; /*!< End of regular conversion of the master ADC */
constexpr uint32_t ADC34_CSR_ADRDY_EOS_MST     = 0x00000008; /*!< End of regular sequence flag of the master ADC */
constexpr uint32_t ADC34_CSR_ADRDY_OVR_MST     = 0x00000010; /*!< Overrun flag of the master ADC */
constexpr uint32_t ADC34_CSR_ADRDY_JEOC_MST    = 0x00000020; /*!< End of injected conversion of the master ADC */
constexpr uint32_t ADC34_CSR_ADRDY_JEOS_MST    = 0x00000040; /*!< End of injected sequence flag of the master ADC */
constexpr uint32_t ADC34_CSR_AWD1_MST          = 0x00000080; /*!< Analog watchdog 1 flag of the master ADC */
constexpr uint32_t ADC34_CSR_AWD2_MST          = 0x00000100; /*!< Analog watchdog 2 flag of the master ADC */
constexpr uint32_t ADC34_CSR_AWD3_MST          = 0x00000200; /*!< Analog watchdog 3 flag of the master ADC */
constexpr uint32_t ADC34_CSR_JQOVF_MST         = 0x00000400; /*!< Injected context queue overflow flag of the master ADC */
constexpr uint32_t ADC34_CSR_ADRDY_SLV         = 0x00010000; /*!< Slave ADC ready */
constexpr uint32_t ADC34_CSR_ADRDY_EOSMP_SLV   = 0x00020000; /*!< End of sampling phase flag of the slave ADC */
constexpr uint32_t ADC34_CSR_ADRDY_EOC_SLV     = 0x00040000; /*!< End of regular conversion of the slave ADC */
constexpr uint32_t ADC34_CSR_ADRDY_EOS_SLV     = 0x00080000; /*!< End of regular sequence flag of the slave ADC */
constexpr uint32_t ADC34_CSR_ADRDY_OVR_SLV     = 0x00100000; /*!< Overrun flag of the slave ADC */
constexpr uint32_t ADC34_CSR_ADRDY_JEOC_SLV    = 0x00200000; /*!< End of injected conversion of the slave ADC */
constexpr uint32_t ADC34_CSR_ADRDY_JEOS_SLV    = 0x00400000; /*!< End of injected sequence flag of the slave ADC */
constexpr uint32_t ADC34_CSR_AWD1_SLV          = 0x00800000; /*!< Analog watchdog 1 flag of the slave ADC */
constexpr uint32_t ADC34_CSR_AWD2_SLV          = 0x01000000; /*!< Analog watchdog 2 flag of the slave ADC */
constexpr uint32_t ADC34_CSR_AWD3_SLV          = 0x02000000; /*!< Analog watchdog 3 flag of the slave ADC */
constexpr uint32_t ADC34_CSR_JQOVF_SLV         = 0x04000000; /*!< Injected context queue overflow flag of the slave ADC */

/********************  Bit definition for ADC_CCR register  ********************/
constexpr uint32_t ADC12_CCR_MULTI             = 0x0000001F; /*!< Multi ADC mode selection */
constexpr uint32_t ADC12_CCR_MULTI_0           = 0x00000001; /*!< MULTI bit 0 */
constexpr uint32_t ADC12_CCR_MULTI_1           = 0x00000002; /*!< MULTI bit 1 */
constexpr uint32_t ADC12_CCR_MULTI_2           = 0x00000004; /*!< MULTI bit 2 */
constexpr uint32_t ADC12_CCR_MULTI_3           = 0x00000008; /*!< MULTI bit 3 */
constexpr uint32_t ADC12_CCR_MULTI_4           = 0x00000010; /*!< MULTI bit 4 */
constexpr uint32_t ADC12_CCR_DELAY             = 0x00000F00; /*!< Delay between 2 sampling phases */
constexpr uint32_t ADC12_CCR_DELAY_0           = 0x00000100; /*!< DELAY bit 0 */
constexpr uint32_t ADC12_CCR_DELAY_1           = 0x00000200; /*!< DELAY bit 1 */
constexpr uint32_t ADC12_CCR_DELAY_2           = 0x00000400; /*!< DELAY bit 2 */
constexpr uint32_t ADC12_CCR_DELAY_3           = 0x00000800; /*!< DELAY bit 3 */
constexpr uint32_t ADC12_CCR_DMACFG            = 0x00002000; /*!< DMA configuration for multi-ADC mode */
constexpr uint32_t ADC12_CCR_MDMA              = 0x0000C000; /*!< DMA mode for multi-ADC mode */
constexpr uint32_t ADC12_CCR_MDMA_0            = 0x00004000; /*!< MDMA bit 0 */
constexpr uint32_t ADC12_CCR_MDMA_1            = 0x00008000; /*!< MDMA bit 1 */
constexpr uint32_t ADC12_CCR_CKMODE            = 0x00030000; /*!< ADC clock mode */
constexpr uint32_t ADC12_CCR_CKMODE_0          = 0x00010000; /*!< CKMODE bit 0 */
constexpr uint32_t ADC12_CCR_CKMODE_1          = 0x00020000; /*!< CKMODE bit 1 */
constexpr uint32_t ADC12_CCR_VREFEN            = 0x00400000; /*!< VREFINT enable */
constexpr uint32_t ADC12_CCR_TSEN              = 0x00800000; /*!< Temperature sensor enable */
constexpr uint32_t ADC12_CCR_VBATEN            = 0x01000000; /*!< VBAT enable */

/********************  Bit definition for ADC_CCR register  ********************/
constexpr uint32_t ADC34_CCR_MULTI             = 0x0000001F; /*!< Multi ADC mode selection */
constexpr uint32_t ADC34_CCR_MULTI_0           = 0x00000001; /*!< MULTI bit 0 */
constexpr uint32_t ADC34_CCR_MULTI_1           = 0x00000002; /*!< MULTI bit 1 */
constexpr uint32_t ADC34_CCR_MULTI_2           = 0x00000004; /*!< MULTI bit 2 */
constexpr uint32_t ADC34_CCR_MULTI_3           = 0x00000008; /*!< MULTI bit 3 */
constexpr uint32_t ADC34_CCR_MULTI_4           = 0x00000010; /*!< MULTI bit 4 */

constexpr uint32_t ADC34_CCR_DELAY             = 0x00000F00; /*!< Delay between 2 sampling phases */
constexpr uint32_t ADC34_CCR_DELAY_0           = 0x00000100; /*!< DELAY bit 0 */
constexpr uint32_t ADC34_CCR_DELAY_1           = 0x00000200; /*!< DELAY bit 1 */
constexpr uint32_t ADC34_CCR_DELAY_2           = 0x00000400; /*!< DELAY bit 2 */
constexpr uint32_t ADC34_CCR_DELAY_3           = 0x00000800; /*!< DELAY bit 3 */

constexpr uint32_t ADC34_CCR_DMACFG            = 0x00002000; /*!< DMA configuration for multi-ADC mode */
constexpr uint32_t ADC34_CCR_MDMA              = 0x0000C000; /*!< DMA mode for multi-ADC mode */
constexpr uint32_t ADC34_CCR_MDMA_0            = 0x00004000; /*!< MDMA bit 0 */
constexpr uint32_t ADC34_CCR_MDMA_1            = 0x00008000; /*!< MDMA bit 1 */

constexpr uint32_t ADC34_CCR_CKMODE            = 0x00030000; /*!< ADC clock mode */
constexpr uint32_t ADC34_CCR_CKMODE_0          = 0x00010000; /*!< CKMODE bit 0 */
constexpr uint32_t ADC34_CCR_CKMODE_1          = 0x00020000; /*!< CKMODE bit 1 */

constexpr uint32_t ADC34_CCR_VREFEN            = 0x00400000; /*!< VREFINT enable */
constexpr uint32_t ADC34_CCR_TSEN              = 0x00800000; /*!< Temperature sensor enable */
constexpr uint32_t ADC34_CCR_VBATEN            = 0x01000000; /*!< VBAT enable */

/********************  Bit definition for ADC_CDR register  ********************/
constexpr uint32_t ADC12_CDR_RDATA_MST         = 0x0000FFFF; /*!< Regular Data of the master ADC */
constexpr uint32_t ADC12_CDR_RDATA_MST_0       = 0x00000001; /*!< RDATA_MST bit 0 */
constexpr uint32_t ADC12_CDR_RDATA_MST_1       = 0x00000002; /*!< RDATA_MST bit 1 */
constexpr uint32_t ADC12_CDR_RDATA_MST_2       = 0x00000004; /*!< RDATA_MST bit 2 */
constexpr uint32_t ADC12_CDR_RDATA_MST_3       = 0x00000008; /*!< RDATA_MST bit 3 */
constexpr uint32_t ADC12_CDR_RDATA_MST_4       = 0x00000010; /*!< RDATA_MST bit 4 */
constexpr uint32_t ADC12_CDR_RDATA_MST_5       = 0x00000020; /*!< RDATA_MST bit 5 */
constexpr uint32_t ADC12_CDR_RDATA_MST_6       = 0x00000040; /*!< RDATA_MST bit 6 */
constexpr uint32_t ADC12_CDR_RDATA_MST_7       = 0x00000080; /*!< RDATA_MST bit 7 */
constexpr uint32_t ADC12_CDR_RDATA_MST_8       = 0x00000100; /*!< RDATA_MST bit 8 */
constexpr uint32_t ADC12_CDR_RDATA_MST_9       = 0x00000200; /*!< RDATA_MST bit 9 */
constexpr uint32_t ADC12_CDR_RDATA_MST_10      = 0x00000400; /*!< RDATA_MST bit 10 */
constexpr uint32_t ADC12_CDR_RDATA_MST_11      = 0x00000800; /*!< RDATA_MST bit 11 */
constexpr uint32_t ADC12_CDR_RDATA_MST_12      = 0x00001000; /*!< RDATA_MST bit 12 */
constexpr uint32_t ADC12_CDR_RDATA_MST_13      = 0x00002000; /*!< RDATA_MST bit 13 */
constexpr uint32_t ADC12_CDR_RDATA_MST_14      = 0x00004000; /*!< RDATA_MST bit 14 */
constexpr uint32_t ADC12_CDR_RDATA_MST_15      = 0x00008000; /*!< RDATA_MST bit 15 */

constexpr uint32_t ADC12_CDR_RDATA_SLV         = 0xFFFF0000; /*!< Regular Data of the master ADC */
constexpr uint32_t ADC12_CDR_RDATA_SLV_0       = 0x00010000; /*!< RDATA_SLV bit 0 */
constexpr uint32_t ADC12_CDR_RDATA_SLV_1       = 0x00020000; /*!< RDATA_SLV bit 1 */
constexpr uint32_t ADC12_CDR_RDATA_SLV_2       = 0x00040000; /*!< RDATA_SLV bit 2 */
constexpr uint32_t ADC12_CDR_RDATA_SLV_3       = 0x00080000; /*!< RDATA_SLV bit 3 */
constexpr uint32_t ADC12_CDR_RDATA_SLV_4       = 0x00100000; /*!< RDATA_SLV bit 4 */
constexpr uint32_t ADC12_CDR_RDATA_SLV_5       = 0x00200000; /*!< RDATA_SLV bit 5 */
constexpr uint32_t ADC12_CDR_RDATA_SLV_6       = 0x00400000; /*!< RDATA_SLV bit 6 */
constexpr uint32_t ADC12_CDR_RDATA_SLV_7       = 0x00800000; /*!< RDATA_SLV bit 7 */
constexpr uint32_t ADC12_CDR_RDATA_SLV_8       = 0x01000000; /*!< RDATA_SLV bit 8 */
constexpr uint32_t ADC12_CDR_RDATA_SLV_9       = 0x02000000; /*!< RDATA_SLV bit 9 */
constexpr uint32_t ADC12_CDR_RDATA_SLV_10      = 0x04000000; /*!< RDATA_SLV bit 10 */
constexpr uint32_t ADC12_CDR_RDATA_SLV_11      = 0x08000000; /*!< RDATA_SLV bit 11 */
constexpr uint32_t ADC12_CDR_RDATA_SLV_12      = 0x10000000; /*!< RDATA_SLV bit 12 */
constexpr uint32_t ADC12_CDR_RDATA_SLV_13      = 0x20000000; /*!< RDATA_SLV bit 13 */
constexpr uint32_t ADC12_CDR_RDATA_SLV_14      = 0x40000000; /*!< RDATA_SLV bit 14 */
constexpr uint32_t ADC12_CDR_RDATA_SLV_15      = 0x80000000; /*!< RDATA_SLV bit 15 */

/********************  Bit definition for ADC_CDR register  ********************/
constexpr uint32_t ADC34_CDR_RDATA_MST         = 0x0000FFFF; /*!< Regular Data of the master ADC */
constexpr uint32_t ADC34_CDR_RDATA_MST_0       = 0x00000001; /*!< RDATA_MST bit 0 */
constexpr uint32_t ADC34_CDR_RDATA_MST_1       = 0x00000002; /*!< RDATA_MST bit 1 */
constexpr uint32_t ADC34_CDR_RDATA_MST_2       = 0x00000004; /*!< RDATA_MST bit 2 */
constexpr uint32_t ADC34_CDR_RDATA_MST_3       = 0x00000008; /*!< RDATA_MST bit 3 */
constexpr uint32_t ADC34_CDR_RDATA_MST_4       = 0x00000010; /*!< RDATA_MST bit 4 */
constexpr uint32_t ADC34_CDR_RDATA_MST_5       = 0x00000020; /*!< RDATA_MST bit 5 */
constexpr uint32_t ADC34_CDR_RDATA_MST_6       = 0x00000040; /*!< RDATA_MST bit 6 */
constexpr uint32_t ADC34_CDR_RDATA_MST_7       = 0x00000080; /*!< RDATA_MST bit 7 */
constexpr uint32_t ADC34_CDR_RDATA_MST_8       = 0x00000100; /*!< RDATA_MST bit 8 */
constexpr uint32_t ADC34_CDR_RDATA_MST_9       = 0x00000200; /*!< RDATA_MST bit 9 */
constexpr uint32_t ADC34_CDR_RDATA_MST_10      = 0x00000400; /*!< RDATA_MST bit 10 */
constexpr uint32_t ADC34_CDR_RDATA_MST_11      = 0x00000800; /*!< RDATA_MST bit 11 */
constexpr uint32_t ADC34_CDR_RDATA_MST_12      = 0x00001000; /*!< RDATA_MST bit 12 */
constexpr uint32_t ADC34_CDR_RDATA_MST_13      = 0x00002000; /*!< RDATA_MST bit 13 */
constexpr uint32_t ADC34_CDR_RDATA_MST_14      = 0x00004000; /*!< RDATA_MST bit 14 */
constexpr uint32_t ADC34_CDR_RDATA_MST_15      = 0x00008000; /*!< RDATA_MST bit 15 */

constexpr uint32_t ADC34_CDR_RDATA_SLV         = 0xFFFF0000; /*!< Regular Data of the master ADC */
constexpr uint32_t ADC34_CDR_RDATA_SLV_0       = 0x00010000; /*!< RDATA_SLV bit 0 */
constexpr uint32_t ADC34_CDR_RDATA_SLV_1       = 0x00020000; /*!< RDATA_SLV bit 1 */
constexpr uint32_t ADC34_CDR_RDATA_SLV_2       = 0x00040000; /*!< RDATA_SLV bit 2 */
constexpr uint32_t ADC34_CDR_RDATA_SLV_3       = 0x00080000; /*!< RDATA_SLV bit 3 */
constexpr uint32_t ADC34_CDR_RDATA_SLV_4       = 0x00100000; /*!< RDATA_SLV bit 4 */
constexpr uint32_t ADC34_CDR_RDATA_SLV_5       = 0x00200000; /*!< RDATA_SLV bit 5 */
constexpr uint32_t ADC34_CDR_RDATA_SLV_6       = 0x00400000; /*!< RDATA_SLV bit 6 */
constexpr uint32_t ADC34_CDR_RDATA_SLV_7       = 0x00800000; /*!< RDATA_SLV bit 7 */
constexpr uint32_t ADC34_CDR_RDATA_SLV_8       = 0x01000000; /*!< RDATA_SLV bit 8 */
constexpr uint32_t ADC34_CDR_RDATA_SLV_9       = 0x02000000; /*!< RDATA_SLV bit 9 */
constexpr uint32_t ADC34_CDR_RDATA_SLV_10      = 0x04000000; /*!< RDATA_SLV bit 10 */
constexpr uint32_t ADC34_CDR_RDATA_SLV_11      = 0x08000000; /*!< RDATA_SLV bit 11 */
constexpr uint32_t ADC34_CDR_RDATA_SLV_12      = 0x10000000; /*!< RDATA_SLV bit 12 */
constexpr uint32_t ADC34_CDR_RDATA_SLV_13      = 0x20000000; /*!< RDATA_SLV bit 13 */
constexpr uint32_t ADC34_CDR_RDATA_SLV_14      = 0x40000000; /*!< RDATA_SLV bit 14 */
constexpr uint32_t ADC34_CDR_RDATA_SLV_15      = 0x80000000; /*!< RDATA_SLV bit 15 */

/******************************************************************************/
/*                                                                            */
/*                      Analog Comparators = COMP;                             */
/*                                                                            */
/******************************************************************************/
/**********************  Bit definition for COMP1_CSR register  ***************/
constexpr uint32_t COMP1_CSR_COMP1EN               = 0x00000001; /*!< COMP1 enable */
constexpr uint32_t COMP1_CSR_COMP1SW1              = 0x00000002; /*!< COMP1 SW1 switch control */
constexpr uint32_t COMP1_CSR_COMP1INSEL            = 0x00000070; /*!< COMP1 inverting input select */
constexpr uint32_t COMP1_CSR_COMP1INSEL_0          = 0x00000010; /*!< COMP1 inverting input select bit 0 */
constexpr uint32_t COMP1_CSR_COMP1INSEL_1          = 0x00000020; /*!< COMP1 inverting input select bit 1 */
constexpr uint32_t COMP1_CSR_COMP1INSEL_2          = 0x00000040; /*!< COMP1 inverting input select bit 2 */
constexpr uint32_t COMP1_CSR_COMP1OUTSEL           = 0x00003C00; /*!< COMP1 output select */
constexpr uint32_t COMP1_CSR_COMP1OUTSEL_0         = 0x00000400; /*!< COMP1 output select bit 0 */
constexpr uint32_t COMP1_CSR_COMP1OUTSEL_1         = 0x00000800; /*!< COMP1 output select bit 1 */
constexpr uint32_t COMP1_CSR_COMP1OUTSEL_2         = 0x00001000; /*!< COMP1 output select bit 2 */
constexpr uint32_t COMP1_CSR_COMP1OUTSEL_3         = 0x00002000; /*!< COMP1 output select bit 3 */
constexpr uint32_t COMP1_CSR_COMP1POL              = 0x00008000; /*!< COMP1 output polarity */
constexpr uint32_t COMP1_CSR_COMP1BLANKING         = 0x000C0000; /*!< COMP1 blanking */
constexpr uint32_t COMP1_CSR_COMP1BLANKING_0       = 0x00040000; /*!< COMP1 blanking bit 0 */
constexpr uint32_t COMP1_CSR_COMP1BLANKING_1       = 0x00080000; /*!< COMP1 blanking bit 1 */
constexpr uint32_t COMP1_CSR_COMP1BLANKING_2       = 0x00100000; /*!< COMP1 blanking bit 2 */
constexpr uint32_t COMP1_CSR_COMP1OUT              = 0x40000000; /*!< COMP1 output level */
constexpr uint32_t COMP1_CSR_COMP1LOCK             = 0x80000000; /*!< COMP1 lock */

/**********************  Bit definition for COMP2_CSR register  ***************/
constexpr uint32_t COMP2_CSR_COMP2EN               = 0x00000001; /*!< COMP2 enable */
constexpr uint32_t COMP2_CSR_COMP2INSEL            = 0x00400070; /*!< COMP2 inverting input select */
constexpr uint32_t COMP2_CSR_COMP2INSEL_0          = 0x00000010; /*!< COMP2 inverting input select bit 0 */
constexpr uint32_t COMP2_CSR_COMP2INSEL_1          = 0x00000020; /*!< COMP2 inverting input select bit 1 */
constexpr uint32_t COMP2_CSR_COMP2INSEL_2          = 0x00000040; /*!< COMP2 inverting input select bit 2 */
constexpr uint32_t COMP2_CSR_COMP2INSEL_3          = 0x00400000; /*!< COMP2 inverting input select bit 3 */
constexpr uint32_t COMP2_CSR_COMP2OUTSEL           = 0x00003C00; /*!< COMP2 output select */
constexpr uint32_t COMP2_CSR_COMP2OUTSEL_0         = 0x00000400; /*!< COMP2 output select bit 0 */
constexpr uint32_t COMP2_CSR_COMP2OUTSEL_1         = 0x00000800; /*!< COMP2 output select bit 1 */
constexpr uint32_t COMP2_CSR_COMP2OUTSEL_2         = 0x00001000; /*!< COMP2 output select bit 2 */
constexpr uint32_t COMP2_CSR_COMP2OUTSEL_3         = 0x00002000; /*!< COMP2 output select bit 3 */
constexpr uint32_t COMP2_CSR_COMP2POL              = 0x00008000; /*!< COMP2 output polarity */
constexpr uint32_t COMP2_CSR_COMP2BLANKING         = 0x000C0000; /*!< COMP2 blanking */
constexpr uint32_t COMP2_CSR_COMP2BLANKING_0       = 0x00040000; /*!< COMP2 blanking bit 0 */
constexpr uint32_t COMP2_CSR_COMP2BLANKING_1       = 0x00080000; /*!< COMP2 blanking bit 1 */
constexpr uint32_t COMP2_CSR_COMP2BLANKING_2       = 0x00100000; /*!< COMP2 blanking bit 2 */
constexpr uint32_t COMP2_CSR_COMP2OUT              = 0x40000000; /*!< COMP2 output level */
constexpr uint32_t COMP2_CSR_COMP2LOCK             = 0x80000000; /*!< COMP2 lock */

/**********************  Bit definition for COMP3_CSR register  ***************/
constexpr uint32_t COMP3_CSR_COMP3EN               = 0x00000001; /*!< COMP3 enable */
constexpr uint32_t COMP3_CSR_COMP3INSEL            = 0x00000070; /*!< COMP3 inverting input select */
constexpr uint32_t COMP3_CSR_COMP3INSEL_0          = 0x00000010; /*!< COMP3 inverting input select bit 0 */
constexpr uint32_t COMP3_CSR_COMP3INSEL_1          = 0x00000020; /*!< COMP3 inverting input select bit 1 */
constexpr uint32_t COMP3_CSR_COMP3INSEL_2          = 0x00000040; /*!< COMP3 inverting input select bit 2 */
constexpr uint32_t COMP3_CSR_COMP3OUTSEL           = 0x00003C00; /*!< COMP3 output select */
constexpr uint32_t COMP3_CSR_COMP3OUTSEL_0         = 0x00000400; /*!< COMP3 output select bit 0 */
constexpr uint32_t COMP3_CSR_COMP3OUTSEL_1         = 0x00000800; /*!< COMP3 output select bit 1 */
constexpr uint32_t COMP3_CSR_COMP3OUTSEL_2         = 0x00001000; /*!< COMP3 output select bit 2 */
constexpr uint32_t COMP3_CSR_COMP3OUTSEL_3         = 0x00002000; /*!< COMP3 output select bit 3 */
constexpr uint32_t COMP3_CSR_COMP3POL              = 0x00008000; /*!< COMP3 output polarity */
constexpr uint32_t COMP3_CSR_COMP3BLANKING         = 0x000C0000; /*!< COMP3 blanking */
constexpr uint32_t COMP3_CSR_COMP3BLANKING_0       = 0x00040000; /*!< COMP3 blanking bit 0 */
constexpr uint32_t COMP3_CSR_COMP3BLANKING_1       = 0x00080000; /*!< COMP3 blanking bit 1 */
constexpr uint32_t COMP3_CSR_COMP3BLANKING_2       = 0x00100000; /*!< COMP3 blanking bit 2 */
constexpr uint32_t COMP3_CSR_COMP3OUT              = 0x40000000; /*!< COMP3 output level */
constexpr uint32_t COMP3_CSR_COMP3LOCK             = 0x80000000; /*!< COMP3 lock */

/**********************  Bit definition for COMP4_CSR register  ***************/
constexpr uint32_t COMP4_CSR_COMP4EN               = 0x00000001; /*!< COMP4 enable */
constexpr uint32_t COMP4_CSR_COMP4INSEL            = 0x00400070; /*!< COMP4 inverting input select */
constexpr uint32_t COMP4_CSR_COMP4INSEL_0          = 0x00000010; /*!< COMP4 inverting input select bit 0 */
constexpr uint32_t COMP4_CSR_COMP4INSEL_1          = 0x00000020; /*!< COMP4 inverting input select bit 1 */
constexpr uint32_t COMP4_CSR_COMP4INSEL_2          = 0x00000040; /*!< COMP4 inverting input select bit 2 */
constexpr uint32_t COMP4_CSR_COMP4INSEL_3          = 0x00400000; /*!< COMP4 inverting input select bit 3 */
constexpr uint32_t COMP4_CSR_COMP4OUTSEL           = 0x00003C00; /*!< COMP4 output select */
constexpr uint32_t COMP4_CSR_COMP4OUTSEL_0         = 0x00000400; /*!< COMP4 output select bit 0 */
constexpr uint32_t COMP4_CSR_COMP4OUTSEL_1         = 0x00000800; /*!< COMP4 output select bit 1 */
constexpr uint32_t COMP4_CSR_COMP4OUTSEL_2         = 0x00001000; /*!< COMP4 output select bit 2 */
constexpr uint32_t COMP4_CSR_COMP4OUTSEL_3         = 0x00002000; /*!< COMP4 output select bit 3 */
constexpr uint32_t COMP4_CSR_COMP4POL              = 0x00008000; /*!< COMP4 output polarity */
constexpr uint32_t COMP4_CSR_COMP4BLANKING         = 0x000C0000; /*!< COMP4 blanking */
constexpr uint32_t COMP4_CSR_COMP4BLANKING_0       = 0x00040000; /*!< COMP4 blanking bit 0 */
constexpr uint32_t COMP4_CSR_COMP4BLANKING_1       = 0x00080000; /*!< COMP4 blanking bit 1 */
constexpr uint32_t COMP4_CSR_COMP4BLANKING_2       = 0x00100000; /*!< COMP4 blanking bit 2 */
constexpr uint32_t COMP4_CSR_COMP4OUT              = 0x40000000; /*!< COMP4 output level */
constexpr uint32_t COMP4_CSR_COMP4LOCK             = 0x80000000; /*!< COMP4 lock */

/**********************  Bit definition for COMP5_CSR register  ***************/
constexpr uint32_t COMP5_CSR_COMP5EN               = 0x00000001; /*!< COMP5 enable */
constexpr uint32_t COMP5_CSR_COMP5INSEL            = 0x00000070; /*!< COMP5 inverting input select */
constexpr uint32_t COMP5_CSR_COMP5INSEL_0          = 0x00000010; /*!< COMP5 inverting input select bit 0 */
constexpr uint32_t COMP5_CSR_COMP5INSEL_1          = 0x00000020; /*!< COMP5 inverting input select bit 1 */
constexpr uint32_t COMP5_CSR_COMP5INSEL_2          = 0x00000040; /*!< COMP5 inverting input select bit 2 */
constexpr uint32_t COMP5_CSR_COMP5OUTSEL           = 0x00003C00; /*!< COMP5 output select */
constexpr uint32_t COMP5_CSR_COMP5OUTSEL_0         = 0x00000400; /*!< COMP5 output select bit 0 */
constexpr uint32_t COMP5_CSR_COMP5OUTSEL_1         = 0x00000800; /*!< COMP5 output select bit 1 */
constexpr uint32_t COMP5_CSR_COMP5OUTSEL_2         = 0x00001000; /*!< COMP5 output select bit 2 */
constexpr uint32_t COMP5_CSR_COMP5OUTSEL_3         = 0x00002000; /*!< COMP5 output select bit 3 */
constexpr uint32_t COMP5_CSR_COMP5POL              = 0x00008000; /*!< COMP5 output polarity */
constexpr uint32_t COMP5_CSR_COMP5BLANKING         = 0x000C0000; /*!< COMP5 blanking */
constexpr uint32_t COMP5_CSR_COMP5BLANKING_0       = 0x00040000; /*!< COMP5 blanking bit 0 */
constexpr uint32_t COMP5_CSR_COMP5BLANKING_1       = 0x00080000; /*!< COMP5 blanking bit 1 */
constexpr uint32_t COMP5_CSR_COMP5BLANKING_2       = 0x00100000; /*!< COMP5 blanking bit 2 */
constexpr uint32_t COMP5_CSR_COMP5OUT              = 0x40000000; /*!< COMP5 output level */
constexpr uint32_t COMP5_CSR_COMP5LOCK             = 0x80000000; /*!< COMP5 lock */

/**********************  Bit definition for COMP6_CSR register  ***************/
constexpr uint32_t COMP6_CSR_COMP6EN               = 0x00000001; /*!< COMP6 enable */
constexpr uint32_t COMP6_CSR_COMP6INSEL            = 0x00400070; /*!< COMP6 inverting input select */
constexpr uint32_t COMP6_CSR_COMP6INSEL_0          = 0x00000010; /*!< COMP6 inverting input select bit 0 */
constexpr uint32_t COMP6_CSR_COMP6INSEL_1          = 0x00000020; /*!< COMP6 inverting input select bit 1 */
constexpr uint32_t COMP6_CSR_COMP6INSEL_2          = 0x00000040; /*!< COMP6 inverting input select bit 2 */
constexpr uint32_t COMP6_CSR_COMP6INSEL_3          = 0x00400000; /*!< COMP6 inverting input select bit 3 */
constexpr uint32_t COMP6_CSR_COMP6OUTSEL           = 0x00003C00; /*!< COMP6 output select */
constexpr uint32_t COMP6_CSR_COMP6OUTSEL_0         = 0x00000400; /*!< COMP6 output select bit 0 */
constexpr uint32_t COMP6_CSR_COMP6OUTSEL_1         = 0x00000800; /*!< COMP6 output select bit 1 */
constexpr uint32_t COMP6_CSR_COMP6OUTSEL_2         = 0x00001000; /*!< COMP6 output select bit 2 */
constexpr uint32_t COMP6_CSR_COMP6OUTSEL_3         = 0x00002000; /*!< COMP6 output select bit 3 */
constexpr uint32_t COMP6_CSR_COMP6POL              = 0x00008000; /*!< COMP6 output polarity */
constexpr uint32_t COMP6_CSR_COMP6BLANKING         = 0x000C0000; /*!< COMP6 blanking */
constexpr uint32_t COMP6_CSR_COMP6BLANKING_0       = 0x00040000; /*!< COMP6 blanking bit 0 */
constexpr uint32_t COMP6_CSR_COMP6BLANKING_1       = 0x00080000; /*!< COMP6 blanking bit 1 */
constexpr uint32_t COMP6_CSR_COMP6BLANKING_2       = 0x00100000; /*!< COMP6 blanking bit 2 */
constexpr uint32_t COMP6_CSR_COMP6OUT              = 0x40000000; /*!< COMP6 output level */
constexpr uint32_t COMP6_CSR_COMP6LOCK             = 0x80000000; /*!< COMP6 lock */

/**********************  Bit definition for COMP7_CSR register  ***************/
constexpr uint32_t COMP7_CSR_COMP7EN               = 0x00000001; /*!< COMP7 enable */
constexpr uint32_t COMP7_CSR_COMP7INSEL            = 0x00000070; /*!< COMP7 inverting input select */
constexpr uint32_t COMP7_CSR_COMP7INSEL_0          = 0x00000010; /*!< COMP7 inverting input select bit 0 */
constexpr uint32_t COMP7_CSR_COMP7INSEL_1          = 0x00000020; /*!< COMP7 inverting input select bit 1 */
constexpr uint32_t COMP7_CSR_COMP7INSEL_2          = 0x00000040; /*!< COMP7 inverting input select bit 2 */
constexpr uint32_t COMP7_CSR_COMP7OUTSEL           = 0x00003C00; /*!< COMP7 output select */
constexpr uint32_t COMP7_CSR_COMP7OUTSEL_0         = 0x00000400; /*!< COMP7 output select bit 0 */
constexpr uint32_t COMP7_CSR_COMP7OUTSEL_1         = 0x00000800; /*!< COMP7 output select bit 1 */
constexpr uint32_t COMP7_CSR_COMP7OUTSEL_2         = 0x00001000; /*!< COMP7 output select bit 2 */
constexpr uint32_t COMP7_CSR_COMP7OUTSEL_3         = 0x00002000; /*!< COMP7 output select bit 3 */
constexpr uint32_t COMP7_CSR_COMP7POL              = 0x00008000; /*!< COMP7 output polarity */
constexpr uint32_t COMP7_CSR_COMP7BLANKING         = 0x000C0000; /*!< COMP7 blanking */
constexpr uint32_t COMP7_CSR_COMP7BLANKING_0       = 0x00040000; /*!< COMP7 blanking bit 0 */
constexpr uint32_t COMP7_CSR_COMP7BLANKING_1       = 0x00080000; /*!< COMP7 blanking bit 1 */
constexpr uint32_t COMP7_CSR_COMP7BLANKING_2       = 0x00100000; /*!< COMP7 blanking bit 2 */
constexpr uint32_t COMP7_CSR_COMP7OUT              = 0x40000000; /*!< COMP7 output level */
constexpr uint32_t COMP7_CSR_COMP7LOCK             = 0x80000000; /*!< COMP7 lock */

/**********************  Bit definition for COMP_CSR register  ****************/
constexpr uint32_t COMP_CSR_COMPxEN               = 0x00000001; /*!< COMPx enable */
constexpr uint32_t COMP_CSR_COMPxINSEL            = 0x00400070; /*!< COMPx inverting input select */
constexpr uint32_t COMP_CSR_COMPxINSEL_0          = 0x00000010; /*!< COMPx inverting input select bit 0 */
constexpr uint32_t COMP_CSR_COMPxINSEL_1          = 0x00000020; /*!< COMPx inverting input select bit 1 */
constexpr uint32_t COMP_CSR_COMPxINSEL_2          = 0x00000040; /*!< COMPx inverting input select bit 2 */
constexpr uint32_t COMP_CSR_COMPxINSEL_3          = 0x00400000; /*!< COMPx inverting input select bit 3 */
constexpr uint32_t COMP_CSR_COMPxOUTSEL           = 0x00003C00; /*!< COMPx output select */
constexpr uint32_t COMP_CSR_COMPxOUTSEL_0         = 0x00000400; /*!< COMPx output select bit 0 */
constexpr uint32_t COMP_CSR_COMPxOUTSEL_1         = 0x00000800; /*!< COMPx output select bit 1 */
constexpr uint32_t COMP_CSR_COMPxOUTSEL_2         = 0x00001000; /*!< COMPx output select bit 2 */
constexpr uint32_t COMP_CSR_COMPxOUTSEL_3         = 0x00002000; /*!< COMPx output select bit 3 */
constexpr uint32_t COMP_CSR_COMPxPOL              = 0x00008000; /*!< COMPx output polarity */
constexpr uint32_t COMP_CSR_COMPxBLANKING         = 0x000C0000; /*!< COMPx blanking */
constexpr uint32_t COMP_CSR_COMPxBLANKING_0       = 0x00040000; /*!< COMPx blanking bit 0 */
constexpr uint32_t COMP_CSR_COMPxBLANKING_1       = 0x00080000; /*!< COMPx blanking bit 1 */
constexpr uint32_t COMP_CSR_COMPxBLANKING_2       = 0x00100000; /*!< COMPx blanking bit 2 */
constexpr uint32_t COMP_CSR_COMPxOUT              = 0x40000000; /*!< COMPx output level */
constexpr uint32_t COMP_CSR_COMPxLOCK             = 0x80000000; /*!< COMPx lock */

/******************************************************************************/
/*                                                                            */
/*                     Operational Amplifier = OPAMP;                          */
/*                                                                            */
/******************************************************************************/
/*********************  Bit definition for OPAMP1_CSR register  ***************/
constexpr uint32_t OPAMP1_CSR_OPAMP1EN               = 0x00000001; /*!< OPAMP1 enable */
constexpr uint32_t OPAMP1_CSR_FORCEVP                = 0x00000002; /*!< Connect the internal references to the plus input of the OPAMPX */
constexpr uint32_t OPAMP1_CSR_VPSEL                  = 0x0000000C; /*!< Non inverting input selection */
constexpr uint32_t OPAMP1_CSR_VPSEL_0                = 0x00000004; /*!< Bit 0 */
constexpr uint32_t OPAMP1_CSR_VPSEL_1                = 0x00000008; /*!< Bit 1 */
constexpr uint32_t OPAMP1_CSR_VMSEL                  = 0x00000060; /*!< Inverting input selection */
constexpr uint32_t OPAMP1_CSR_VMSEL_0                = 0x00000020; /*!< Bit 0 */
constexpr uint32_t OPAMP1_CSR_VMSEL_1                = 0x00000040; /*!< Bit 1 */
constexpr uint32_t OPAMP1_CSR_TCMEN                  = 0x00000080; /*!< Timer-Controlled Mux mode enable */
constexpr uint32_t OPAMP1_CSR_VMSSEL                 = 0x00000100; /*!< Inverting input secondary selection */
constexpr uint32_t OPAMP1_CSR_VPSSEL                 = 0x00000600; /*!< Non inverting input secondary selection */
constexpr uint32_t OPAMP1_CSR_VPSSEL_0               = 0x00000200; /*!< Bit 0 */
constexpr uint32_t OPAMP1_CSR_VPSSEL_1               = 0x00000400; /*!< Bit 1 */
constexpr uint32_t OPAMP1_CSR_CALON                  = 0x00000800; /*!< Calibration mode enable */
constexpr uint32_t OPAMP1_CSR_CALSEL                 = 0x00003000; /*!< Calibration selection */
constexpr uint32_t OPAMP1_CSR_CALSEL_0               = 0x00001000; /*!< Bit 0 */
constexpr uint32_t OPAMP1_CSR_CALSEL_1               = 0x00002000; /*!< Bit 1 */
constexpr uint32_t OPAMP1_CSR_PGGAIN                 = 0x0003C000; /*!< Gain in PGA mode */
constexpr uint32_t OPAMP1_CSR_PGGAIN_0               = 0x00004000; /*!< Bit 0 */
constexpr uint32_t OPAMP1_CSR_PGGAIN_1               = 0x00008000; /*!< Bit 1 */
constexpr uint32_t OPAMP1_CSR_PGGAIN_2               = 0x00010000; /*!< Bit 2 */
constexpr uint32_t OPAMP1_CSR_PGGAIN_3               = 0x00020000; /*!< Bit 3 */
constexpr uint32_t OPAMP1_CSR_USERTRIM               = 0x00040000; /*!< User trimming enable */
constexpr uint32_t OPAMP1_CSR_TRIMOFFSETP            = 0x00F80000; /*!< Offset trimming value = PMOS; */
constexpr uint32_t OPAMP1_CSR_TRIMOFFSETN            = 0x1F000000; /*!< Offset trimming value = NMOS; */
constexpr uint32_t OPAMP1_CSR_TSTREF                 = 0x20000000; /*!< It enables the switch to put out the internal reference */
constexpr uint32_t OPAMP1_CSR_OUTCAL                 = 0x40000000; /*!< OPAMP ouput status flag */
constexpr uint32_t OPAMP1_CSR_LOCK                   = 0x80000000; /*!< OPAMP lock */

/*********************  Bit definition for OPAMP2_CSR register  ***************/
constexpr uint32_t OPAMP2_CSR_OPAMP2EN               = 0x00000001; /*!< OPAMP2 enable */
constexpr uint32_t OPAMP2_CSR_FORCEVP                = 0x00000002; /*!< Connect the internal references to the plus input of the OPAMPX */
constexpr uint32_t OPAMP2_CSR_VPSEL                  = 0x0000000C; /*!< Non inverting input selection */
constexpr uint32_t OPAMP2_CSR_VPSEL_0                = 0x00000004; /*!< Bit 0 */
constexpr uint32_t OPAMP2_CSR_VPSEL_1                = 0x00000008; /*!< Bit 1 */
constexpr uint32_t OPAMP2_CSR_VMSEL                  = 0x00000060; /*!< Inverting input selection */
constexpr uint32_t OPAMP2_CSR_VMSEL_0                = 0x00000020; /*!< Bit 0 */
constexpr uint32_t OPAMP2_CSR_VMSEL_1                = 0x00000040; /*!< Bit 1 */
constexpr uint32_t OPAMP2_CSR_TCMEN                  = 0x00000080; /*!< Timer-Controlled Mux mode enable */
constexpr uint32_t OPAMP2_CSR_VMSSEL                 = 0x00000100; /*!< Inverting input secondary selection */
constexpr uint32_t OPAMP2_CSR_VPSSEL                 = 0x00000600; /*!< Non inverting input secondary selection */
constexpr uint32_t OPAMP2_CSR_VPSSEL_0               = 0x00000200; /*!< Bit 0 */
constexpr uint32_t OPAMP2_CSR_VPSSEL_1               = 0x00000400; /*!< Bit 1 */
constexpr uint32_t OPAMP2_CSR_CALON                  = 0x00000800; /*!< Calibration mode enable */
constexpr uint32_t OPAMP2_CSR_CALSEL                 = 0x00003000; /*!< Calibration selection */
constexpr uint32_t OPAMP2_CSR_CALSEL_0               = 0x00001000; /*!< Bit 0 */
constexpr uint32_t OPAMP2_CSR_CALSEL_1               = 0x00002000; /*!< Bit 1 */
constexpr uint32_t OPAMP2_CSR_PGGAIN                 = 0x0003C000; /*!< Gain in PGA mode */
constexpr uint32_t OPAMP2_CSR_PGGAIN_0               = 0x00004000; /*!< Bit 0 */
constexpr uint32_t OPAMP2_CSR_PGGAIN_1               = 0x00008000; /*!< Bit 1 */
constexpr uint32_t OPAMP2_CSR_PGGAIN_2               = 0x00010000; /*!< Bit 2 */
constexpr uint32_t OPAMP2_CSR_PGGAIN_3               = 0x00020000; /*!< Bit 3 */
constexpr uint32_t OPAMP2_CSR_USERTRIM               = 0x00040000; /*!< User trimming enable */
constexpr uint32_t OPAMP2_CSR_TRIMOFFSETP            = 0x00F80000; /*!< Offset trimming value = PMOS; */
constexpr uint32_t OPAMP2_CSR_TRIMOFFSETN            = 0x1F000000; /*!< Offset trimming value = NMOS; */
constexpr uint32_t OPAMP2_CSR_TSTREF                 = 0x20000000; /*!< It enables the switch to put out the internal reference */
constexpr uint32_t OPAMP2_CSR_OUTCAL                 = 0x40000000; /*!< OPAMP ouput status flag */
constexpr uint32_t OPAMP2_CSR_LOCK                   = 0x80000000; /*!< OPAMP lock */

/*********************  Bit definition for OPAMP3_CSR register  ***************/
constexpr uint32_t OPAMP3_CSR_OPAMP3EN               = 0x00000001; /*!< OPAMP3 enable */
constexpr uint32_t OPAMP3_CSR_FORCEVP                = 0x00000002; /*!< Connect the internal references to the plus input of the OPAMPX */
constexpr uint32_t OPAMP3_CSR_VPSEL                  = 0x0000000C; /*!< Non inverting input selection */
constexpr uint32_t OPAMP3_CSR_VPSEL_0                = 0x00000004; /*!< Bit 0 */
constexpr uint32_t OPAMP3_CSR_VPSEL_1                = 0x00000008; /*!< Bit 1 */
constexpr uint32_t OPAMP3_CSR_VMSEL                  = 0x00000060; /*!< Inverting input selection */
constexpr uint32_t OPAMP3_CSR_VMSEL_0                = 0x00000020; /*!< Bit 0 */
constexpr uint32_t OPAMP3_CSR_VMSEL_1                = 0x00000040; /*!< Bit 1 */
constexpr uint32_t OPAMP3_CSR_TCMEN                  = 0x00000080; /*!< Timer-Controlled Mux mode enable */
constexpr uint32_t OPAMP3_CSR_VMSSEL                 = 0x00000100; /*!< Inverting input secondary selection */
constexpr uint32_t OPAMP3_CSR_VPSSEL                 = 0x00000600; /*!< Non inverting input secondary selection */
constexpr uint32_t OPAMP3_CSR_VPSSEL_0               = 0x00000200; /*!< Bit 0 */
constexpr uint32_t OPAMP3_CSR_VPSSEL_1               = 0x00000400; /*!< Bit 1 */
constexpr uint32_t OPAMP3_CSR_CALON                  = 0x00000800; /*!< Calibration mode enable */
constexpr uint32_t OPAMP3_CSR_CALSEL                 = 0x00003000; /*!< Calibration selection */
constexpr uint32_t OPAMP3_CSR_CALSEL_0               = 0x00001000; /*!< Bit 0 */
constexpr uint32_t OPAMP3_CSR_CALSEL_1               = 0x00002000; /*!< Bit 1 */
constexpr uint32_t OPAMP3_CSR_PGGAIN                 = 0x0003C000; /*!< Gain in PGA mode */
constexpr uint32_t OPAMP3_CSR_PGGAIN_0               = 0x00004000; /*!< Bit 0 */
constexpr uint32_t OPAMP3_CSR_PGGAIN_1               = 0x00008000; /*!< Bit 1 */
constexpr uint32_t OPAMP3_CSR_PGGAIN_2               = 0x00010000; /*!< Bit 2 */
constexpr uint32_t OPAMP3_CSR_PGGAIN_3               = 0x00020000; /*!< Bit 3 */
constexpr uint32_t OPAMP3_CSR_USERTRIM               = 0x00040000; /*!< User trimming enable */
constexpr uint32_t OPAMP3_CSR_TRIMOFFSETP            = 0x00F80000; /*!< Offset trimming value = PMOS; */
constexpr uint32_t OPAMP3_CSR_TRIMOFFSETN            = 0x1F000000; /*!< Offset trimming value = NMOS; */
constexpr uint32_t OPAMP3_CSR_TSTREF                 = 0x20000000; /*!< It enables the switch to put out the internal reference */
constexpr uint32_t OPAMP3_CSR_OUTCAL                 = 0x40000000; /*!< OPAMP ouput status flag */
constexpr uint32_t OPAMP3_CSR_LOCK                   = 0x80000000; /*!< OPAMP lock */

/*********************  Bit definition for OPAMP4_CSR register  ***************/
constexpr uint32_t OPAMP4_CSR_OPAMP4EN               = 0x00000001; /*!< OPAMP4 enable */
constexpr uint32_t OPAMP4_CSR_FORCEVP                = 0x00000002; /*!< Connect the internal references to the plus input of the OPAMPX */
constexpr uint32_t OPAMP4_CSR_VPSEL                  = 0x0000000C; /*!< Non inverting input selection */
constexpr uint32_t OPAMP4_CSR_VPSEL_0                = 0x00000004; /*!< Bit 0 */
constexpr uint32_t OPAMP4_CSR_VPSEL_1                = 0x00000008; /*!< Bit 1 */
constexpr uint32_t OPAMP4_CSR_VMSEL                  = 0x00000060; /*!< Inverting input selection */
constexpr uint32_t OPAMP4_CSR_VMSEL_0                = 0x00000020; /*!< Bit 0 */
constexpr uint32_t OPAMP4_CSR_VMSEL_1                = 0x00000040; /*!< Bit 1 */
constexpr uint32_t OPAMP4_CSR_TCMEN                  = 0x00000080; /*!< Timer-Controlled Mux mode enable */
constexpr uint32_t OPAMP4_CSR_VMSSEL                 = 0x00000100; /*!< Inverting input secondary selection */
constexpr uint32_t OPAMP4_CSR_VPSSEL                 = 0x00000600; /*!< Non inverting input secondary selection */
constexpr uint32_t OPAMP4_CSR_VPSSEL_0               = 0x00000200; /*!< Bit 0 */
constexpr uint32_t OPAMP4_CSR_VPSSEL_1               = 0x00000400; /*!< Bit 1 */
constexpr uint32_t OPAMP4_CSR_CALON                  = 0x00000800; /*!< Calibration mode enable */
constexpr uint32_t OPAMP4_CSR_CALSEL                 = 0x00003000; /*!< Calibration selection */
constexpr uint32_t OPAMP4_CSR_CALSEL_0               = 0x00001000; /*!< Bit 0 */
constexpr uint32_t OPAMP4_CSR_CALSEL_1               = 0x00002000; /*!< Bit 1 */
constexpr uint32_t OPAMP4_CSR_PGGAIN                 = 0x0003C000; /*!< Gain in PGA mode */
constexpr uint32_t OPAMP4_CSR_PGGAIN_0               = 0x00004000; /*!< Bit 0 */
constexpr uint32_t OPAMP4_CSR_PGGAIN_1               = 0x00008000; /*!< Bit 1 */
constexpr uint32_t OPAMP4_CSR_PGGAIN_2               = 0x00010000; /*!< Bit 2 */
constexpr uint32_t OPAMP4_CSR_PGGAIN_3               = 0x00020000; /*!< Bit 3 */
constexpr uint32_t OPAMP4_CSR_USERTRIM               = 0x00040000; /*!< User trimming enable */
constexpr uint32_t OPAMP4_CSR_TRIMOFFSETP            = 0x00F80000; /*!< Offset trimming value = PMOS; */
constexpr uint32_t OPAMP4_CSR_TRIMOFFSETN            = 0x1F000000; /*!< Offset trimming value = NMOS; */
constexpr uint32_t OPAMP4_CSR_TSTREF                 = 0x20000000; /*!< It enables the switch to put out the internal reference */
constexpr uint32_t OPAMP4_CSR_OUTCAL                 = 0x40000000; /*!< OPAMP ouput status flag */
constexpr uint32_t OPAMP4_CSR_LOCK                   = 0x80000000; /*!< OPAMP lock */

/*********************  Bit definition for OPAMPx_CSR register  ***************/
constexpr uint32_t OPAMP_CSR_OPAMPxEN               = 0x00000001; /*!< OPAMP enable */
constexpr uint32_t OPAMP_CSR_FORCEVP                = 0x00000002; /*!< Connect the internal references to the plus input of the OPAMPX */
constexpr uint32_t OPAMP_CSR_VPSEL                  = 0x0000000C; /*!< Non inverting input selection */
constexpr uint32_t OPAMP_CSR_VPSEL_0                = 0x00000004; /*!< Bit 0 */
constexpr uint32_t OPAMP_CSR_VPSEL_1                = 0x00000008; /*!< Bit 1 */
constexpr uint32_t OPAMP_CSR_VMSEL                  = 0x00000060; /*!< Inverting input selection */
constexpr uint32_t OPAMP_CSR_VMSEL_0                = 0x00000020; /*!< Bit 0 */
constexpr uint32_t OPAMP_CSR_VMSEL_1                = 0x00000040; /*!< Bit 1 */
constexpr uint32_t OPAMP_CSR_TCMEN                  = 0x00000080; /*!< Timer-Controlled Mux mode enable */
constexpr uint32_t OPAMP_CSR_VMSSEL                 = 0x00000100; /*!< Inverting input secondary selection */
constexpr uint32_t OPAMP_CSR_VPSSEL                 = 0x00000600; /*!< Non inverting input secondary selection */
constexpr uint32_t OPAMP_CSR_VPSSEL_0               = 0x00000200; /*!< Bit 0 */
constexpr uint32_t OPAMP_CSR_VPSSEL_1               = 0x00000400; /*!< Bit 1 */
constexpr uint32_t OPAMP_CSR_CALON                  = 0x00000800; /*!< Calibration mode enable */
constexpr uint32_t OPAMP_CSR_CALSEL                 = 0x00003000; /*!< Calibration selection */
constexpr uint32_t OPAMP_CSR_CALSEL_0               = 0x00001000; /*!< Bit 0 */
constexpr uint32_t OPAMP_CSR_CALSEL_1               = 0x00002000; /*!< Bit 1 */
constexpr uint32_t OPAMP_CSR_PGGAIN                 = 0x0003C000; /*!< Gain in PGA mode */
constexpr uint32_t OPAMP_CSR_PGGAIN_0               = 0x00004000; /*!< Bit 0 */
constexpr uint32_t OPAMP_CSR_PGGAIN_1               = 0x00008000; /*!< Bit 1 */
constexpr uint32_t OPAMP_CSR_PGGAIN_2               = 0x00010000; /*!< Bit 2 */
constexpr uint32_t OPAMP_CSR_PGGAIN_3               = 0x00020000; /*!< Bit 3 */
constexpr uint32_t OPAMP_CSR_USERTRIM               = 0x00040000; /*!< User trimming enable */
constexpr uint32_t OPAMP_CSR_TRIMOFFSETP            = 0x00F80000; /*!< Offset trimming value = PMOS; */
constexpr uint32_t OPAMP_CSR_TRIMOFFSETN            = 0x1F000000; /*!< Offset trimming value = NMOS; */
constexpr uint32_t OPAMP_CSR_TSTREF                 = 0x20000000; /*!< It enables the switch to put out the internal reference */
constexpr uint32_t OPAMP_CSR_OUTCAL                 = 0x40000000; /*!< OPAMP ouput status flag */
constexpr uint32_t OPAMP_CSR_LOCK                   = 0x80000000; /*!< OPAMP lock */

/******************************************************************************/
/*                                                                            */
/*                   Controller Area Network = CAN ;                           */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CAN_MCR register  ********************/
constexpr uint32_t  CAN_MCR_INRQ                        = 0x00000001;        /*!<Initialization Request */
constexpr uint32_t  CAN_MCR_SLEEP                       = 0x00000002;        /*!<Sleep Mode Request */
constexpr uint32_t  CAN_MCR_TXFP                        = 0x00000004;        /*!<Transmit FIFO Priority */
constexpr uint32_t  CAN_MCR_RFLM                        = 0x00000008;        /*!<Receive FIFO Locked Mode */
constexpr uint32_t  CAN_MCR_NART                        = 0x00000010;        /*!<No Automatic Retransmission */
constexpr uint32_t  CAN_MCR_AWUM                        = 0x00000020;        /*!<Automatic Wakeup Mode */
constexpr uint32_t  CAN_MCR_ABOM                        = 0x00000040;        /*!<Automatic Bus-Off Management */
constexpr uint32_t  CAN_MCR_TTCM                        = 0x00000080;        /*!<Time Triggered Communication Mode */
constexpr uint32_t  CAN_MCR_RESET                       = 0x00008000;        /*!<bxCAN software master reset */

/*******************  Bit definition for CAN_MSR register  ********************/
constexpr uint32_t  CAN_MSR_INAK                        = 0x00000001;        /*!<Initialization Acknowledge */
constexpr uint32_t  CAN_MSR_SLAK                        = 0x00000002;        /*!<Sleep Acknowledge */
constexpr uint32_t  CAN_MSR_ERRI                        = 0x00000004;        /*!<Error Interrupt */
constexpr uint32_t  CAN_MSR_WKUI                        = 0x00000008;        /*!<Wakeup Interrupt */
constexpr uint32_t  CAN_MSR_SLAKI                       = 0x00000010;        /*!<Sleep Acknowledge Interrupt */
constexpr uint32_t  CAN_MSR_TXM                         = 0x00000100;        /*!<Transmit Mode */
constexpr uint32_t  CAN_MSR_RXM                         = 0x00000200;        /*!<Receive Mode */
constexpr uint32_t  CAN_MSR_SAMP                        = 0x00000400;        /*!<Last Sample Point */
constexpr uint32_t  CAN_MSR_RX                          = 0x00000800;        /*!<CAN Rx Signal */

/*******************  Bit definition for CAN_TSR register  ********************/
constexpr uint32_t  CAN_TSR_RQCP0                       = 0x00000001;        /*!<Request Completed Mailbox0 */
constexpr uint32_t  CAN_TSR_TXOK0                       = 0x00000002;        /*!<Transmission OK of Mailbox0 */
constexpr uint32_t  CAN_TSR_ALST0                       = 0x00000004;        /*!<Arbitration Lost for Mailbox0 */
constexpr uint32_t  CAN_TSR_TERR0                       = 0x00000008;        /*!<Transmission Error of Mailbox0 */
constexpr uint32_t  CAN_TSR_ABRQ0                       = 0x00000080;        /*!<Abort Request for Mailbox0 */
constexpr uint32_t  CAN_TSR_RQCP1                       = 0x00000100;        /*!<Request Completed Mailbox1 */
constexpr uint32_t  CAN_TSR_TXOK1                       = 0x00000200;        /*!<Transmission OK of Mailbox1 */
constexpr uint32_t  CAN_TSR_ALST1                       = 0x00000400;        /*!<Arbitration Lost for Mailbox1 */
constexpr uint32_t  CAN_TSR_TERR1                       = 0x00000800;        /*!<Transmission Error of Mailbox1 */
constexpr uint32_t  CAN_TSR_ABRQ1                       = 0x00008000;        /*!<Abort Request for Mailbox 1 */
constexpr uint32_t  CAN_TSR_RQCP2                       = 0x00010000;        /*!<Request Completed Mailbox2 */
constexpr uint32_t  CAN_TSR_TXOK2                       = 0x00020000;        /*!<Transmission OK of Mailbox 2 */
constexpr uint32_t  CAN_TSR_ALST2                       = 0x00040000;        /*!<Arbitration Lost for mailbox 2 */
constexpr uint32_t  CAN_TSR_TERR2                       = 0x00080000;        /*!<Transmission Error of Mailbox 2 */
constexpr uint32_t  CAN_TSR_ABRQ2                       = 0x00800000;        /*!<Abort Request for Mailbox 2 */
constexpr uint32_t  CAN_TSR_CODE                        = 0x03000000;        /*!<Mailbox Code */

constexpr uint32_t  CAN_TSR_TME                         = 0x1C000000;        /*!<TME[2:0] bits */
constexpr uint32_t  CAN_TSR_TME0                        = 0x04000000;        /*!<Transmit Mailbox 0 Empty */
constexpr uint32_t  CAN_TSR_TME1                        = 0x08000000;        /*!<Transmit Mailbox 1 Empty */
constexpr uint32_t  CAN_TSR_TME2                        = 0x10000000;        /*!<Transmit Mailbox 2 Empty */

constexpr uint32_t  CAN_TSR_LOW                         = 0xE0000000;        /*!<LOW[2:0] bits */
constexpr uint32_t  CAN_TSR_LOW0                        = 0x20000000;        /*!<Lowest Priority Flag for Mailbox 0 */
constexpr uint32_t  CAN_TSR_LOW1                        = 0x40000000;        /*!<Lowest Priority Flag for Mailbox 1 */
constexpr uint32_t  CAN_TSR_LOW2                        = 0x80000000;        /*!<Lowest Priority Flag for Mailbox 2 */

/*******************  Bit definition for CAN_RF0R register  *******************/
constexpr uint32_t  CAN_RF0R_FMP0                       = 0x00000003;        /*!<FIFO 0 Message Pending */
constexpr uint32_t  CAN_RF0R_FULL0                      = 0x00000008;        /*!<FIFO 0 Full */
constexpr uint32_t  CAN_RF0R_FOVR0                      = 0x00000010;        /*!<FIFO 0 Overrun */
constexpr uint32_t  CAN_RF0R_RFOM0                      = 0x00000020;        /*!<Release FIFO 0 Output Mailbox */

/*******************  Bit definition for CAN_RF1R register  *******************/
constexpr uint32_t  CAN_RF1R_FMP1                       = 0x00000003;        /*!<FIFO 1 Message Pending */
constexpr uint32_t  CAN_RF1R_FULL1                      = 0x00000008;        /*!<FIFO 1 Full */
constexpr uint32_t  CAN_RF1R_FOVR1                      = 0x00000010;        /*!<FIFO 1 Overrun */
constexpr uint32_t  CAN_RF1R_RFOM1                      = 0x00000020;        /*!<Release FIFO 1 Output Mailbox */

/********************  Bit definition for CAN_IER register  *******************/
constexpr uint32_t  CAN_IER_TMEIE                       = 0x00000001;        /*!<Transmit Mailbox Empty Interrupt Enable */
constexpr uint32_t  CAN_IER_FMPIE0                      = 0x00000002;        /*!<FIFO Message Pending Interrupt Enable */
constexpr uint32_t  CAN_IER_FFIE0                       = 0x00000004;        /*!<FIFO Full Interrupt Enable */
constexpr uint32_t  CAN_IER_FOVIE0                      = 0x00000008;        /*!<FIFO Overrun Interrupt Enable */
constexpr uint32_t  CAN_IER_FMPIE1                      = 0x00000010;        /*!<FIFO Message Pending Interrupt Enable */
constexpr uint32_t  CAN_IER_FFIE1                       = 0x00000020;        /*!<FIFO Full Interrupt Enable */
constexpr uint32_t  CAN_IER_FOVIE1                      = 0x00000040;        /*!<FIFO Overrun Interrupt Enable */
constexpr uint32_t  CAN_IER_EWGIE                       = 0x00000100;        /*!<Error Warning Interrupt Enable */
constexpr uint32_t  CAN_IER_EPVIE                       = 0x00000200;        /*!<Error Passive Interrupt Enable */
constexpr uint32_t  CAN_IER_BOFIE                       = 0x00000400;        /*!<Bus-Off Interrupt Enable */
constexpr uint32_t  CAN_IER_LECIE                       = 0x00000800;        /*!<Last Error Code Interrupt Enable */
constexpr uint32_t  CAN_IER_ERRIE                       = 0x00008000;        /*!<Error Interrupt Enable */
constexpr uint32_t  CAN_IER_WKUIE                       = 0x00010000;        /*!<Wakeup Interrupt Enable */
constexpr uint32_t  CAN_IER_SLKIE                       = 0x00020000;        /*!<Sleep Interrupt Enable */

/********************  Bit definition for CAN_ESR register  *******************/
constexpr uint32_t  CAN_ESR_EWGF                        = 0x00000001;        /*!<Error Warning Flag */
constexpr uint32_t  CAN_ESR_EPVF                        = 0x00000002;        /*!<Error Passive Flag */
constexpr uint32_t  CAN_ESR_BOFF                        = 0x00000004;        /*!<Bus-Off Flag */

constexpr uint32_t  CAN_ESR_LEC                         = 0x00000070;        /*!<LEC[2:0] bits = Last Error Code; */
constexpr uint32_t  CAN_ESR_LEC_0                       = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  CAN_ESR_LEC_1                       = 0x00000020;        /*!<Bit 1 */
constexpr uint32_t  CAN_ESR_LEC_2                       = 0x00000040;        /*!<Bit 2 */

constexpr uint32_t  CAN_ESR_TEC                         = 0x00FF0000;        /*!<Least significant byte of the 9-bit Transmit Error Counter */
constexpr uint32_t  CAN_ESR_REC                         = 0xFF000000;        /*!<Receive Error Counter */

/*******************  Bit definition for CAN_BTR register  ********************/
constexpr uint32_t  CAN_BTR_BRP                         = 0x000003FF;        /*!<Baud Rate Prescaler */
constexpr uint32_t  CAN_BTR_TS1                         = 0x000F0000;        /*!<Time Segment 1 */
constexpr uint32_t  CAN_BTR_TS1_0                       = 0x00010000;        /*!<Time Segment 1 = Bit 0; */
constexpr uint32_t  CAN_BTR_TS1_1                       = 0x00020000;        /*!<Time Segment 1 = Bit 1; */
constexpr uint32_t  CAN_BTR_TS1_2                       = 0x00040000;        /*!<Time Segment 1 = Bit 2; */
constexpr uint32_t  CAN_BTR_TS1_3                       = 0x00080000;        /*!<Time Segment 1 = Bit 3; */
constexpr uint32_t  CAN_BTR_TS2                         = 0x00700000;        /*!<Time Segment 2 */
constexpr uint32_t  CAN_BTR_TS2_0                       = 0x00100000;        /*!<Time Segment 2 = Bit 0; */
constexpr uint32_t  CAN_BTR_TS2_1                       = 0x00200000;        /*!<Time Segment 2 = Bit 1; */
constexpr uint32_t  CAN_BTR_TS2_2                       = 0x00400000;        /*!<Time Segment 2 = Bit 2; */
constexpr uint32_t  CAN_BTR_SJW                         = 0x03000000;        /*!<Resynchronization Jump Width */
constexpr uint32_t  CAN_BTR_SJW_0                       = 0x01000000;        /*!<Resynchronization Jump Width = Bit 0; */
constexpr uint32_t  CAN_BTR_SJW_1                       = 0x02000000;        /*!<Resynchronization Jump Width = Bit 1; */
constexpr uint32_t  CAN_BTR_LBKM                        = 0x40000000;        /*!<Loop Back Mode = Debug; */
constexpr uint32_t  CAN_BTR_SILM                        = 0x80000000;        /*!<Silent Mode */

/*!<Mailbox registers */
/******************  Bit definition for CAN_TI0R register  ********************/
constexpr uint32_t  CAN_TI0R_TXRQ                       = 0x00000001;        /*!<Transmit Mailbox Request */
constexpr uint32_t  CAN_TI0R_RTR                        = 0x00000002;        /*!<Remote Transmission Request */
constexpr uint32_t  CAN_TI0R_IDE                        = 0x00000004;        /*!<Identifier Extension */
constexpr uint32_t  CAN_TI0R_EXID                       = 0x001FFFF8;        /*!<Extended Identifier */
constexpr uint32_t  CAN_TI0R_STID                       = 0xFFE00000;        /*!<Standard Identifier or Extended Identifier */

/******************  Bit definition for CAN_TDT0R register  *******************/
constexpr uint32_t  CAN_TDT0R_DLC                       = 0x0000000F;        /*!<Data Length Code */
constexpr uint32_t  CAN_TDT0R_TGT                       = 0x00000100;        /*!<Transmit Global Time */
constexpr uint32_t  CAN_TDT0R_TIME                      = 0xFFFF0000;        /*!<Message Time Stamp */

/******************  Bit definition for CAN_TDL0R register  *******************/
constexpr uint32_t  CAN_TDL0R_DATA0                     = 0x000000FF;        /*!<Data byte 0 */
constexpr uint32_t  CAN_TDL0R_DATA1                     = 0x0000FF00;        /*!<Data byte 1 */
constexpr uint32_t  CAN_TDL0R_DATA2                     = 0x00FF0000;        /*!<Data byte 2 */
constexpr uint32_t  CAN_TDL0R_DATA3                     = 0xFF000000;        /*!<Data byte 3 */

/******************  Bit definition for CAN_TDH0R register  *******************/
constexpr uint32_t  CAN_TDH0R_DATA4                     = 0x000000FF;        /*!<Data byte 4 */
constexpr uint32_t  CAN_TDH0R_DATA5                     = 0x0000FF00;        /*!<Data byte 5 */
constexpr uint32_t  CAN_TDH0R_DATA6                     = 0x00FF0000;        /*!<Data byte 6 */
constexpr uint32_t  CAN_TDH0R_DATA7                     = 0xFF000000;        /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI1R register  *******************/
constexpr uint32_t  CAN_TI1R_TXRQ                       = 0x00000001;        /*!<Transmit Mailbox Request */
constexpr uint32_t  CAN_TI1R_RTR                        = 0x00000002;        /*!<Remote Transmission Request */
constexpr uint32_t  CAN_TI1R_IDE                        = 0x00000004;        /*!<Identifier Extension */
constexpr uint32_t  CAN_TI1R_EXID                       = 0x001FFFF8;        /*!<Extended Identifier */
constexpr uint32_t  CAN_TI1R_STID                       = 0xFFE00000;        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT1R register  ******************/
constexpr uint32_t  CAN_TDT1R_DLC                       = 0x0000000F;        /*!<Data Length Code */
constexpr uint32_t  CAN_TDT1R_TGT                       = 0x00000100;        /*!<Transmit Global Time */
constexpr uint32_t  CAN_TDT1R_TIME                      = 0xFFFF0000;        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL1R register  ******************/
constexpr uint32_t  CAN_TDL1R_DATA0                     = 0x000000FF;        /*!<Data byte 0 */
constexpr uint32_t  CAN_TDL1R_DATA1                     = 0x0000FF00;        /*!<Data byte 1 */
constexpr uint32_t  CAN_TDL1R_DATA2                     = 0x00FF0000;        /*!<Data byte 2 */
constexpr uint32_t  CAN_TDL1R_DATA3                     = 0xFF000000;        /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH1R register  ******************/
constexpr uint32_t  CAN_TDH1R_DATA4                     = 0x000000FF;        /*!<Data byte 4 */
constexpr uint32_t  CAN_TDH1R_DATA5                     = 0x0000FF00;        /*!<Data byte 5 */
constexpr uint32_t  CAN_TDH1R_DATA6                     = 0x00FF0000;        /*!<Data byte 6 */
constexpr uint32_t  CAN_TDH1R_DATA7                     = 0xFF000000;        /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI2R register  *******************/
constexpr uint32_t  CAN_TI2R_TXRQ                       = 0x00000001;        /*!<Transmit Mailbox Request */
constexpr uint32_t  CAN_TI2R_RTR                        = 0x00000002;        /*!<Remote Transmission Request */
constexpr uint32_t  CAN_TI2R_IDE                        = 0x00000004;        /*!<Identifier Extension */
constexpr uint32_t  CAN_TI2R_EXID                       = 0x001FFFF8;        /*!<Extended identifier */
constexpr uint32_t  CAN_TI2R_STID                       = 0xFFE00000;        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT2R register  ******************/
constexpr uint32_t  CAN_TDT2R_DLC                       = 0x0000000F;        /*!<Data Length Code */
constexpr uint32_t  CAN_TDT2R_TGT                       = 0x00000100;        /*!<Transmit Global Time */
constexpr uint32_t  CAN_TDT2R_TIME                      = 0xFFFF0000;        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL2R register  ******************/
constexpr uint32_t  CAN_TDL2R_DATA0                     = 0x000000FF;        /*!<Data byte 0 */
constexpr uint32_t  CAN_TDL2R_DATA1                     = 0x0000FF00;        /*!<Data byte 1 */
constexpr uint32_t  CAN_TDL2R_DATA2                     = 0x00FF0000;        /*!<Data byte 2 */
constexpr uint32_t  CAN_TDL2R_DATA3                     = 0xFF000000;        /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH2R register  ******************/
constexpr uint32_t  CAN_TDH2R_DATA4                     = 0x000000FF;        /*!<Data byte 4 */
constexpr uint32_t  CAN_TDH2R_DATA5                     = 0x0000FF00;        /*!<Data byte 5 */
constexpr uint32_t  CAN_TDH2R_DATA6                     = 0x00FF0000;        /*!<Data byte 6 */
constexpr uint32_t  CAN_TDH2R_DATA7                     = 0xFF000000;        /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI0R register  *******************/
constexpr uint32_t  CAN_RI0R_RTR                        = 0x00000002;        /*!<Remote Transmission Request */
constexpr uint32_t  CAN_RI0R_IDE                        = 0x00000004;        /*!<Identifier Extension */
constexpr uint32_t  CAN_RI0R_EXID                       = 0x001FFFF8;        /*!<Extended Identifier */
constexpr uint32_t  CAN_RI0R_STID                       = 0xFFE00000;        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT0R register  ******************/
constexpr uint32_t  CAN_RDT0R_DLC                       = 0x0000000F;        /*!<Data Length Code */
constexpr uint32_t  CAN_RDT0R_FMI                       = 0x0000FF00;        /*!<Filter Match Index */
constexpr uint32_t  CAN_RDT0R_TIME                      = 0xFFFF0000;        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL0R register  ******************/
constexpr uint32_t  CAN_RDL0R_DATA0                     = 0x000000FF;        /*!<Data byte 0 */
constexpr uint32_t  CAN_RDL0R_DATA1                     = 0x0000FF00;        /*!<Data byte 1 */
constexpr uint32_t  CAN_RDL0R_DATA2                     = 0x00FF0000;        /*!<Data byte 2 */
constexpr uint32_t  CAN_RDL0R_DATA3                     = 0xFF000000;        /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH0R register  ******************/
constexpr uint32_t  CAN_RDH0R_DATA4                     = 0x000000FF;        /*!<Data byte 4 */
constexpr uint32_t  CAN_RDH0R_DATA5                     = 0x0000FF00;        /*!<Data byte 5 */
constexpr uint32_t  CAN_RDH0R_DATA6                     = 0x00FF0000;        /*!<Data byte 6 */
constexpr uint32_t  CAN_RDH0R_DATA7                     = 0xFF000000;        /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI1R register  *******************/
constexpr uint32_t  CAN_RI1R_RTR                        = 0x00000002;        /*!<Remote Transmission Request */
constexpr uint32_t  CAN_RI1R_IDE                        = 0x00000004;        /*!<Identifier Extension */
constexpr uint32_t  CAN_RI1R_EXID                       = 0x001FFFF8;        /*!<Extended identifier */
constexpr uint32_t  CAN_RI1R_STID                       = 0xFFE00000;        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT1R register  ******************/
constexpr uint32_t  CAN_RDT1R_DLC                       = 0x0000000F;        /*!<Data Length Code */
constexpr uint32_t  CAN_RDT1R_FMI                       = 0x0000FF00;        /*!<Filter Match Index */
constexpr uint32_t  CAN_RDT1R_TIME                      = 0xFFFF0000;        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL1R register  ******************/
constexpr uint32_t  CAN_RDL1R_DATA0                     = 0x000000FF;        /*!<Data byte 0 */
constexpr uint32_t  CAN_RDL1R_DATA1                     = 0x0000FF00;        /*!<Data byte 1 */
constexpr uint32_t  CAN_RDL1R_DATA2                     = 0x00FF0000;        /*!<Data byte 2 */
constexpr uint32_t  CAN_RDL1R_DATA3                     = 0xFF000000;        /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH1R register  ******************/
constexpr uint32_t  CAN_RDH1R_DATA4                     = 0x000000FF;        /*!<Data byte 4 */
constexpr uint32_t  CAN_RDH1R_DATA5                     = 0x0000FF00;        /*!<Data byte 5 */
constexpr uint32_t  CAN_RDH1R_DATA6                     = 0x00FF0000;        /*!<Data byte 6 */
constexpr uint32_t  CAN_RDH1R_DATA7                     = 0xFF000000;        /*!<Data byte 7 */

/*!<CAN filter registers */
/*******************  Bit definition for CAN_FMR register  ********************/
constexpr uint32_t  CAN_FMR_FINIT                       = 0x00000001;        /*!<Filter Init Mode */

/*******************  Bit definition for CAN_FM1R register  *******************/
constexpr uint32_t  CAN_FM1R_FBM                        = 0x00003FFF;        /*!<Filter Mode */
constexpr uint32_t  CAN_FM1R_FBM0                       = 0x00000001;        /*!<Filter Init Mode bit 0 */
constexpr uint32_t  CAN_FM1R_FBM1                       = 0x00000002;        /*!<Filter Init Mode bit 1 */
constexpr uint32_t  CAN_FM1R_FBM2                       = 0x00000004;        /*!<Filter Init Mode bit 2 */
constexpr uint32_t  CAN_FM1R_FBM3                       = 0x00000008;        /*!<Filter Init Mode bit 3 */
constexpr uint32_t  CAN_FM1R_FBM4                       = 0x00000010;        /*!<Filter Init Mode bit 4 */
constexpr uint32_t  CAN_FM1R_FBM5                       = 0x00000020;        /*!<Filter Init Mode bit 5 */
constexpr uint32_t  CAN_FM1R_FBM6                       = 0x00000040;        /*!<Filter Init Mode bit 6 */
constexpr uint32_t  CAN_FM1R_FBM7                       = 0x00000080;        /*!<Filter Init Mode bit 7 */
constexpr uint32_t  CAN_FM1R_FBM8                       = 0x00000100;        /*!<Filter Init Mode bit 8 */
constexpr uint32_t  CAN_FM1R_FBM9                       = 0x00000200;        /*!<Filter Init Mode bit 9 */
constexpr uint32_t  CAN_FM1R_FBM10                      = 0x00000400;        /*!<Filter Init Mode bit 10 */
constexpr uint32_t  CAN_FM1R_FBM11                      = 0x00000800;        /*!<Filter Init Mode bit 11 */
constexpr uint32_t  CAN_FM1R_FBM12                      = 0x00001000;        /*!<Filter Init Mode bit 12 */
constexpr uint32_t  CAN_FM1R_FBM13                      = 0x00002000;        /*!<Filter Init Mode bit 13 */

/*******************  Bit definition for CAN_FS1R register  *******************/
constexpr uint32_t  CAN_FS1R_FSC                        = 0x00003FFF;        /*!<Filter Scale Configuration */
constexpr uint32_t  CAN_FS1R_FSC0                       = 0x00000001;        /*!<Filter Scale Configuration bit 0 */
constexpr uint32_t  CAN_FS1R_FSC1                       = 0x00000002;        /*!<Filter Scale Configuration bit 1 */
constexpr uint32_t  CAN_FS1R_FSC2                       = 0x00000004;        /*!<Filter Scale Configuration bit 2 */
constexpr uint32_t  CAN_FS1R_FSC3                       = 0x00000008;        /*!<Filter Scale Configuration bit 3 */
constexpr uint32_t  CAN_FS1R_FSC4                       = 0x00000010;        /*!<Filter Scale Configuration bit 4 */
constexpr uint32_t  CAN_FS1R_FSC5                       = 0x00000020;        /*!<Filter Scale Configuration bit 5 */
constexpr uint32_t  CAN_FS1R_FSC6                       = 0x00000040;        /*!<Filter Scale Configuration bit 6 */
constexpr uint32_t  CAN_FS1R_FSC7                       = 0x00000080;        /*!<Filter Scale Configuration bit 7 */
constexpr uint32_t  CAN_FS1R_FSC8                       = 0x00000100;        /*!<Filter Scale Configuration bit 8 */
constexpr uint32_t  CAN_FS1R_FSC9                       = 0x00000200;        /*!<Filter Scale Configuration bit 9 */
constexpr uint32_t  CAN_FS1R_FSC10                      = 0x00000400;        /*!<Filter Scale Configuration bit 10 */
constexpr uint32_t  CAN_FS1R_FSC11                      = 0x00000800;        /*!<Filter Scale Configuration bit 11 */
constexpr uint32_t  CAN_FS1R_FSC12                      = 0x00001000;        /*!<Filter Scale Configuration bit 12 */
constexpr uint32_t  CAN_FS1R_FSC13                      = 0x00002000;        /*!<Filter Scale Configuration bit 13 */

/******************  Bit definition for CAN_FFA1R register  *******************/
constexpr uint32_t  CAN_FFA1R_FFA                       = 0x00003FFF;        /*!<Filter FIFO Assignment */
constexpr uint32_t  CAN_FFA1R_FFA0                      = 0x00000001;        /*!<Filter FIFO Assignment for Filter 0 */
constexpr uint32_t  CAN_FFA1R_FFA1                      = 0x00000002;        /*!<Filter FIFO Assignment for Filter 1 */
constexpr uint32_t  CAN_FFA1R_FFA2                      = 0x00000004;        /*!<Filter FIFO Assignment for Filter 2 */
constexpr uint32_t  CAN_FFA1R_FFA3                      = 0x00000008;        /*!<Filter FIFO Assignment for Filter 3 */
constexpr uint32_t  CAN_FFA1R_FFA4                      = 0x00000010;        /*!<Filter FIFO Assignment for Filter 4 */
constexpr uint32_t  CAN_FFA1R_FFA5                      = 0x00000020;        /*!<Filter FIFO Assignment for Filter 5 */
constexpr uint32_t  CAN_FFA1R_FFA6                      = 0x00000040;        /*!<Filter FIFO Assignment for Filter 6 */
constexpr uint32_t  CAN_FFA1R_FFA7                      = 0x00000080;        /*!<Filter FIFO Assignment for Filter 7 */
constexpr uint32_t  CAN_FFA1R_FFA8                      = 0x00000100;        /*!<Filter FIFO Assignment for Filter 8 */
constexpr uint32_t  CAN_FFA1R_FFA9                      = 0x00000200;        /*!<Filter FIFO Assignment for Filter 9 */
constexpr uint32_t  CAN_FFA1R_FFA10                     = 0x00000400;        /*!<Filter FIFO Assignment for Filter 10 */
constexpr uint32_t  CAN_FFA1R_FFA11                     = 0x00000800;        /*!<Filter FIFO Assignment for Filter 11 */
constexpr uint32_t  CAN_FFA1R_FFA12                     = 0x00001000;        /*!<Filter FIFO Assignment for Filter 12 */
constexpr uint32_t  CAN_FFA1R_FFA13                     = 0x00002000;        /*!<Filter FIFO Assignment for Filter 13 */

/*******************  Bit definition for CAN_FA1R register  *******************/
constexpr uint32_t  CAN_FA1R_FACT                       = 0x00003FFF;        /*!<Filter Active */
constexpr uint32_t  CAN_FA1R_FACT0                      = 0x00000001;        /*!<Filter 0 Active */
constexpr uint32_t  CAN_FA1R_FACT1                      = 0x00000002;        /*!<Filter 1 Active */
constexpr uint32_t  CAN_FA1R_FACT2                      = 0x00000004;        /*!<Filter 2 Active */
constexpr uint32_t  CAN_FA1R_FACT3                      = 0x00000008;        /*!<Filter 3 Active */
constexpr uint32_t  CAN_FA1R_FACT4                      = 0x00000010;        /*!<Filter 4 Active */
constexpr uint32_t  CAN_FA1R_FACT5                      = 0x00000020;        /*!<Filter 5 Active */
constexpr uint32_t  CAN_FA1R_FACT6                      = 0x00000040;        /*!<Filter 6 Active */
constexpr uint32_t  CAN_FA1R_FACT7                      = 0x00000080;        /*!<Filter 7 Active */
constexpr uint32_t  CAN_FA1R_FACT8                      = 0x00000100;        /*!<Filter 8 Active */
constexpr uint32_t  CAN_FA1R_FACT9                      = 0x00000200;        /*!<Filter 9 Active */
constexpr uint32_t  CAN_FA1R_FACT10                     = 0x00000400;        /*!<Filter 10 Active */
constexpr uint32_t  CAN_FA1R_FACT11                     = 0x00000800;        /*!<Filter 11 Active */
constexpr uint32_t  CAN_FA1R_FACT12                     = 0x00001000;        /*!<Filter 12 Active */
constexpr uint32_t  CAN_FA1R_FACT13                     = 0x00002000;        /*!<Filter 13 Active */

/*******************  Bit definition for CAN_F0R1 register  *******************/
constexpr uint32_t  CAN_F0R1_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F0R1_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F0R1_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F0R1_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F0R1_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F0R1_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F0R1_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F0R1_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F0R1_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F0R1_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F0R1_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F0R1_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F0R1_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F0R1_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F0R1_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F0R1_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F0R1_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F0R1_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F0R1_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F0R1_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F0R1_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F0R1_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F0R1_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F0R1_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F0R1_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F0R1_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F0R1_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F0R1_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F0R1_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F0R1_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F0R1_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F0R1_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F1R1 register  *******************/
constexpr uint32_t  CAN_F1R1_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F1R1_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F1R1_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F1R1_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F1R1_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F1R1_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F1R1_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F1R1_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F1R1_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F1R1_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F1R1_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F1R1_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F1R1_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F1R1_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F1R1_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F1R1_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F1R1_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F1R1_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F1R1_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F1R1_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F1R1_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F1R1_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F1R1_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F1R1_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F1R1_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F1R1_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F1R1_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F1R1_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F1R1_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F1R1_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F1R1_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F1R1_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F2R1 register  *******************/
constexpr uint32_t  CAN_F2R1_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F2R1_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F2R1_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F2R1_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F2R1_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F2R1_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F2R1_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F2R1_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F2R1_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F2R1_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F2R1_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F2R1_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F2R1_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F2R1_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F2R1_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F2R1_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F2R1_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F2R1_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F2R1_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F2R1_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F2R1_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F2R1_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F2R1_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F2R1_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F2R1_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F2R1_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F2R1_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F2R1_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F2R1_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F2R1_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F2R1_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F2R1_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F3R1 register  *******************/
constexpr uint32_t  CAN_F3R1_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F3R1_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F3R1_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F3R1_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F3R1_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F3R1_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F3R1_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F3R1_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F3R1_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F3R1_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F3R1_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F3R1_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F3R1_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F3R1_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F3R1_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F3R1_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F3R1_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F3R1_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F3R1_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F3R1_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F3R1_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F3R1_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F3R1_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F3R1_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F3R1_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F3R1_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F3R1_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F3R1_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F3R1_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F3R1_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F3R1_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F3R1_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F4R1 register  *******************/
constexpr uint32_t  CAN_F4R1_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F4R1_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F4R1_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F4R1_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F4R1_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F4R1_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F4R1_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F4R1_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F4R1_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F4R1_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F4R1_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F4R1_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F4R1_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F4R1_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F4R1_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F4R1_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F4R1_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F4R1_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F4R1_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F4R1_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F4R1_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F4R1_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F4R1_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F4R1_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F4R1_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F4R1_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F4R1_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F4R1_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F4R1_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F4R1_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F4R1_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F4R1_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F5R1 register  *******************/
constexpr uint32_t  CAN_F5R1_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F5R1_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F5R1_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F5R1_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F5R1_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F5R1_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F5R1_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F5R1_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F5R1_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F5R1_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F5R1_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F5R1_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F5R1_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F5R1_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F5R1_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F5R1_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F5R1_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F5R1_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F5R1_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F5R1_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F5R1_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F5R1_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F5R1_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F5R1_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F5R1_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F5R1_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F5R1_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F5R1_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F5R1_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F5R1_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F5R1_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F5R1_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F6R1 register  *******************/
constexpr uint32_t  CAN_F6R1_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F6R1_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F6R1_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F6R1_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F6R1_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F6R1_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F6R1_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F6R1_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F6R1_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F6R1_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F6R1_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F6R1_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F6R1_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F6R1_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F6R1_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F6R1_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F6R1_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F6R1_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F6R1_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F6R1_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F6R1_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F6R1_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F6R1_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F6R1_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F6R1_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F6R1_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F6R1_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F6R1_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F6R1_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F6R1_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F6R1_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F6R1_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F7R1 register  *******************/
constexpr uint32_t  CAN_F7R1_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F7R1_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F7R1_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F7R1_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F7R1_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F7R1_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F7R1_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F7R1_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F7R1_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F7R1_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F7R1_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F7R1_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F7R1_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F7R1_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F7R1_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F7R1_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F7R1_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F7R1_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F7R1_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F7R1_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F7R1_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F7R1_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F7R1_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F7R1_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F7R1_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F7R1_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F7R1_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F7R1_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F7R1_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F7R1_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F7R1_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F7R1_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F8R1 register  *******************/
constexpr uint32_t  CAN_F8R1_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F8R1_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F8R1_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F8R1_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F8R1_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F8R1_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F8R1_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F8R1_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F8R1_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F8R1_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F8R1_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F8R1_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F8R1_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F8R1_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F8R1_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F8R1_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F8R1_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F8R1_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F8R1_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F8R1_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F8R1_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F8R1_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F8R1_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F8R1_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F8R1_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F8R1_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F8R1_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F8R1_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F8R1_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F8R1_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F8R1_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F8R1_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F9R1 register  *******************/
constexpr uint32_t  CAN_F9R1_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F9R1_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F9R1_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F9R1_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F9R1_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F9R1_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F9R1_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F9R1_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F9R1_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F9R1_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F9R1_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F9R1_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F9R1_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F9R1_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F9R1_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F9R1_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F9R1_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F9R1_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F9R1_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F9R1_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F9R1_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F9R1_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F9R1_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F9R1_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F9R1_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F9R1_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F9R1_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F9R1_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F9R1_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F9R1_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F9R1_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F9R1_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F10R1 register  ******************/
constexpr uint32_t  CAN_F10R1_FB0                       = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F10R1_FB1                       = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F10R1_FB2                       = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F10R1_FB3                       = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F10R1_FB4                       = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F10R1_FB5                       = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F10R1_FB6                       = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F10R1_FB7                       = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F10R1_FB8                       = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F10R1_FB9                       = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F10R1_FB10                      = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F10R1_FB11                      = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F10R1_FB12                      = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F10R1_FB13                      = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F10R1_FB14                      = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F10R1_FB15                      = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F10R1_FB16                      = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F10R1_FB17                      = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F10R1_FB18                      = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F10R1_FB19                      = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F10R1_FB20                      = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F10R1_FB21                      = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F10R1_FB22                      = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F10R1_FB23                      = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F10R1_FB24                      = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F10R1_FB25                      = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F10R1_FB26                      = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F10R1_FB27                      = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F10R1_FB28                      = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F10R1_FB29                      = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F10R1_FB30                      = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F10R1_FB31                      = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F11R1 register  ******************/
constexpr uint32_t  CAN_F11R1_FB0                       = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F11R1_FB1                       = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F11R1_FB2                       = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F11R1_FB3                       = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F11R1_FB4                       = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F11R1_FB5                       = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F11R1_FB6                       = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F11R1_FB7                       = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F11R1_FB8                       = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F11R1_FB9                       = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F11R1_FB10                      = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F11R1_FB11                      = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F11R1_FB12                      = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F11R1_FB13                      = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F11R1_FB14                      = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F11R1_FB15                      = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F11R1_FB16                      = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F11R1_FB17                      = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F11R1_FB18                      = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F11R1_FB19                      = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F11R1_FB20                      = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F11R1_FB21                      = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F11R1_FB22                      = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F11R1_FB23                      = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F11R1_FB24                      = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F11R1_FB25                      = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F11R1_FB26                      = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F11R1_FB27                      = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F11R1_FB28                      = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F11R1_FB29                      = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F11R1_FB30                      = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F11R1_FB31                      = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F12R1 register  ******************/
constexpr uint32_t  CAN_F12R1_FB0                       = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F12R1_FB1                       = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F12R1_FB2                       = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F12R1_FB3                       = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F12R1_FB4                       = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F12R1_FB5                       = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F12R1_FB6                       = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F12R1_FB7                       = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F12R1_FB8                       = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F12R1_FB9                       = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F12R1_FB10                      = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F12R1_FB11                      = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F12R1_FB12                      = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F12R1_FB13                      = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F12R1_FB14                      = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F12R1_FB15                      = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F12R1_FB16                      = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F12R1_FB17                      = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F12R1_FB18                      = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F12R1_FB19                      = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F12R1_FB20                      = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F12R1_FB21                      = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F12R1_FB22                      = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F12R1_FB23                      = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F12R1_FB24                      = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F12R1_FB25                      = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F12R1_FB26                      = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F12R1_FB27                      = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F12R1_FB28                      = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F12R1_FB29                      = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F12R1_FB30                      = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F12R1_FB31                      = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F13R1 register  ******************/
constexpr uint32_t  CAN_F13R1_FB0                       = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F13R1_FB1                       = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F13R1_FB2                       = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F13R1_FB3                       = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F13R1_FB4                       = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F13R1_FB5                       = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F13R1_FB6                       = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F13R1_FB7                       = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F13R1_FB8                       = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F13R1_FB9                       = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F13R1_FB10                      = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F13R1_FB11                      = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F13R1_FB12                      = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F13R1_FB13                      = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F13R1_FB14                      = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F13R1_FB15                      = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F13R1_FB16                      = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F13R1_FB17                      = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F13R1_FB18                      = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F13R1_FB19                      = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F13R1_FB20                      = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F13R1_FB21                      = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F13R1_FB22                      = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F13R1_FB23                      = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F13R1_FB24                      = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F13R1_FB25                      = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F13R1_FB26                      = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F13R1_FB27                      = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F13R1_FB28                      = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F13R1_FB29                      = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F13R1_FB30                      = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F13R1_FB31                      = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F0R2 register  *******************/
constexpr uint32_t  CAN_F0R2_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F0R2_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F0R2_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F0R2_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F0R2_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F0R2_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F0R2_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F0R2_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F0R2_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F0R2_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F0R2_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F0R2_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F0R2_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F0R2_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F0R2_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F0R2_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F0R2_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F0R2_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F0R2_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F0R2_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F0R2_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F0R2_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F0R2_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F0R2_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F0R2_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F0R2_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F0R2_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F0R2_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F0R2_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F0R2_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F0R2_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F0R2_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F1R2 register  *******************/
constexpr uint32_t  CAN_F1R2_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F1R2_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F1R2_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F1R2_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F1R2_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F1R2_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F1R2_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F1R2_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F1R2_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F1R2_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F1R2_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F1R2_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F1R2_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F1R2_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F1R2_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F1R2_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F1R2_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F1R2_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F1R2_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F1R2_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F1R2_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F1R2_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F1R2_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F1R2_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F1R2_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F1R2_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F1R2_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F1R2_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F1R2_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F1R2_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F1R2_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F1R2_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F2R2 register  *******************/
constexpr uint32_t  CAN_F2R2_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F2R2_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F2R2_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F2R2_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F2R2_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F2R2_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F2R2_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F2R2_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F2R2_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F2R2_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F2R2_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F2R2_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F2R2_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F2R2_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F2R2_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F2R2_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F2R2_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F2R2_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F2R2_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F2R2_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F2R2_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F2R2_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F2R2_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F2R2_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F2R2_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F2R2_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F2R2_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F2R2_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F2R2_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F2R2_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F2R2_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F2R2_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F3R2 register  *******************/
constexpr uint32_t  CAN_F3R2_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F3R2_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F3R2_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F3R2_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F3R2_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F3R2_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F3R2_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F3R2_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F3R2_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F3R2_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F3R2_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F3R2_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F3R2_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F3R2_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F3R2_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F3R2_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F3R2_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F3R2_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F3R2_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F3R2_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F3R2_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F3R2_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F3R2_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F3R2_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F3R2_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F3R2_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F3R2_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F3R2_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F3R2_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F3R2_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F3R2_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F3R2_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F4R2 register  *******************/
constexpr uint32_t  CAN_F4R2_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F4R2_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F4R2_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F4R2_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F4R2_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F4R2_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F4R2_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F4R2_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F4R2_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F4R2_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F4R2_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F4R2_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F4R2_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F4R2_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F4R2_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F4R2_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F4R2_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F4R2_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F4R2_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F4R2_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F4R2_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F4R2_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F4R2_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F4R2_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F4R2_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F4R2_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F4R2_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F4R2_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F4R2_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F4R2_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F4R2_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F4R2_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F5R2 register  *******************/
constexpr uint32_t  CAN_F5R2_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F5R2_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F5R2_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F5R2_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F5R2_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F5R2_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F5R2_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F5R2_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F5R2_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F5R2_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F5R2_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F5R2_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F5R2_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F5R2_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F5R2_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F5R2_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F5R2_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F5R2_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F5R2_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F5R2_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F5R2_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F5R2_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F5R2_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F5R2_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F5R2_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F5R2_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F5R2_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F5R2_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F5R2_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F5R2_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F5R2_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F5R2_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F6R2 register  *******************/
constexpr uint32_t  CAN_F6R2_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F6R2_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F6R2_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F6R2_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F6R2_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F6R2_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F6R2_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F6R2_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F6R2_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F6R2_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F6R2_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F6R2_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F6R2_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F6R2_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F6R2_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F6R2_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F6R2_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F6R2_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F6R2_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F6R2_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F6R2_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F6R2_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F6R2_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F6R2_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F6R2_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F6R2_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F6R2_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F6R2_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F6R2_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F6R2_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F6R2_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F6R2_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F7R2 register  *******************/
constexpr uint32_t  CAN_F7R2_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F7R2_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F7R2_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F7R2_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F7R2_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F7R2_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F7R2_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F7R2_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F7R2_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F7R2_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F7R2_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F7R2_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F7R2_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F7R2_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F7R2_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F7R2_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F7R2_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F7R2_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F7R2_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F7R2_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F7R2_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F7R2_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F7R2_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F7R2_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F7R2_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F7R2_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F7R2_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F7R2_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F7R2_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F7R2_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F7R2_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F7R2_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F8R2 register  *******************/
constexpr uint32_t  CAN_F8R2_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F8R2_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F8R2_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F8R2_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F8R2_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F8R2_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F8R2_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F8R2_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F8R2_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F8R2_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F8R2_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F8R2_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F8R2_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F8R2_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F8R2_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F8R2_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F8R2_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F8R2_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F8R2_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F8R2_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F8R2_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F8R2_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F8R2_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F8R2_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F8R2_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F8R2_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F8R2_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F8R2_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F8R2_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F8R2_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F8R2_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F8R2_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F9R2 register  *******************/
constexpr uint32_t  CAN_F9R2_FB0                        = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F9R2_FB1                        = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F9R2_FB2                        = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F9R2_FB3                        = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F9R2_FB4                        = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F9R2_FB5                        = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F9R2_FB6                        = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F9R2_FB7                        = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F9R2_FB8                        = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F9R2_FB9                        = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F9R2_FB10                       = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F9R2_FB11                       = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F9R2_FB12                       = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F9R2_FB13                       = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F9R2_FB14                       = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F9R2_FB15                       = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F9R2_FB16                       = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F9R2_FB17                       = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F9R2_FB18                       = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F9R2_FB19                       = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F9R2_FB20                       = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F9R2_FB21                       = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F9R2_FB22                       = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F9R2_FB23                       = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F9R2_FB24                       = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F9R2_FB25                       = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F9R2_FB26                       = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F9R2_FB27                       = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F9R2_FB28                       = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F9R2_FB29                       = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F9R2_FB30                       = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F9R2_FB31                       = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F10R2 register  ******************/
constexpr uint32_t  CAN_F10R2_FB0                       = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F10R2_FB1                       = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F10R2_FB2                       = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F10R2_FB3                       = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F10R2_FB4                       = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F10R2_FB5                       = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F10R2_FB6                       = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F10R2_FB7                       = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F10R2_FB8                       = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F10R2_FB9                       = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F10R2_FB10                      = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F10R2_FB11                      = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F10R2_FB12                      = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F10R2_FB13                      = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F10R2_FB14                      = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F10R2_FB15                      = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F10R2_FB16                      = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F10R2_FB17                      = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F10R2_FB18                      = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F10R2_FB19                      = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F10R2_FB20                      = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F10R2_FB21                      = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F10R2_FB22                      = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F10R2_FB23                      = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F10R2_FB24                      = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F10R2_FB25                      = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F10R2_FB26                      = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F10R2_FB27                      = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F10R2_FB28                      = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F10R2_FB29                      = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F10R2_FB30                      = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F10R2_FB31                      = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F11R2 register  ******************/
constexpr uint32_t  CAN_F11R2_FB0                       = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F11R2_FB1                       = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F11R2_FB2                       = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F11R2_FB3                       = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F11R2_FB4                       = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F11R2_FB5                       = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F11R2_FB6                       = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F11R2_FB7                       = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F11R2_FB8                       = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F11R2_FB9                       = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F11R2_FB10                      = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F11R2_FB11                      = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F11R2_FB12                      = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F11R2_FB13                      = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F11R2_FB14                      = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F11R2_FB15                      = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F11R2_FB16                      = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F11R2_FB17                      = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F11R2_FB18                      = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F11R2_FB19                      = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F11R2_FB20                      = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F11R2_FB21                      = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F11R2_FB22                      = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F11R2_FB23                      = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F11R2_FB24                      = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F11R2_FB25                      = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F11R2_FB26                      = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F11R2_FB27                      = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F11R2_FB28                      = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F11R2_FB29                      = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F11R2_FB30                      = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F11R2_FB31                      = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F12R2 register  ******************/
constexpr uint32_t  CAN_F12R2_FB0                       = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F12R2_FB1                       = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F12R2_FB2                       = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F12R2_FB3                       = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F12R2_FB4                       = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F12R2_FB5                       = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F12R2_FB6                       = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F12R2_FB7                       = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F12R2_FB8                       = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F12R2_FB9                       = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F12R2_FB10                      = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F12R2_FB11                      = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F12R2_FB12                      = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F12R2_FB13                      = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F12R2_FB14                      = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F12R2_FB15                      = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F12R2_FB16                      = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F12R2_FB17                      = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F12R2_FB18                      = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F12R2_FB19                      = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F12R2_FB20                      = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F12R2_FB21                      = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F12R2_FB22                      = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F12R2_FB23                      = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F12R2_FB24                      = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F12R2_FB25                      = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F12R2_FB26                      = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F12R2_FB27                      = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F12R2_FB28                      = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F12R2_FB29                      = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F12R2_FB30                      = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F12R2_FB31                      = 0x80000000;        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F13R2 register  ******************/
constexpr uint32_t  CAN_F13R2_FB0                       = 0x00000001;        /*!<Filter bit 0 */
constexpr uint32_t  CAN_F13R2_FB1                       = 0x00000002;        /*!<Filter bit 1 */
constexpr uint32_t  CAN_F13R2_FB2                       = 0x00000004;        /*!<Filter bit 2 */
constexpr uint32_t  CAN_F13R2_FB3                       = 0x00000008;        /*!<Filter bit 3 */
constexpr uint32_t  CAN_F13R2_FB4                       = 0x00000010;        /*!<Filter bit 4 */
constexpr uint32_t  CAN_F13R2_FB5                       = 0x00000020;        /*!<Filter bit 5 */
constexpr uint32_t  CAN_F13R2_FB6                       = 0x00000040;        /*!<Filter bit 6 */
constexpr uint32_t  CAN_F13R2_FB7                       = 0x00000080;        /*!<Filter bit 7 */
constexpr uint32_t  CAN_F13R2_FB8                       = 0x00000100;        /*!<Filter bit 8 */
constexpr uint32_t  CAN_F13R2_FB9                       = 0x00000200;        /*!<Filter bit 9 */
constexpr uint32_t  CAN_F13R2_FB10                      = 0x00000400;        /*!<Filter bit 10 */
constexpr uint32_t  CAN_F13R2_FB11                      = 0x00000800;        /*!<Filter bit 11 */
constexpr uint32_t  CAN_F13R2_FB12                      = 0x00001000;        /*!<Filter bit 12 */
constexpr uint32_t  CAN_F13R2_FB13                      = 0x00002000;        /*!<Filter bit 13 */
constexpr uint32_t  CAN_F13R2_FB14                      = 0x00004000;        /*!<Filter bit 14 */
constexpr uint32_t  CAN_F13R2_FB15                      = 0x00008000;        /*!<Filter bit 15 */
constexpr uint32_t  CAN_F13R2_FB16                      = 0x00010000;        /*!<Filter bit 16 */
constexpr uint32_t  CAN_F13R2_FB17                      = 0x00020000;        /*!<Filter bit 17 */
constexpr uint32_t  CAN_F13R2_FB18                      = 0x00040000;        /*!<Filter bit 18 */
constexpr uint32_t  CAN_F13R2_FB19                      = 0x00080000;        /*!<Filter bit 19 */
constexpr uint32_t  CAN_F13R2_FB20                      = 0x00100000;        /*!<Filter bit 20 */
constexpr uint32_t  CAN_F13R2_FB21                      = 0x00200000;        /*!<Filter bit 21 */
constexpr uint32_t  CAN_F13R2_FB22                      = 0x00400000;        /*!<Filter bit 22 */
constexpr uint32_t  CAN_F13R2_FB23                      = 0x00800000;        /*!<Filter bit 23 */
constexpr uint32_t  CAN_F13R2_FB24                      = 0x01000000;        /*!<Filter bit 24 */
constexpr uint32_t  CAN_F13R2_FB25                      = 0x02000000;        /*!<Filter bit 25 */
constexpr uint32_t  CAN_F13R2_FB26                      = 0x04000000;        /*!<Filter bit 26 */
constexpr uint32_t  CAN_F13R2_FB27                      = 0x08000000;        /*!<Filter bit 27 */
constexpr uint32_t  CAN_F13R2_FB28                      = 0x10000000;        /*!<Filter bit 28 */
constexpr uint32_t  CAN_F13R2_FB29                      = 0x20000000;        /*!<Filter bit 29 */
constexpr uint32_t  CAN_F13R2_FB30                      = 0x40000000;        /*!<Filter bit 30 */
constexpr uint32_t  CAN_F13R2_FB31                      = 0x80000000;        /*!<Filter bit 31 */

/******************************************************************************/
/*                                                                            */
/*                     CRC calculation unit = CRC;                             */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CRC_DR register  *********************/
constexpr uint32_t  CRC_DR_DR                           = 0xFFFFFFFF; /*!< Data register bits */

/*******************  Bit definition for CRC_IDR register  ********************/
constexpr uint32_t  CRC_IDR_IDR                         = 0xFF;       /*!< General-purpose 8-bit data register bits */

/********************  Bit definition for CRC_CR register  ********************/
constexpr uint32_t  CRC_CR_RESET                        = 0x00000001; /*!< RESET the CRC computation unit bit */
constexpr uint32_t  CRC_CR_POLYSIZE                     = 0x00000018; /*!< Polynomial size bits */
constexpr uint32_t  CRC_CR_POLYSIZE_0                   = 0x00000008; /*!< Polynomial size bit 0 */
constexpr uint32_t  CRC_CR_POLYSIZE_1                   = 0x00000010; /*!< Polynomial size bit 1 */
constexpr uint32_t  CRC_CR_REV_IN                       = 0x00000060; /*!< REV_IN Reverse Input Data bits */
constexpr uint32_t  CRC_CR_REV_IN_0                     = 0x00000020; /*!< Bit 0 */
constexpr uint32_t  CRC_CR_REV_IN_1                     = 0x00000040; /*!< Bit 1 */
constexpr uint32_t  CRC_CR_REV_OUT                      = 0x00000080; /*!< REV_OUT Reverse Output Data bits */

/*******************  Bit definition for CRC_INIT register  *******************/
constexpr uint32_t  CRC_INIT_INIT                       = 0xFFFFFFFF; /*!< Initial CRC value bits */

/*******************  Bit definition for CRC_POL register  ********************/
constexpr uint32_t  CRC_POL_POL                         = 0xFFFFFFFF; /*!< Coefficients of the polynomial */

/******************************************************************************/
/*                                                                            */
/*                 Digital to Analog Converter = DAC;                          */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DAC_CR register  ********************/
constexpr uint32_t  DAC_CR_EN1                          = 0x00000001;        /*!< DAC channel1 enable */
constexpr uint32_t  DAC_CR_BOFF1                        = 0x00000002;        /*!< DAC channel1 output buffer disable */
constexpr uint32_t  DAC_CR_TEN1                         = 0x00000004;        /*!< DAC channel1 Trigger enable */

constexpr uint32_t  DAC_CR_TSEL1                        = 0x00000038;        /*!< TSEL1[2:0] = DAC channel1 Trigger selection; */
constexpr uint32_t  DAC_CR_TSEL1_0                      = 0x00000008;        /*!< Bit 0 */
constexpr uint32_t  DAC_CR_TSEL1_1                      = 0x00000010;        /*!< Bit 1 */
constexpr uint32_t  DAC_CR_TSEL1_2                      = 0x00000020;        /*!< Bit 2 */

constexpr uint32_t  DAC_CR_WAVE1                        = 0x000000C0;        /*!< WAVE1[1:0] = DAC channel1 noise/triangle wave generation enable; */
constexpr uint32_t  DAC_CR_WAVE1_0                      = 0x00000040;        /*!< Bit 0 */
constexpr uint32_t  DAC_CR_WAVE1_1                      = 0x00000080;        /*!< Bit 1 */

constexpr uint32_t  DAC_CR_MAMP1                        = 0x00000F00;        /*!< MAMP1[3:0] = DAC channel1 Mask/Amplitude selector; */
constexpr uint32_t  DAC_CR_MAMP1_0                      = 0x00000100;        /*!< Bit 0 */
constexpr uint32_t  DAC_CR_MAMP1_1                      = 0x00000200;        /*!< Bit 1 */
constexpr uint32_t  DAC_CR_MAMP1_2                      = 0x00000400;        /*!< Bit 2 */
constexpr uint32_t  DAC_CR_MAMP1_3                      = 0x00000800;        /*!< Bit 3 */

constexpr uint32_t  DAC_CR_DMAEN1                       = 0x00001000;        /*!< DAC channel1 DMA enable */
constexpr uint32_t  DAC_CR_DMAUDRIE1                    = 0x00002000;        /*!< DAC channel1 DMA underrun IT enable */
constexpr uint32_t  DAC_CR_EN2                          = 0x00010000;        /*!< DAC channel2 enable */
constexpr uint32_t  DAC_CR_BOFF2                        = 0x00020000;        /*!< DAC channel2 output buffer disable */
constexpr uint32_t  DAC_CR_TEN2                         = 0x00040000;        /*!< DAC channel2 Trigger enable */

constexpr uint32_t  DAC_CR_TSEL2                        = 0x00380000;        /*!< TSEL2[2:0] = DAC channel2 Trigger selection; */
constexpr uint32_t  DAC_CR_TSEL2_0                      = 0x00080000;        /*!< Bit 0 */
constexpr uint32_t  DAC_CR_TSEL2_1                      = 0x00100000;        /*!< Bit 1 */
constexpr uint32_t  DAC_CR_TSEL2_2                      = 0x00200000;        /*!< Bit 2 */

constexpr uint32_t  DAC_CR_WAVE2                        = 0x00C00000;        /*!< WAVE2[1:0] = DAC channel2 noise/triangle wave generation enable; */
constexpr uint32_t  DAC_CR_WAVE2_0                      = 0x00400000;        /*!< Bit 0 */
constexpr uint32_t  DAC_CR_WAVE2_1                      = 0x00800000;        /*!< Bit 1 */

constexpr uint32_t  DAC_CR_MAMP2                        = 0x0F000000;        /*!< MAMP2[3:0] = DAC channel2 Mask/Amplitude selector; */
constexpr uint32_t  DAC_CR_MAMP2_0                      = 0x01000000;        /*!< Bit 0 */
constexpr uint32_t  DAC_CR_MAMP2_1                      = 0x02000000;        /*!< Bit 1 */
constexpr uint32_t  DAC_CR_MAMP2_2                      = 0x04000000;        /*!< Bit 2 */
constexpr uint32_t  DAC_CR_MAMP2_3                      = 0x08000000;        /*!< Bit 3 */

constexpr uint32_t  DAC_CR_DMAEN2                       = 0x10000000;        /*!< DAC channel2 DMA enabled */
constexpr uint32_t  DAC_CR_DMAUDRIE2                    = 0x20000000;        /*!< DAC channel2 DMA underrun IT enable */

/*****************  Bit definition for DAC_SWTRIGR register  ******************/
constexpr uint32_t  DAC_SWTRIGR_SWTRIG1                 = 0x00000001;        /*!< DAC channel1 software trigger */
constexpr uint32_t  DAC_SWTRIGR_SWTRIG2                 = 0x00000002;        /*!< DAC channel2 software trigger */

/*****************  Bit definition for DAC_DHR12R1 register  ******************/
constexpr uint32_t  DAC_DHR12R1_DACC1DHR                = 0x00000FFF;        /*!< DAC channel1 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L1 register  ******************/
constexpr uint32_t  DAC_DHR12L1_DACC1DHR                = 0x0000FFF0;        /*!< DAC channel1 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R1 register  ******************/
constexpr uint32_t  DAC_DHR8R1_DACC1DHR                 = 0x000000FF;        /*!< DAC channel1 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12R2 register  ******************/
constexpr uint32_t  DAC_DHR12R2_DACC2DHR                = 0x00000FFF;        /*!< DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L2 register  ******************/
constexpr uint32_t  DAC_DHR12L2_DACC2DHR                = 0x0000FFF0;        /*!< DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R2 register  ******************/
constexpr uint32_t  DAC_DHR8R2_DACC2DHR                 = 0x000000FF;        /*!< DAC channel2 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12RD register  ******************/
constexpr uint32_t  DAC_DHR12RD_DACC1DHR                = 0x00000FFF;        /*!< DAC channel1 12-bit Right aligned data */
constexpr uint32_t  DAC_DHR12RD_DACC2DHR                = 0x0FFF0000;        /*!< DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12LD register  ******************/
constexpr uint32_t  DAC_DHR12LD_DACC1DHR                = 0x0000FFF0;        /*!< DAC channel1 12-bit Left aligned data */
constexpr uint32_t  DAC_DHR12LD_DACC2DHR                = 0xFFF00000;        /*!< DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8RD register  ******************/
constexpr uint32_t  DAC_DHR8RD_DACC1DHR                 = 0x000000FF;        /*!< DAC channel1 8-bit Right aligned data */
constexpr uint32_t  DAC_DHR8RD_DACC2DHR                 = 0x0000FF00;        /*!< DAC channel2 8-bit Right aligned data */

/*******************  Bit definition for DAC_DOR1 register  *******************/
constexpr uint32_t  DAC_DOR1_DACC1DOR                   = 0x00000FFF;        /*!< DAC channel1 data output */

/*******************  Bit definition for DAC_DOR2 register  *******************/
constexpr uint32_t  DAC_DOR2_DACC2DOR                   = 0x00000FFF;        /*!< DAC channel2 data output */

/********************  Bit definition for DAC_SR register  ********************/
constexpr uint32_t  DAC_SR_DMAUDR1                      = 0x00002000;        /*!< DAC channel1 DMA underrun flag */
constexpr uint32_t  DAC_SR_DMAUDR2                      = 0x20000000;        /*!< DAC channel2 DMA underrun flag */

/******************************************************************************/
/*                                                                            */
/*                                 Debug MCU = DBGMCU;                         */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DBGMCU_IDCODE register  *************/
constexpr uint32_t  DBGMCU_IDCODE_DEV_ID                = 0x00000FFF;
constexpr uint32_t  DBGMCU_IDCODE_REV_ID                = 0xFFFF0000;

/********************  Bit definition for DBGMCU_CR register  *****************/
constexpr uint32_t  DBGMCU_CR_DBG_SLEEP                 = 0x00000001;
constexpr uint32_t  DBGMCU_CR_DBG_STOP                  = 0x00000002;
constexpr uint32_t  DBGMCU_CR_DBG_STANDBY               = 0x00000004;
constexpr uint32_t  DBGMCU_CR_TRACE_IOEN                = 0x00000020;

constexpr uint32_t  DBGMCU_CR_TRACE_MODE                = 0x000000C0;
constexpr uint32_t  DBGMCU_CR_TRACE_MODE_0              = 0x00000040;/*!<Bit 0 */
constexpr uint32_t  DBGMCU_CR_TRACE_MODE_1              = 0x00000080;/*!<Bit 1 */

/********************  Bit definition for DBGMCU_APB1_FZ register  ************/
constexpr uint32_t  DBGMCU_APB1_FZ_DBG_TIM2_STOP            = 0x00000001;
constexpr uint32_t  DBGMCU_APB1_FZ_DBG_TIM3_STOP            = 0x00000002;
constexpr uint32_t  DBGMCU_APB1_FZ_DBG_TIM4_STOP            = 0x00000004;
constexpr uint32_t  DBGMCU_APB1_FZ_DBG_TIM6_STOP            = 0x00000010;
constexpr uint32_t  DBGMCU_APB1_FZ_DBG_TIM7_STOP            = 0x00000020;
constexpr uint32_t  DBGMCU_APB1_FZ_DBG_RTC_STOP             = 0x00000400;
constexpr uint32_t  DBGMCU_APB1_FZ_DBG_WWDG_STOP            = 0x00000800;
constexpr uint32_t  DBGMCU_APB1_FZ_DBG_IWDG_STOP            = 0x00001000;
constexpr uint32_t  DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT   = 0x00200000;
constexpr uint32_t  DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT   = 0x00400000;
constexpr uint32_t  DBGMCU_APB1_FZ_DBG_CAN_STOP             = 0x02000000;
constexpr uint32_t  DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT   = 0x04000000;

/********************  Bit definition for DBGMCU_APB2_FZ register  ************/
constexpr uint32_t  DBGMCU_APB2_FZ_DBG_TIM1_STOP        = 0x00000001;
constexpr uint32_t  DBGMCU_APB2_FZ_DBG_TIM8_STOP        = 0x00000002;
constexpr uint32_t  DBGMCU_APB2_FZ_DBG_TIM15_STOP       = 0x00000004;
constexpr uint32_t  DBGMCU_APB2_FZ_DBG_TIM16_STOP       = 0x00000008;
constexpr uint32_t  DBGMCU_APB2_FZ_DBG_TIM17_STOP       = 0x00000010;
constexpr uint32_t  DBGMCU_APB2_FZ_DBG_TIM20_STOP       = 0x00000020;

/******************************************************************************/
/*                                                                            */
/*                             DMA Controller = DMA;                           */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for DMA_ISR register  ********************/
constexpr uint32_t  DMA_ISR_GIF1                        = 0x00000001;        /*!< Channel 1 Global interrupt flag */
constexpr uint32_t  DMA_ISR_TCIF1                       = 0x00000002;        /*!< Channel 1 Transfer Complete flag */
constexpr uint32_t  DMA_ISR_HTIF1                       = 0x00000004;        /*!< Channel 1 Half Transfer flag */
constexpr uint32_t  DMA_ISR_TEIF1                       = 0x00000008;        /*!< Channel 1 Transfer Error flag */
constexpr uint32_t  DMA_ISR_GIF2                        = 0x00000010;        /*!< Channel 2 Global interrupt flag */
constexpr uint32_t  DMA_ISR_TCIF2                       = 0x00000020;        /*!< Channel 2 Transfer Complete flag */
constexpr uint32_t  DMA_ISR_HTIF2                       = 0x00000040;        /*!< Channel 2 Half Transfer flag */
constexpr uint32_t  DMA_ISR_TEIF2                       = 0x00000080;        /*!< Channel 2 Transfer Error flag */
constexpr uint32_t  DMA_ISR_GIF3                        = 0x00000100;        /*!< Channel 3 Global interrupt flag */
constexpr uint32_t  DMA_ISR_TCIF3                       = 0x00000200;        /*!< Channel 3 Transfer Complete flag */
constexpr uint32_t  DMA_ISR_HTIF3                       = 0x00000400;        /*!< Channel 3 Half Transfer flag */
constexpr uint32_t  DMA_ISR_TEIF3                       = 0x00000800;        /*!< Channel 3 Transfer Error flag */
constexpr uint32_t  DMA_ISR_GIF4                        = 0x00001000;        /*!< Channel 4 Global interrupt flag */
constexpr uint32_t  DMA_ISR_TCIF4                       = 0x00002000;        /*!< Channel 4 Transfer Complete flag */
constexpr uint32_t  DMA_ISR_HTIF4                       = 0x00004000;        /*!< Channel 4 Half Transfer flag */
constexpr uint32_t  DMA_ISR_TEIF4                       = 0x00008000;        /*!< Channel 4 Transfer Error flag */
constexpr uint32_t  DMA_ISR_GIF5                        = 0x00010000;        /*!< Channel 5 Global interrupt flag */
constexpr uint32_t  DMA_ISR_TCIF5                       = 0x00020000;        /*!< Channel 5 Transfer Complete flag */
constexpr uint32_t  DMA_ISR_HTIF5                       = 0x00040000;        /*!< Channel 5 Half Transfer flag */
constexpr uint32_t  DMA_ISR_TEIF5                       = 0x00080000;        /*!< Channel 5 Transfer Error flag */
constexpr uint32_t  DMA_ISR_GIF6                        = 0x00100000;        /*!< Channel 6 Global interrupt flag */
constexpr uint32_t  DMA_ISR_TCIF6                       = 0x00200000;        /*!< Channel 6 Transfer Complete flag */
constexpr uint32_t  DMA_ISR_HTIF6                       = 0x00400000;        /*!< Channel 6 Half Transfer flag */
constexpr uint32_t  DMA_ISR_TEIF6                       = 0x00800000;        /*!< Channel 6 Transfer Error flag */
constexpr uint32_t  DMA_ISR_GIF7                        = 0x01000000;        /*!< Channel 7 Global interrupt flag */
constexpr uint32_t  DMA_ISR_TCIF7                       = 0x02000000;        /*!< Channel 7 Transfer Complete flag */
constexpr uint32_t  DMA_ISR_HTIF7                       = 0x04000000;        /*!< Channel 7 Half Transfer flag */
constexpr uint32_t  DMA_ISR_TEIF7                       = 0x08000000;        /*!< Channel 7 Transfer Error flag */

/*******************  Bit definition for DMA_IFCR register  *******************/
constexpr uint32_t  DMA_IFCR_CGIF1                      = 0x00000001;        /*!< Channel 1 Global interrupt clear */
constexpr uint32_t  DMA_IFCR_CTCIF1                     = 0x00000002;        /*!< Channel 1 Transfer Complete clear */
constexpr uint32_t  DMA_IFCR_CHTIF1                     = 0x00000004;        /*!< Channel 1 Half Transfer clear */
constexpr uint32_t  DMA_IFCR_CTEIF1                     = 0x00000008;        /*!< Channel 1 Transfer Error clear */
constexpr uint32_t  DMA_IFCR_CGIF2                      = 0x00000010;        /*!< Channel 2 Global interrupt clear */
constexpr uint32_t  DMA_IFCR_CTCIF2                     = 0x00000020;        /*!< Channel 2 Transfer Complete clear */
constexpr uint32_t  DMA_IFCR_CHTIF2                     = 0x00000040;        /*!< Channel 2 Half Transfer clear */
constexpr uint32_t  DMA_IFCR_CTEIF2                     = 0x00000080;        /*!< Channel 2 Transfer Error clear */
constexpr uint32_t  DMA_IFCR_CGIF3                      = 0x00000100;        /*!< Channel 3 Global interrupt clear */
constexpr uint32_t  DMA_IFCR_CTCIF3                     = 0x00000200;        /*!< Channel 3 Transfer Complete clear */
constexpr uint32_t  DMA_IFCR_CHTIF3                     = 0x00000400;        /*!< Channel 3 Half Transfer clear */
constexpr uint32_t  DMA_IFCR_CTEIF3                     = 0x00000800;        /*!< Channel 3 Transfer Error clear */
constexpr uint32_t  DMA_IFCR_CGIF4                      = 0x00001000;        /*!< Channel 4 Global interrupt clear */
constexpr uint32_t  DMA_IFCR_CTCIF4                     = 0x00002000;        /*!< Channel 4 Transfer Complete clear */
constexpr uint32_t  DMA_IFCR_CHTIF4                     = 0x00004000;        /*!< Channel 4 Half Transfer clear */
constexpr uint32_t  DMA_IFCR_CTEIF4                     = 0x00008000;        /*!< Channel 4 Transfer Error clear */
constexpr uint32_t  DMA_IFCR_CGIF5                      = 0x00010000;        /*!< Channel 5 Global interrupt clear */
constexpr uint32_t  DMA_IFCR_CTCIF5                     = 0x00020000;        /*!< Channel 5 Transfer Complete clear */
constexpr uint32_t  DMA_IFCR_CHTIF5                     = 0x00040000;        /*!< Channel 5 Half Transfer clear */
constexpr uint32_t  DMA_IFCR_CTEIF5                     = 0x00080000;        /*!< Channel 5 Transfer Error clear */
constexpr uint32_t  DMA_IFCR_CGIF6                      = 0x00100000;        /*!< Channel 6 Global interrupt clear */
constexpr uint32_t  DMA_IFCR_CTCIF6                     = 0x00200000;        /*!< Channel 6 Transfer Complete clear */
constexpr uint32_t  DMA_IFCR_CHTIF6                     = 0x00400000;        /*!< Channel 6 Half Transfer clear */
constexpr uint32_t  DMA_IFCR_CTEIF6                     = 0x00800000;        /*!< Channel 6 Transfer Error clear */
constexpr uint32_t  DMA_IFCR_CGIF7                      = 0x01000000;        /*!< Channel 7 Global interrupt clear */
constexpr uint32_t  DMA_IFCR_CTCIF7                     = 0x02000000;        /*!< Channel 7 Transfer Complete clear */
constexpr uint32_t  DMA_IFCR_CHTIF7                     = 0x04000000;        /*!< Channel 7 Half Transfer clear */
constexpr uint32_t  DMA_IFCR_CTEIF7                     = 0x08000000;        /*!< Channel 7 Transfer Error clear */

/*******************  Bit definition for DMA_CCR register  ********************/
constexpr uint32_t  DMA_CCR_EN                          = 0x00000001;        /*!< Channel enable                      */
constexpr uint32_t  DMA_CCR_TCIE                        = 0x00000002;        /*!< Transfer complete interrupt enable  */
constexpr uint32_t  DMA_CCR_HTIE                        = 0x00000004;        /*!< Half Transfer interrupt enable      */
constexpr uint32_t  DMA_CCR_TEIE                        = 0x00000008;        /*!< Transfer error interrupt enable     */
constexpr uint32_t  DMA_CCR_DIR                         = 0x00000010;        /*!< Data transfer direction             */
constexpr uint32_t  DMA_CCR_CIRC                        = 0x00000020;        /*!< Circular mode                       */
constexpr uint32_t  DMA_CCR_PINC                        = 0x00000040;        /*!< Peripheral increment mode           */
constexpr uint32_t  DMA_CCR_MINC                        = 0x00000080;        /*!< Memory increment mode               */

constexpr uint32_t  DMA_CCR_PSIZE                       = 0x00000300;        /*!< PSIZE[1:0] bits = Peripheral size;   */
constexpr uint32_t  DMA_CCR_PSIZE_0                     = 0x00000100;        /*!< Bit 0                               */
constexpr uint32_t  DMA_CCR_PSIZE_1                     = 0x00000200;        /*!< Bit 1                               */

constexpr uint32_t  DMA_CCR_MSIZE                       = 0x00000C00;        /*!< MSIZE[1:0] bits = Memory size;       */
constexpr uint32_t  DMA_CCR_MSIZE_0                     = 0x00000400;        /*!< Bit 0                               */
constexpr uint32_t  DMA_CCR_MSIZE_1                     = 0x00000800;        /*!< Bit 1                               */

constexpr uint32_t  DMA_CCR_PL                          = 0x00003000;        /*!< PL[1:0] bits= Channel Priority level;*/
constexpr uint32_t  DMA_CCR_PL_0                        = 0x00001000;        /*!< Bit 0                               */
constexpr uint32_t  DMA_CCR_PL_1                        = 0x00002000;        /*!< Bit 1                               */

constexpr uint32_t  DMA_CCR_MEM2MEM                     = 0x00004000;        /*!< Memory to memory mode               */

/******************  Bit definition for DMA_CNDTR register  *******************/
constexpr uint32_t  DMA_CNDTR_NDT                       = 0x0000FFFF;        /*!< Number of data to Transfer          */

/******************  Bit definition for DMA_CPAR register  ********************/
constexpr uint32_t  DMA_CPAR_PA                         = 0xFFFFFFFF;        /*!< Peripheral Address                  */

/******************  Bit definition for DMA_CMAR register  ********************/
constexpr uint32_t  DMA_CMAR_MA                         = 0xFFFFFFFF;        /*!< Memory Address                      */

/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller = EXTI;              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for EXTI_IMR1/EXTI_IMR2 register  ********/
constexpr uint32_t  EXTI_IMR_MR0                        = 0x00000001;        /*!< Interrupt Mask on line 0 */
constexpr uint32_t  EXTI_IMR_MR1                        = 0x00000002;        /*!< Interrupt Mask on line 1 */
constexpr uint32_t  EXTI_IMR_MR2                        = 0x00000004;        /*!< Interrupt Mask on line 2 */
constexpr uint32_t  EXTI_IMR_MR3                        = 0x00000008;        /*!< Interrupt Mask on line 3 */
constexpr uint32_t  EXTI_IMR_MR4                        = 0x00000010;        /*!< Interrupt Mask on line 4 */
constexpr uint32_t  EXTI_IMR_MR5                        = 0x00000020;        /*!< Interrupt Mask on line 5 */
constexpr uint32_t  EXTI_IMR_MR6                        = 0x00000040;        /*!< Interrupt Mask on line 6 */
constexpr uint32_t  EXTI_IMR_MR7                        = 0x00000080;        /*!< Interrupt Mask on line 7 */
constexpr uint32_t  EXTI_IMR_MR8                        = 0x00000100;        /*!< Interrupt Mask on line 8 */
constexpr uint32_t  EXTI_IMR_MR9                        = 0x00000200;        /*!< Interrupt Mask on line 9 */
constexpr uint32_t  EXTI_IMR_MR10                       = 0x00000400;        /*!< Interrupt Mask on line 10 */
constexpr uint32_t  EXTI_IMR_MR11                       = 0x00000800;        /*!< Interrupt Mask on line 11 */
constexpr uint32_t  EXTI_IMR_MR12                       = 0x00001000;        /*!< Interrupt Mask on line 12 */
constexpr uint32_t  EXTI_IMR_MR13                       = 0x00002000;        /*!< Interrupt Mask on line 13 */
constexpr uint32_t  EXTI_IMR_MR14                       = 0x00004000;        /*!< Interrupt Mask on line 14 */
constexpr uint32_t  EXTI_IMR_MR15                       = 0x00008000;        /*!< Interrupt Mask on line 15 */
constexpr uint32_t  EXTI_IMR_MR16                       = 0x00010000;        /*!< Interrupt Mask on line 16 */
constexpr uint32_t  EXTI_IMR_MR17                       = 0x00020000;        /*!< Interrupt Mask on line 17 */
constexpr uint32_t  EXTI_IMR_MR18                       = 0x00040000;        /*!< Interrupt Mask on line 18 */
constexpr uint32_t  EXTI_IMR_MR19                       = 0x00080000;        /*!< Interrupt Mask on line 19 */
constexpr uint32_t  EXTI_IMR_MR20                       = 0x00100000;        /*!< Interrupt Mask on line 20 */
constexpr uint32_t  EXTI_IMR_MR21                       = 0x00200000;        /*!< Interrupt Mask on line 21 */
constexpr uint32_t  EXTI_IMR_MR22                       = 0x00400000;        /*!< Interrupt Mask on line 22 */
constexpr uint32_t  EXTI_IMR_MR23                       = 0x00800000;        /*!< Interrupt Mask on line 23 */
constexpr uint32_t  EXTI_IMR_MR24                       = 0x01000000;        /*!< Interrupt Mask on line 24 */
constexpr uint32_t  EXTI_IMR_MR25                       = 0x02000000;        /*!< Interrupt Mask on line 25 */
constexpr uint32_t  EXTI_IMR_MR26                       = 0x04000000;        /*!< Interrupt Mask on line 26 */
constexpr uint32_t  EXTI_IMR_MR27                       = 0x08000000;        /*!< Interrupt Mask on line 27 */
constexpr uint32_t  EXTI_IMR_MR28                       = 0x10000000;        /*!< Interrupt Mask on line 28 */

/*******************  Bit definition for EXTI_EMR1/EXTI_EMR2 register  ********/
constexpr uint32_t  EXTI_EMR_MR0                        = 0x00000001;        /*!< Event Mask on line 0 */
constexpr uint32_t  EXTI_EMR_MR1                        = 0x00000002;        /*!< Event Mask on line 1 */
constexpr uint32_t  EXTI_EMR_MR2                        = 0x00000004;        /*!< Event Mask on line 2 */
constexpr uint32_t  EXTI_EMR_MR3                        = 0x00000008;        /*!< Event Mask on line 3 */
constexpr uint32_t  EXTI_EMR_MR4                        = 0x00000010;        /*!< Event Mask on line 4 */
constexpr uint32_t  EXTI_EMR_MR5                        = 0x00000020;        /*!< Event Mask on line 5 */
constexpr uint32_t  EXTI_EMR_MR6                        = 0x00000040;        /*!< Event Mask on line 6 */
constexpr uint32_t  EXTI_EMR_MR7                        = 0x00000080;        /*!< Event Mask on line 7 */
constexpr uint32_t  EXTI_EMR_MR8                        = 0x00000100;        /*!< Event Mask on line 8 */
constexpr uint32_t  EXTI_EMR_MR9                        = 0x00000200;        /*!< Event Mask on line 9 */
constexpr uint32_t  EXTI_EMR_MR10                       = 0x00000400;        /*!< Event Mask on line 10 */
constexpr uint32_t  EXTI_EMR_MR11                       = 0x00000800;        /*!< Event Mask on line 11 */
constexpr uint32_t  EXTI_EMR_MR12                       = 0x00001000;        /*!< Event Mask on line 12 */
constexpr uint32_t  EXTI_EMR_MR13                       = 0x00002000;        /*!< Event Mask on line 13 */
constexpr uint32_t  EXTI_EMR_MR14                       = 0x00004000;        /*!< Event Mask on line 14 */
constexpr uint32_t  EXTI_EMR_MR15                       = 0x00008000;        /*!< Event Mask on line 15 */
constexpr uint32_t  EXTI_EMR_MR16                       = 0x00010000;        /*!< Event Mask on line 16 */
constexpr uint32_t  EXTI_EMR_MR17                       = 0x00020000;        /*!< Event Mask on line 17 */
constexpr uint32_t  EXTI_EMR_MR18                       = 0x00040000;        /*!< Event Mask on line 18 */
constexpr uint32_t  EXTI_EMR_MR19                       = 0x00080000;        /*!< Event Mask on line 19 */
constexpr uint32_t  EXTI_EMR_MR20                       = 0x00100000;        /*!< Event Mask on line 20 */
constexpr uint32_t  EXTI_EMR_MR21                       = 0x00200000;        /*!< Event Mask on line 21 */
constexpr uint32_t  EXTI_EMR_MR22                       = 0x00400000;        /*!< Event Mask on line 22 */
constexpr uint32_t  EXTI_EMR_MR23                       = 0x00800000;        /*!< Event Mask on line 23 */
constexpr uint32_t  EXTI_EMR_MR24                       = 0x01000000;        /*!< Event Mask on line 24 */
constexpr uint32_t  EXTI_EMR_MR25                       = 0x02000000;        /*!< Event Mask on line 25 */
constexpr uint32_t  EXTI_EMR_MR26                       = 0x04000000;        /*!< Event Mask on line 26 */
constexpr uint32_t  EXTI_EMR_MR27                       = 0x08000000;        /*!< Event Mask on line 27 */
constexpr uint32_t  EXTI_EMR_MR28                       = 0x10000000;        /*!< Event Mask on line 28 */

/******************  Bit definition for EXTI_RTSR1/EXTI_RTSR2 register  *******/
constexpr uint32_t  EXTI_RTSR_TR0                       = 0x00000001;        /*!< Rising trigger event configuration bit of line 0 */
constexpr uint32_t  EXTI_RTSR_TR1                       = 0x00000002;        /*!< Rising trigger event configuration bit of line 1 */
constexpr uint32_t  EXTI_RTSR_TR2                       = 0x00000004;        /*!< Rising trigger event configuration bit of line 2 */
constexpr uint32_t  EXTI_RTSR_TR3                       = 0x00000008;        /*!< Rising trigger event configuration bit of line 3 */
constexpr uint32_t  EXTI_RTSR_TR4                       = 0x00000010;        /*!< Rising trigger event configuration bit of line 4 */
constexpr uint32_t  EXTI_RTSR_TR5                       = 0x00000020;        /*!< Rising trigger event configuration bit of line 5 */
constexpr uint32_t  EXTI_RTSR_TR6                       = 0x00000040;        /*!< Rising trigger event configuration bit of line 6 */
constexpr uint32_t  EXTI_RTSR_TR7                       = 0x00000080;        /*!< Rising trigger event configuration bit of line 7 */
constexpr uint32_t  EXTI_RTSR_TR8                       = 0x00000100;        /*!< Rising trigger event configuration bit of line 8 */
constexpr uint32_t  EXTI_RTSR_TR9                       = 0x00000200;        /*!< Rising trigger event configuration bit of line 9 */
constexpr uint32_t  EXTI_RTSR_TR10                      = 0x00000400;        /*!< Rising trigger event configuration bit of line 10 */
constexpr uint32_t  EXTI_RTSR_TR11                      = 0x00000800;        /*!< Rising trigger event configuration bit of line 11 */
constexpr uint32_t  EXTI_RTSR_TR12                      = 0x00001000;        /*!< Rising trigger event configuration bit of line 12 */
constexpr uint32_t  EXTI_RTSR_TR13                      = 0x00002000;        /*!< Rising trigger event configuration bit of line 13 */
constexpr uint32_t  EXTI_RTSR_TR14                      = 0x00004000;        /*!< Rising trigger event configuration bit of line 14 */
constexpr uint32_t  EXTI_RTSR_TR15                      = 0x00008000;        /*!< Rising trigger event configuration bit of line 15 */
constexpr uint32_t  EXTI_RTSR_TR16                      = 0x00010000;        /*!< Rising trigger event configuration bit of line 16 */
constexpr uint32_t  EXTI_RTSR_TR17                      = 0x00020000;        /*!< Rising trigger event configuration bit of line 17 */
constexpr uint32_t  EXTI_RTSR_TR18                      = 0x00040000;        /*!< Rising trigger event configuration bit of line 18 */
constexpr uint32_t  EXTI_RTSR_TR19                      = 0x00080000;        /*!< Rising trigger event configuration bit of line 19 */
constexpr uint32_t  EXTI_RTSR_TR20                      = 0x00100000;        /*!< Rising trigger event configuration bit of line 20 */
constexpr uint32_t  EXTI_RTSR_TR21                      = 0x00200000;        /*!< Rising trigger event configuration bit of line 21 */
constexpr uint32_t  EXTI_RTSR_TR22                      = 0x00400000;        /*!< Rising trigger event configuration bit of line 22 */
constexpr uint32_t  EXTI_RTSR_TR23                      = 0x00800000;        /*!< Rising trigger event configuration bit of line 23 */
constexpr uint32_t  EXTI_RTSR_TR24                      = 0x01000000;        /*!< Rising trigger event configuration bit of line 24 */
constexpr uint32_t  EXTI_RTSR_TR25                      = 0x02000000;        /*!< Rising trigger event configuration bit of line 25 */
constexpr uint32_t  EXTI_RTSR_TR26                      = 0x04000000;        /*!< Rising trigger event configuration bit of line 26 */
constexpr uint32_t  EXTI_RTSR_TR27                      = 0x08000000;        /*!< Rising trigger event configuration bit of line 27 */
constexpr uint32_t  EXTI_RTSR_TR28                      = 0x10000000;        /*!< Rising trigger event configuration bit of line 28 */

/******************  Bit definition for EXTI_FTSR1/EXTI_FTSR2 register  *******/
constexpr uint32_t  EXTI_FTSR_TR0                       = 0x00000001;        /*!< Falling trigger event configuration bit of line 0 */
constexpr uint32_t  EXTI_FTSR_TR1                       = 0x00000002;        /*!< Falling trigger event configuration bit of line 1 */
constexpr uint32_t  EXTI_FTSR_TR2                       = 0x00000004;        /*!< Falling trigger event configuration bit of line 2 */
constexpr uint32_t  EXTI_FTSR_TR3                       = 0x00000008;        /*!< Falling trigger event configuration bit of line 3 */
constexpr uint32_t  EXTI_FTSR_TR4                       = 0x00000010;        /*!< Falling trigger event configuration bit of line 4 */
constexpr uint32_t  EXTI_FTSR_TR5                       = 0x00000020;        /*!< Falling trigger event configuration bit of line 5 */
constexpr uint32_t  EXTI_FTSR_TR6                       = 0x00000040;        /*!< Falling trigger event configuration bit of line 6 */
constexpr uint32_t  EXTI_FTSR_TR7                       = 0x00000080;        /*!< Falling trigger event configuration bit of line 7 */
constexpr uint32_t  EXTI_FTSR_TR8                       = 0x00000100;        /*!< Falling trigger event configuration bit of line 8 */
constexpr uint32_t  EXTI_FTSR_TR9                       = 0x00000200;        /*!< Falling trigger event configuration bit of line 9 */
constexpr uint32_t  EXTI_FTSR_TR10                      = 0x00000400;        /*!< Falling trigger event configuration bit of line 10 */
constexpr uint32_t  EXTI_FTSR_TR11                      = 0x00000800;        /*!< Falling trigger event configuration bit of line 11 */
constexpr uint32_t  EXTI_FTSR_TR12                      = 0x00001000;        /*!< Falling trigger event configuration bit of line 12 */
constexpr uint32_t  EXTI_FTSR_TR13                      = 0x00002000;        /*!< Falling trigger event configuration bit of line 13 */
constexpr uint32_t  EXTI_FTSR_TR14                      = 0x00004000;        /*!< Falling trigger event configuration bit of line 14 */
constexpr uint32_t  EXTI_FTSR_TR15                      = 0x00008000;        /*!< Falling trigger event configuration bit of line 15 */
constexpr uint32_t  EXTI_FTSR_TR16                      = 0x00010000;        /*!< Falling trigger event configuration bit of line 16 */
constexpr uint32_t  EXTI_FTSR_TR17                      = 0x00020000;        /*!< Falling trigger event configuration bit of line 17 */
constexpr uint32_t  EXTI_FTSR_TR18                      = 0x00040000;        /*!< Falling trigger event configuration bit of line 18 */
constexpr uint32_t  EXTI_FTSR_TR19                      = 0x00080000;        /*!< Falling trigger event configuration bit of line 19 */
constexpr uint32_t  EXTI_FTSR_TR20                      = 0x00100000;        /*!< Falling trigger event configuration bit of line 20 */
constexpr uint32_t  EXTI_FTSR_TR21                      = 0x00200000;        /*!< Falling trigger event configuration bit of line 21 */
constexpr uint32_t  EXTI_FTSR_TR22                      = 0x00400000;        /*!< Falling trigger event configuration bit of line 22 */
constexpr uint32_t  EXTI_FTSR_TR23                      = 0x00800000;        /*!< Falling trigger event configuration bit of line 23 */
constexpr uint32_t  EXTI_FTSR_TR24                      = 0x01000000;        /*!< Falling trigger event configuration bit of line 24 */
constexpr uint32_t  EXTI_FTSR_TR25                      = 0x02000000;        /*!< Falling trigger event configuration bit of line 25 */
constexpr uint32_t  EXTI_FTSR_TR26                      = 0x04000000;        /*!< Falling trigger event configuration bit of line 26 */
constexpr uint32_t  EXTI_FTSR_TR27                      = 0x08000000;        /*!< Falling trigger event configuration bit of line 27 */
constexpr uint32_t  EXTI_FTSR_TR28                      = 0x10000000;        /*!< Falling trigger event configuration bit of line 28 */

/******************  Bit definition for EXTI_SWIER1/EXTI_SWIER2 register  *****/
constexpr uint32_t  EXTI_SWIER_SWIER0                   = 0x00000001;        /*!< Software Interrupt on line 0 */
constexpr uint32_t  EXTI_SWIER_SWIER1                   = 0x00000002;        /*!< Software Interrupt on line 1 */
constexpr uint32_t  EXTI_SWIER_SWIER2                   = 0x00000004;        /*!< Software Interrupt on line 2 */
constexpr uint32_t  EXTI_SWIER_SWIER3                   = 0x00000008;        /*!< Software Interrupt on line 3 */
constexpr uint32_t  EXTI_SWIER_SWIER4                   = 0x00000010;        /*!< Software Interrupt on line 4 */
constexpr uint32_t  EXTI_SWIER_SWIER5                   = 0x00000020;        /*!< Software Interrupt on line 5 */
constexpr uint32_t  EXTI_SWIER_SWIER6                   = 0x00000040;        /*!< Software Interrupt on line 6 */
constexpr uint32_t  EXTI_SWIER_SWIER7                   = 0x00000080;        /*!< Software Interrupt on line 7 */
constexpr uint32_t  EXTI_SWIER_SWIER8                   = 0x00000100;        /*!< Software Interrupt on line 8 */
constexpr uint32_t  EXTI_SWIER_SWIER9                   = 0x00000200;        /*!< Software Interrupt on line 9 */
constexpr uint32_t  EXTI_SWIER_SWIER10                  = 0x00000400;        /*!< Software Interrupt on line 10 */
constexpr uint32_t  EXTI_SWIER_SWIER11                  = 0x00000800;        /*!< Software Interrupt on line 11 */
constexpr uint32_t  EXTI_SWIER_SWIER12                  = 0x00001000;        /*!< Software Interrupt on line 12 */
constexpr uint32_t  EXTI_SWIER_SWIER13                  = 0x00002000;        /*!< Software Interrupt on line 13 */
constexpr uint32_t  EXTI_SWIER_SWIER14                  = 0x00004000;        /*!< Software Interrupt on line 14 */
constexpr uint32_t  EXTI_SWIER_SWIER15                  = 0x00008000;        /*!< Software Interrupt on line 15 */
constexpr uint32_t  EXTI_SWIER_SWIER16                  = 0x00010000;        /*!< Software Interrupt on line 16 */
constexpr uint32_t  EXTI_SWIER_SWIER17                  = 0x00020000;        /*!< Software Interrupt on line 17 */
constexpr uint32_t  EXTI_SWIER_SWIER18                  = 0x00040000;        /*!< Software Interrupt on line 18 */
constexpr uint32_t  EXTI_SWIER_SWIER19                  = 0x00080000;        /*!< Software Interrupt on line 19 */
constexpr uint32_t  EXTI_SWIER_SWIER20                  = 0x00100000;        /*!< Software Interrupt on line 20 */
constexpr uint32_t  EXTI_SWIER_SWIER21                  = 0x00200000;        /*!< Software Interrupt on line 21 */
constexpr uint32_t  EXTI_SWIER_SWIER22                  = 0x00400000;        /*!< Software Interrupt on line 22 */
constexpr uint32_t  EXTI_SWIER_SWIER23                  = 0x00800000;        /*!< Software Interrupt on line 23 */
constexpr uint32_t  EXTI_SWIER_SWIER24                  = 0x01000000;        /*!< Software Interrupt on line 24 */
constexpr uint32_t  EXTI_SWIER_SWIER25                  = 0x02000000;        /*!< Software Interrupt on line 25 */
constexpr uint32_t  EXTI_SWIER_SWIER26                  = 0x04000000;        /*!< Software Interrupt on line 26 */
constexpr uint32_t  EXTI_SWIER_SWIER27                  = 0x08000000;        /*!< Software Interrupt on line 27 */
constexpr uint32_t  EXTI_SWIER_SWIER28                  = 0x10000000;        /*!< Software Interrupt on line 28 */

/*******************  Bit definition for EXTI_PR1/EXTI_PR2 register  **********/
constexpr uint32_t  EXTI_PR_PR0                         = 0x00000001;        /*!< Pending bit for line 0 */
constexpr uint32_t  EXTI_PR_PR1                         = 0x00000002;        /*!< Pending bit for line 1 */
constexpr uint32_t  EXTI_PR_PR2                         = 0x00000004;        /*!< Pending bit for line 2 */
constexpr uint32_t  EXTI_PR_PR3                         = 0x00000008;        /*!< Pending bit for line 3 */
constexpr uint32_t  EXTI_PR_PR4                         = 0x00000010;        /*!< Pending bit for line 4 */
constexpr uint32_t  EXTI_PR_PR5                         = 0x00000020;        /*!< Pending bit for line 5 */
constexpr uint32_t  EXTI_PR_PR6                         = 0x00000040;        /*!< Pending bit for line 6 */
constexpr uint32_t  EXTI_PR_PR7                         = 0x00000080;        /*!< Pending bit for line 7 */
constexpr uint32_t  EXTI_PR_PR8                         = 0x00000100;        /*!< Pending bit for line 8 */
constexpr uint32_t  EXTI_PR_PR9                         = 0x00000200;        /*!< Pending bit for line 9 */
constexpr uint32_t  EXTI_PR_PR10                        = 0x00000400;        /*!< Pending bit for line 10 */
constexpr uint32_t  EXTI_PR_PR11                        = 0x00000800;        /*!< Pending bit for line 11 */
constexpr uint32_t  EXTI_PR_PR12                        = 0x00001000;        /*!< Pending bit for line 12 */
constexpr uint32_t  EXTI_PR_PR13                        = 0x00002000;        /*!< Pending bit for line 13 */
constexpr uint32_t  EXTI_PR_PR14                        = 0x00004000;        /*!< Pending bit for line 14 */
constexpr uint32_t  EXTI_PR_PR15                        = 0x00008000;        /*!< Pending bit for line 15 */
constexpr uint32_t  EXTI_PR_PR16                        = 0x00010000;        /*!< Pending bit for line 16 */
constexpr uint32_t  EXTI_PR_PR17                        = 0x00020000;        /*!< Pending bit for line 17 */
constexpr uint32_t  EXTI_PR_PR18                        = 0x00040000;        /*!< Pending bit for line 18 */
constexpr uint32_t  EXTI_PR_PR19                        = 0x00080000;        /*!< Pending bit for line 19 */
constexpr uint32_t  EXTI_PR_PR20                        = 0x00100000;        /*!< Pending bit for line 20 */
constexpr uint32_t  EXTI_PR_PR21                        = 0x00200000;        /*!< Pending bit for line 21 */
constexpr uint32_t  EXTI_PR_PR22                        = 0x00400000;        /*!< Pending bit for line 22 */
constexpr uint32_t  EXTI_PR_PR23                        = 0x00800000;        /*!< Pending bit for line 23 */
constexpr uint32_t  EXTI_PR_PR24                        = 0x01000000;        /*!< Pending bit for line 24 */
constexpr uint32_t  EXTI_PR_PR25                        = 0x02000000;        /*!< Pending bit for line 25 */
constexpr uint32_t  EXTI_PR_PR26                        = 0x04000000;        /*!< Pending bit for line 26 */
constexpr uint32_t  EXTI_PR_PR27                        = 0x08000000;        /*!< Pending bit for line 27 */
constexpr uint32_t  EXTI_PR_PR28                        = 0x10000000;        /*!< Pending bit for line 28 */

/******************************************************************************/
/*                                                                            */
/*                                    FLASH                                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for FLASH_ACR register  ******************/
constexpr uint32_t  FLASH_ACR_LATENCY                   = 0x00000007;        /*!< LATENCY[2:0] bits = Latency; */
constexpr uint32_t  FLASH_ACR_LATENCY_0                 = 0x00000001;        /*!< Bit 0 */
constexpr uint32_t  FLASH_ACR_LATENCY_1                 = 0x00000002;        /*!< Bit 1 */
constexpr uint32_t  FLASH_ACR_LATENCY_2                 = 0x00000004;        /*!< Bit 2 */

constexpr uint32_t  FLASH_ACR_HLFCYA                    = 0x00000008;        /*!< Flash Half Cycle Access Enable */
constexpr uint32_t  FLASH_ACR_PRFTBE                    = 0x00000010;        /*!< Prefetch Buffer Enable */
constexpr uint32_t  FLASH_ACR_PRFTBS                    = 0x00000020;        /*!< Prefetch Buffer Status */

/******************  Bit definition for FLASH_KEYR register  ******************/
constexpr uint32_t  FLASH_KEYR_FKEYR                    = 0xFFFFFFFF;        /*!< FPEC Key */

constexpr uint32_t  RDP_KEY                             = 0x000000A5;        /*!< RDP Key */
constexpr uint32_t  FLASH_KEY1                          = 0x45670123;        /*!< FPEC Key1 */
constexpr uint32_t  FLASH_KEY2                          = 0xCDEF89AB;        /*!< FPEC Key2 */

/*****************  Bit definition for FLASH_OPTKEYR register  ****************/
constexpr uint32_t  FLASH_OPTKEYR_OPTKEYR               = 0xFFFFFFFF;        /*!< Option Byte Key */

constexpr uint32_t  FLASH_OPTKEY1                       = FLASH_KEY1;        /*!< Option Byte Key1 */
constexpr uint32_t  FLASH_OPTKEY2                       = FLASH_KEY2;        /*!< Option Byte Key2 */

/******************  Bit definition for FLASH_SR register  *******************/
constexpr uint32_t  FLASH_SR_BSY                        = 0x00000001;        /*!< Busy */
constexpr uint32_t  FLASH_SR_PGERR                      = 0x00000004;        /*!< Programming Error */
constexpr uint32_t  FLASH_SR_WRPERR                     = 0x00000010;        /*!< Write Protection Error */
constexpr uint32_t  FLASH_SR_EOP                        = 0x00000020;        /*!< End of operation */

/*******************  Bit definition for FLASH_CR register  *******************/
constexpr uint32_t  FLASH_CR_PG                         = 0x00000001;        /*!< Programming */
constexpr uint32_t  FLASH_CR_PER                        = 0x00000002;        /*!< Page Erase */
constexpr uint32_t  FLASH_CR_MER                        = 0x00000004;        /*!< Mass Erase */
constexpr uint32_t  FLASH_CR_OPTPG                      = 0x00000010;        /*!< Option Byte Programming */
constexpr uint32_t  FLASH_CR_OPTER                      = 0x00000020;        /*!< Option Byte Erase */
constexpr uint32_t  FLASH_CR_STRT                       = 0x00000040;        /*!< Start */
constexpr uint32_t  FLASH_CR_LOCK                       = 0x00000080;        /*!< Lock */
constexpr uint32_t  FLASH_CR_OPTWRE                     = 0x00000200;        /*!< Option Bytes Write Enable */
constexpr uint32_t  FLASH_CR_ERRIE                      = 0x00000400;        /*!< Error Interrupt Enable */
constexpr uint32_t  FLASH_CR_EOPIE                      = 0x00001000;        /*!< End of operation interrupt enable */
constexpr uint32_t  FLASH_CR_OBL_LAUNCH                 = 0x00002000;        /*!< OptionBytes Loader Launch */

/*******************  Bit definition for FLASH_AR register  *******************/
constexpr uint32_t  FLASH_AR_FAR                        = 0xFFFFFFFF;        /*!< Flash Address */

/******************  Bit definition for FLASH_OBR register  *******************/
constexpr uint32_t  FLASH_OBR_OPTERR                    = 0x00000001;        /*!< Option Byte Error */
constexpr uint32_t  FLASH_OBR_RDPRT                     = 0x00000006;        /*!< Read protection */
constexpr uint32_t  FLASH_OBR_RDPRT_1                   = 0x00000002;        /*!< Read protection Level 1 */
constexpr uint32_t  FLASH_OBR_RDPRT_2                   = 0x00000006;        /*!< Read protection Level 2 */

constexpr uint32_t  FLASH_OBR_USER                      = 0x00007700;        /*!< User Option Bytes */
constexpr uint32_t  FLASH_OBR_IWDG_SW                   = 0x00000100;        /*!< IWDG SW */
constexpr uint32_t  FLASH_OBR_nRST_STOP                 = 0x00000200;        /*!< nRST_STOP */
constexpr uint32_t  FLASH_OBR_nRST_STDBY                = 0x00000400;        /*!< nRST_STDBY */
constexpr uint32_t  FLASH_OBR_nBOOT1                    = 0x00001000;        /*!< nBOOT1 */
constexpr uint32_t  FLASH_OBR_VDDA_MONITOR              = 0x00002000;        /*!< VDDA_MONITOR */
constexpr uint32_t  FLASH_OBR_SRAM_PE                   = 0x00004000;        /*!< SRAM_PE */

/******************  Bit definition for FLASH_WRPR register  ******************/
constexpr uint32_t  FLASH_WRPR_WRP                        = 0xFFFFFFFF;      /*!< Write Protect */

/*----------------------------------------------------------------------------*/

/******************  Bit definition for OB_RDP register  **********************/
constexpr uint32_t  OB_RDP_RDP                          = 0x000000FF;        /*!< Read protection option byte */
constexpr uint32_t  OB_RDP_nRDP                         = 0x0000FF00;        /*!< Read protection complemented option byte */

/******************  Bit definition for OB_USER register  *********************/
constexpr uint32_t  OB_USER_USER                        = 0x00FF0000;        /*!< User option byte */
constexpr uint32_t  OB_USER_nUSER                       = 0xFF000000;        /*!< User complemented option byte */

/******************  Bit definition for FLASH_WRP0 register  ******************/
constexpr uint32_t  OB_WRP0_WRP0                        = 0x000000FF;        /*!< Flash memory write protection option bytes */
constexpr uint32_t  OB_WRP0_nWRP0                       = 0x0000FF00;        /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRP1 register  ******************/
constexpr uint32_t  OB_WRP1_WRP1                        = 0x00FF0000;        /*!< Flash memory write protection option bytes */
constexpr uint32_t  OB_WRP1_nWRP1                       = 0xFF000000;        /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRP2 register  ******************/
constexpr uint32_t  OB_WRP2_WRP2                        = 0x000000FF;        /*!< Flash memory write protection option bytes */
constexpr uint32_t  OB_WRP2_nWRP2                       = 0x0000FF00;        /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRP3 register  ******************/
constexpr uint32_t  OB_WRP3_WRP3                        = 0x00FF0000;        /*!< Flash memory write protection option bytes */
constexpr uint32_t  OB_WRP3_nWRP3                       = 0xFF000000;        /*!< Flash memory write protection complemented option bytes */


/******************************************************************************/
/*                                                                            */
/*                          Flexible Memory Controller                        */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for FMC_BCR1 register  *******************/
constexpr uint32_t  FMC_BCR1_MBKEN                     = 0x00000001;        /*!<Memory bank enable bit                 */
constexpr uint32_t  FMC_BCR1_MUXEN                     = 0x00000002;        /*!<Address/data multiplexing enable bit   */

constexpr uint32_t  FMC_BCR1_MTYP                      = 0x0000000C;        /*!<MTYP[1:0] bits = Memory type;           */
constexpr uint32_t  FMC_BCR1_MTYP_0                    = 0x00000004;        /*!<Bit 0 */
constexpr uint32_t  FMC_BCR1_MTYP_1                    = 0x00000008;        /*!<Bit 1 */

constexpr uint32_t  FMC_BCR1_MWID                      = 0x00000030;        /*!<MWID[1:0] bits = Memory data bus width; */
constexpr uint32_t  FMC_BCR1_MWID_0                    = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_BCR1_MWID_1                    = 0x00000020;        /*!<Bit 1 */

constexpr uint32_t  FMC_BCR1_FACCEN                    = 0x00000040;        /*!<Flash access enable        */
constexpr uint32_t  FMC_BCR1_BURSTEN                   = 0x00000100;        /*!<Burst enable bit           */
constexpr uint32_t  FMC_BCR1_WAITPOL                   = 0x00000200;        /*!<Wait signal polarity bit   */
constexpr uint32_t  FMC_BCR1_WRAPMOD                   = 0x00000400;        /*!<Wrapped burst mode support */
constexpr uint32_t  FMC_BCR1_WAITCFG                   = 0x00000800;        /*!<Wait timing configuration  */
constexpr uint32_t  FMC_BCR1_WREN                      = 0x00001000;        /*!<Write enable bit           */
constexpr uint32_t  FMC_BCR1_WAITEN                    = 0x00002000;        /*!<Wait enable bit            */
constexpr uint32_t  FMC_BCR1_EXTMOD                    = 0x00004000;        /*!<Extended mode enable       */
constexpr uint32_t  FMC_BCR1_ASYNCWAIT                 = 0x00008000;        /*!<Asynchronous wait          */
constexpr uint32_t  FMC_BCR1_CBURSTRW                  = 0x00080000;        /*!<Write burst enable         */
constexpr uint32_t  FMC_BCR1_CCLKEN                    = 0x00100000;        /*!<Continous clock enable     */

/******************  Bit definition for FMC_BCR2 register  *******************/
constexpr uint32_t  FMC_BCR2_MBKEN                     = 0x00000001;        /*!<Memory bank enable bit                 */
constexpr uint32_t  FMC_BCR2_MUXEN                     = 0x00000002;        /*!<Address/data multiplexing enable bit   */

constexpr uint32_t  FMC_BCR2_MTYP                      = 0x0000000C;        /*!<MTYP[1:0] bits = Memory type;           */
constexpr uint32_t  FMC_BCR2_MTYP_0                    = 0x00000004;        /*!<Bit 0 */
constexpr uint32_t  FMC_BCR2_MTYP_1                    = 0x00000008;        /*!<Bit 1 */

constexpr uint32_t  FMC_BCR2_MWID                      = 0x00000030;        /*!<MWID[1:0] bits = Memory data bus width; */
constexpr uint32_t  FMC_BCR2_MWID_0                    = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_BCR2_MWID_1                    = 0x00000020;        /*!<Bit 1 */

constexpr uint32_t  FMC_BCR2_FACCEN                    = 0x00000040;        /*!<Flash access enable        */
constexpr uint32_t  FMC_BCR2_BURSTEN                   = 0x00000100;        /*!<Burst enable bit           */
constexpr uint32_t  FMC_BCR2_WAITPOL                   = 0x00000200;        /*!<Wait signal polarity bit   */
constexpr uint32_t  FMC_BCR2_WRAPMOD                   = 0x00000400;        /*!<Wrapped burst mode support */
constexpr uint32_t  FMC_BCR2_WAITCFG                   = 0x00000800;        /*!<Wait timing configuration  */
constexpr uint32_t  FMC_BCR2_WREN                      = 0x00001000;        /*!<Write enable bit           */
constexpr uint32_t  FMC_BCR2_WAITEN                    = 0x00002000;        /*!<Wait enable bit            */
constexpr uint32_t  FMC_BCR2_EXTMOD                    = 0x00004000;        /*!<Extended mode enable       */
constexpr uint32_t  FMC_BCR2_ASYNCWAIT                 = 0x00008000;        /*!<Asynchronous wait          */
constexpr uint32_t  FMC_BCR2_CBURSTRW                  = 0x00080000;        /*!<Write burst enable         */

/******************  Bit definition for FMC_BCR3 register  *******************/
constexpr uint32_t  FMC_BCR3_MBKEN                     = 0x00000001;        /*!<Memory bank enable bit                 */
constexpr uint32_t  FMC_BCR3_MUXEN                     = 0x00000002;        /*!<Address/data multiplexing enable bit   */

constexpr uint32_t  FMC_BCR3_MTYP                      = 0x0000000C;        /*!<MTYP[1:0] bits = Memory type;           */
constexpr uint32_t  FMC_BCR3_MTYP_0                    = 0x00000004;        /*!<Bit 0 */
constexpr uint32_t  FMC_BCR3_MTYP_1                    = 0x00000008;        /*!<Bit 1 */

constexpr uint32_t  FMC_BCR3_MWID                      = 0x00000030;        /*!<MWID[1:0] bits = Memory data bus width; */
constexpr uint32_t  FMC_BCR3_MWID_0                    = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_BCR3_MWID_1                    = 0x00000020;        /*!<Bit 1 */

constexpr uint32_t  FMC_BCR3_FACCEN                    = 0x00000040;        /*!<Flash access enable        */
constexpr uint32_t  FMC_BCR3_BURSTEN                   = 0x00000100;        /*!<Burst enable bit           */
constexpr uint32_t  FMC_BCR3_WAITPOL                   = 0x00000200;        /*!<Wait signal polarity bit   */
constexpr uint32_t  FMC_BCR3_WRAPMOD                   = 0x00000400;        /*!<Wrapped burst mode support */
constexpr uint32_t  FMC_BCR3_WAITCFG                   = 0x00000800;        /*!<Wait timing configuration  */
constexpr uint32_t  FMC_BCR3_WREN                      = 0x00001000;        /*!<Write enable bit           */
constexpr uint32_t  FMC_BCR3_WAITEN                    = 0x00002000;        /*!<Wait enable bit            */
constexpr uint32_t  FMC_BCR3_EXTMOD                    = 0x00004000;        /*!<Extended mode enable       */
constexpr uint32_t  FMC_BCR3_ASYNCWAIT                 = 0x00008000;        /*!<Asynchronous wait          */
constexpr uint32_t  FMC_BCR3_CBURSTRW                  = 0x00080000;        /*!<Write burst enable         */

/******************  Bit definition for FMC_BCR4 register  *******************/
constexpr uint32_t  FMC_BCR4_MBKEN                     = 0x00000001;        /*!<Memory bank enable bit                 */
constexpr uint32_t  FMC_BCR4_MUXEN                     = 0x00000002;        /*!<Address/data multiplexing enable bit   */

constexpr uint32_t  FMC_BCR4_MTYP                      = 0x0000000C;        /*!<MTYP[1:0] bits = Memory type;           */
constexpr uint32_t  FMC_BCR4_MTYP_0                    = 0x00000004;        /*!<Bit 0 */
constexpr uint32_t  FMC_BCR4_MTYP_1                    = 0x00000008;        /*!<Bit 1 */

constexpr uint32_t  FMC_BCR4_MWID                      = 0x00000030;        /*!<MWID[1:0] bits = Memory data bus width; */
constexpr uint32_t  FMC_BCR4_MWID_0                    = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_BCR4_MWID_1                    = 0x00000020;        /*!<Bit 1 */

constexpr uint32_t  FMC_BCR4_FACCEN                    = 0x00000040;        /*!<Flash access enable        */
constexpr uint32_t  FMC_BCR4_BURSTEN                   = 0x00000100;        /*!<Burst enable bit           */
constexpr uint32_t  FMC_BCR4_WAITPOL                   = 0x00000200;        /*!<Wait signal polarity bit   */
constexpr uint32_t  FMC_BCR4_WRAPMOD                   = 0x00000400;        /*!<Wrapped burst mode support */
constexpr uint32_t  FMC_BCR4_WAITCFG                   = 0x00000800;        /*!<Wait timing configuration  */
constexpr uint32_t  FMC_BCR4_WREN                      = 0x00001000;        /*!<Write enable bit           */
constexpr uint32_t  FMC_BCR4_WAITEN                    = 0x00002000;        /*!<Wait enable bit            */
constexpr uint32_t  FMC_BCR4_EXTMOD                    = 0x00004000;        /*!<Extended mode enable       */
constexpr uint32_t  FMC_BCR4_ASYNCWAIT                 = 0x00008000;        /*!<Asynchronous wait          */
constexpr uint32_t  FMC_BCR4_CBURSTRW                  = 0x00080000;        /*!<Write burst enable         */

/******************  Bit definition for FMC_BTR1 register  ******************/
constexpr uint32_t  FMC_BTR1_ADDSET                    = 0x0000000F;        /*!<ADDSET[3:0] bits = Address setup phase duration; */
constexpr uint32_t  FMC_BTR1_ADDSET_0                  = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR1_ADDSET_1                  = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR1_ADDSET_2                  = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR1_ADDSET_3                  = 0x00000008;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR1_ADDHLD                    = 0x000000F0;        /*!<ADDHLD[3:0] bits = Address-hold phase duration;  */
constexpr uint32_t  FMC_BTR1_ADDHLD_0                  = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR1_ADDHLD_1                  = 0x00000020;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR1_ADDHLD_2                  = 0x00000040;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR1_ADDHLD_3                  = 0x00000080;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR1_DATAST                    = 0x0000FF00;        /*!<DATAST [3:0] bits = Data-phase duration; */
constexpr uint32_t  FMC_BTR1_DATAST_0                  = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR1_DATAST_1                  = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR1_DATAST_2                  = 0x00000400;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR1_DATAST_3                  = 0x00000800;        /*!<Bit 3 */
constexpr uint32_t  FMC_BTR1_DATAST_4                  = 0x00001000;        /*!<Bit 4 */
constexpr uint32_t  FMC_BTR1_DATAST_5                  = 0x00002000;        /*!<Bit 5 */
constexpr uint32_t  FMC_BTR1_DATAST_6                  = 0x00004000;        /*!<Bit 6 */
constexpr uint32_t  FMC_BTR1_DATAST_7                  = 0x00008000;        /*!<Bit 7 */

constexpr uint32_t  FMC_BTR1_BUSTURN                   = 0x000F0000;        /*!<BUSTURN[3:0] bits = Bus turnaround phase duration; */
constexpr uint32_t  FMC_BTR1_BUSTURN_0                 = 0x00010000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR1_BUSTURN_1                 = 0x00020000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR1_BUSTURN_2                 = 0x00040000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR1_BUSTURN_3                 = 0x00080000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR1_CLKDIV                    = 0x00F00000;        /*!<CLKDIV[3:0] bits = Clock divide ratio; */
constexpr uint32_t  FMC_BTR1_CLKDIV_0                  = 0x00100000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR1_CLKDIV_1                  = 0x00200000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR1_CLKDIV_2                  = 0x00400000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR1_CLKDIV_3                  = 0x00800000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR1_DATLAT                    = 0x0F000000;        /*!<DATLA[3:0] bits = Data latency; */
constexpr uint32_t  FMC_BTR1_DATLAT_0                  = 0x01000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR1_DATLAT_1                  = 0x02000000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR1_DATLAT_2                  = 0x04000000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR1_DATLAT_3                  = 0x08000000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR1_ACCMOD                    = 0x30000000;        /*!<ACCMOD[1:0] bits = Access mode; */
constexpr uint32_t  FMC_BTR1_ACCMOD_0                  = 0x10000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR1_ACCMOD_1                  = 0x20000000;        /*!<Bit 1 */

/******************  Bit definition for FMC_BTR2 register  *******************/
constexpr uint32_t  FMC_BTR2_ADDSET                    = 0x0000000F;        /*!<ADDSET[3:0] bits = Address setup phase duration; */
constexpr uint32_t  FMC_BTR2_ADDSET_0                  = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR2_ADDSET_1                  = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR2_ADDSET_2                  = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR2_ADDSET_3                  = 0x00000008;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR2_ADDHLD                    = 0x000000F0;        /*!<ADDHLD[3:0] bits = Address-hold phase duration; */
constexpr uint32_t  FMC_BTR2_ADDHLD_0                  = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR2_ADDHLD_1                  = 0x00000020;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR2_ADDHLD_2                  = 0x00000040;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR2_ADDHLD_3                  = 0x00000080;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR2_DATAST                    = 0x0000FF00;        /*!<DATAST [3:0] bits = Data-phase duration; */
constexpr uint32_t  FMC_BTR2_DATAST_0                  = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR2_DATAST_1                  = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR2_DATAST_2                  = 0x00000400;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR2_DATAST_3                  = 0x00000800;        /*!<Bit 3 */
constexpr uint32_t  FMC_BTR2_DATAST_4                  = 0x00001000;        /*!<Bit 4 */
constexpr uint32_t  FMC_BTR2_DATAST_5                  = 0x00002000;        /*!<Bit 5 */
constexpr uint32_t  FMC_BTR2_DATAST_6                  = 0x00004000;        /*!<Bit 6 */
constexpr uint32_t  FMC_BTR2_DATAST_7                  = 0x00008000;        /*!<Bit 7 */

constexpr uint32_t  FMC_BTR2_BUSTURN                   = 0x000F0000;        /*!<BUSTURN[3:0] bits = Bus turnaround phase duration; */
constexpr uint32_t  FMC_BTR2_BUSTURN_0                 = 0x00010000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR2_BUSTURN_1                 = 0x00020000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR2_BUSTURN_2                 = 0x00040000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR2_BUSTURN_3                 = 0x00080000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR2_CLKDIV                    = 0x00F00000;        /*!<CLKDIV[3:0] bits = Clock divide ratio; */
constexpr uint32_t  FMC_BTR2_CLKDIV_0                  = 0x00100000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR2_CLKDIV_1                  = 0x00200000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR2_CLKDIV_2                  = 0x00400000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR2_CLKDIV_3                  = 0x00800000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR2_DATLAT                    = 0x0F000000;        /*!<DATLA[3:0] bits = Data latency; */
constexpr uint32_t  FMC_BTR2_DATLAT_0                  = 0x01000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR2_DATLAT_1                  = 0x02000000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR2_DATLAT_2                  = 0x04000000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR2_DATLAT_3                  = 0x08000000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR2_ACCMOD                    = 0x30000000;        /*!<ACCMOD[1:0] bits = Access mode; */
constexpr uint32_t  FMC_BTR2_ACCMOD_0                  = 0x10000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR2_ACCMOD_1                  = 0x20000000;        /*!<Bit 1 */

/*******************  Bit definition for FMC_BTR3 register  *******************/
constexpr uint32_t  FMC_BTR3_ADDSET                    = 0x0000000F;        /*!<ADDSET[3:0] bits = Address setup phase duration; */
constexpr uint32_t  FMC_BTR3_ADDSET_0                  = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR3_ADDSET_1                  = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR3_ADDSET_2                  = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR3_ADDSET_3                  = 0x00000008;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR3_ADDHLD                    = 0x000000F0;        /*!<ADDHLD[3:0] bits = Address-hold phase duration; */
constexpr uint32_t  FMC_BTR3_ADDHLD_0                  = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR3_ADDHLD_1                  = 0x00000020;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR3_ADDHLD_2                  = 0x00000040;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR3_ADDHLD_3                  = 0x00000080;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR3_DATAST                    = 0x0000FF00;        /*!<DATAST [3:0] bits = Data-phase duration; */
constexpr uint32_t  FMC_BTR3_DATAST_0                  = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR3_DATAST_1                  = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR3_DATAST_2                  = 0x00000400;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR3_DATAST_3                  = 0x00000800;        /*!<Bit 3 */
constexpr uint32_t  FMC_BTR3_DATAST_4                  = 0x00001000;        /*!<Bit 4 */
constexpr uint32_t  FMC_BTR3_DATAST_5                  = 0x00002000;        /*!<Bit 5 */
constexpr uint32_t  FMC_BTR3_DATAST_6                  = 0x00004000;        /*!<Bit 6 */
constexpr uint32_t  FMC_BTR3_DATAST_7                  = 0x00008000;        /*!<Bit 7 */

constexpr uint32_t  FMC_BTR3_BUSTURN                   = 0x000F0000;        /*!<BUSTURN[3:0] bits = Bus turnaround phase duration; */
constexpr uint32_t  FMC_BTR3_BUSTURN_0                 = 0x00010000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR3_BUSTURN_1                 = 0x00020000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR3_BUSTURN_2                 = 0x00040000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR3_BUSTURN_3                 = 0x00080000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR3_CLKDIV                    = 0x00F00000;        /*!<CLKDIV[3:0] bits = Clock divide ratio; */
constexpr uint32_t  FMC_BTR3_CLKDIV_0                  = 0x00100000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR3_CLKDIV_1                  = 0x00200000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR3_CLKDIV_2                  = 0x00400000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR3_CLKDIV_3                  = 0x00800000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR3_DATLAT                    = 0x0F000000;        /*!<DATLA[3:0] bits = Data latency; */
constexpr uint32_t  FMC_BTR3_DATLAT_0                  = 0x01000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR3_DATLAT_1                  = 0x02000000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR3_DATLAT_2                  = 0x04000000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR3_DATLAT_3                  = 0x08000000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR3_ACCMOD                    = 0x30000000;        /*!<ACCMOD[1:0] bits = Access mode; */
constexpr uint32_t  FMC_BTR3_ACCMOD_0                  = 0x10000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR3_ACCMOD_1                  = 0x20000000;        /*!<Bit 1 */

/******************  Bit definition for FMC_BTR4 register  *******************/
constexpr uint32_t  FMC_BTR4_ADDSET                    = 0x0000000F;        /*!<ADDSET[3:0] bits = Address setup phase duration; */
constexpr uint32_t  FMC_BTR4_ADDSET_0                  = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR4_ADDSET_1                  = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR4_ADDSET_2                  = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR4_ADDSET_3                  = 0x00000008;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR4_ADDHLD                    = 0x000000F0;        /*!<ADDHLD[3:0] bits = Address-hold phase duration; */
constexpr uint32_t  FMC_BTR4_ADDHLD_0                  = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR4_ADDHLD_1                  = 0x00000020;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR4_ADDHLD_2                  = 0x00000040;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR4_ADDHLD_3                  = 0x00000080;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR4_DATAST                    = 0x0000FF00;        /*!<DATAST [3:0] bits = Data-phase duration; */
constexpr uint32_t  FMC_BTR4_DATAST_0                  = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR4_DATAST_1                  = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR4_DATAST_2                  = 0x00000400;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR4_DATAST_3                  = 0x00000800;        /*!<Bit 3 */
constexpr uint32_t  FMC_BTR4_DATAST_4                  = 0x00001000;        /*!<Bit 4 */
constexpr uint32_t  FMC_BTR4_DATAST_5                  = 0x00002000;        /*!<Bit 5 */
constexpr uint32_t  FMC_BTR4_DATAST_6                  = 0x00004000;        /*!<Bit 6 */
constexpr uint32_t  FMC_BTR4_DATAST_7                  = 0x00008000;        /*!<Bit 7 */

constexpr uint32_t  FMC_BTR4_BUSTURN                   = 0x000F0000;        /*!<BUSTURN[3:0] bits = Bus turnaround phase duration; */
constexpr uint32_t  FMC_BTR4_BUSTURN_0                 = 0x00010000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR4_BUSTURN_1                 = 0x00020000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR4_BUSTURN_2                 = 0x00040000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR4_BUSTURN_3                 = 0x00080000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR4_CLKDIV                    = 0x00F00000;        /*!<CLKDIV[3:0] bits = Clock divide ratio; */
constexpr uint32_t  FMC_BTR4_CLKDIV_0                  = 0x00100000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR4_CLKDIV_1                  = 0x00200000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR4_CLKDIV_2                  = 0x00400000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR4_CLKDIV_3                  = 0x00800000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR4_DATLAT                    = 0x0F000000;        /*!<DATLA[3:0] bits = Data latency; */
constexpr uint32_t  FMC_BTR4_DATLAT_0                  = 0x01000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR4_DATLAT_1                  = 0x02000000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BTR4_DATLAT_2                  = 0x04000000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BTR4_DATLAT_3                  = 0x08000000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BTR4_ACCMOD                    = 0x30000000;        /*!<ACCMOD[1:0] bits = Access mode; */
constexpr uint32_t  FMC_BTR4_ACCMOD_0                  = 0x10000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BTR4_ACCMOD_1                  = 0x20000000;        /*!<Bit 1 */

/******************  Bit definition for FMC_BWTR1 register  ******************/
constexpr uint32_t  FMC_BWTR1_ADDSET                   = 0x0000000F;        /*!<ADDSET[3:0] bits = Address setup phase duration; */
constexpr uint32_t  FMC_BWTR1_ADDSET_0                 = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR1_ADDSET_1                 = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR1_ADDSET_2                 = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR1_ADDSET_3                 = 0x00000008;        /*!<Bit 3 */

constexpr uint32_t  FMC_BWTR1_ADDHLD                   = 0x000000F0;        /*!<ADDHLD[3:0] bits = Address-hold phase duration; */
constexpr uint32_t  FMC_BWTR1_ADDHLD_0                 = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR1_ADDHLD_1                 = 0x00000020;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR1_ADDHLD_2                 = 0x00000040;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR1_ADDHLD_3                 = 0x00000080;        /*!<Bit 3 */

constexpr uint32_t  FMC_BWTR1_DATAST                   = 0x0000FF00;        /*!<DATAST [3:0] bits = Data-phase duration; */
constexpr uint32_t  FMC_BWTR1_DATAST_0                 = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR1_DATAST_1                 = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR1_DATAST_2                 = 0x00000400;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR1_DATAST_3                 = 0x00000800;        /*!<Bit 3 */
constexpr uint32_t  FMC_BWTR1_DATAST_4                 = 0x00001000;        /*!<Bit 4 */
constexpr uint32_t  FMC_BWTR1_DATAST_5                 = 0x00002000;        /*!<Bit 5 */
constexpr uint32_t  FMC_BWTR1_DATAST_6                 = 0x00004000;        /*!<Bit 6 */
constexpr uint32_t  FMC_BWTR1_DATAST_7                 = 0x00008000;        /*!<Bit 7 */

constexpr uint32_t  FMC_BWTR1_CLKDIV                   = 0x00F00000;        /*!<CLKDIV[3:0] bits = Clock divide ratio; */
constexpr uint32_t  FMC_BWTR1_CLKDIV_0                 = 0x00100000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR1_CLKDIV_1                 = 0x00200000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR1_CLKDIV_2                 = 0x00400000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR1_CLKDIV_3                 = 0x00800000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BWTR1_DATLAT                   = 0x0F000000;        /*!<DATLA[3:0] bits = Data latency; */
constexpr uint32_t  FMC_BWTR1_DATLAT_0                 = 0x01000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR1_DATLAT_1                 = 0x02000000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR1_DATLAT_2                 = 0x04000000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR1_DATLAT_3                 = 0x08000000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BWTR1_ACCMOD                   = 0x30000000;        /*!<ACCMOD[1:0] bits = Access mode; */
constexpr uint32_t  FMC_BWTR1_ACCMOD_0                 = 0x10000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR1_ACCMOD_1                 = 0x20000000;        /*!<Bit 1 */

/******************  Bit definition for FMC_BWTR2 register  ******************/
constexpr uint32_t  FMC_BWTR2_ADDSET                   = 0x0000000F;        /*!<ADDSET[3:0] bits = Address setup phase duration; */
constexpr uint32_t  FMC_BWTR2_ADDSET_0                 = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR2_ADDSET_1                 = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR2_ADDSET_2                 = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR2_ADDSET_3                 = 0x00000008;        /*!<Bit 3 */

constexpr uint32_t  FMC_BWTR2_ADDHLD                   = 0x000000F0;        /*!<ADDHLD[3:0] bits = Address-hold phase duration; */
constexpr uint32_t  FMC_BWTR2_ADDHLD_0                 = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR2_ADDHLD_1                 = 0x00000020;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR2_ADDHLD_2                 = 0x00000040;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR2_ADDHLD_3                 = 0x00000080;        /*!<Bit 3 */

constexpr uint32_t  FMC_BWTR2_DATAST                   = 0x0000FF00;        /*!<DATAST [3:0] bits = Data-phase duration; */
constexpr uint32_t  FMC_BWTR2_DATAST_0                 = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR2_DATAST_1                 = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR2_DATAST_2                 = 0x00000400;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR2_DATAST_3                 = 0x00000800;        /*!<Bit 3 */
constexpr uint32_t  FMC_BWTR2_DATAST_4                 = 0x00001000;        /*!<Bit 4 */
constexpr uint32_t  FMC_BWTR2_DATAST_5                 = 0x00002000;        /*!<Bit 5 */
constexpr uint32_t  FMC_BWTR2_DATAST_6                 = 0x00004000;        /*!<Bit 6 */
constexpr uint32_t  FMC_BWTR2_DATAST_7                 = 0x00008000;        /*!<Bit 7 */

constexpr uint32_t  FMC_BWTR2_CLKDIV                   = 0x00F00000;        /*!<CLKDIV[3:0] bits = Clock divide ratio; */
constexpr uint32_t  FMC_BWTR2_CLKDIV_0                 = 0x00100000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR2_CLKDIV_1                 = 0x00200000;        /*!<Bit 1*/
constexpr uint32_t  FMC_BWTR2_CLKDIV_2                 = 0x00400000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR2_CLKDIV_3                 = 0x00800000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BWTR2_DATLAT                   = 0x0F000000;        /*!<DATLA[3:0] bits = Data latency; */
constexpr uint32_t  FMC_BWTR2_DATLAT_0                 = 0x01000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR2_DATLAT_1                 = 0x02000000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR2_DATLAT_2                 = 0x04000000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR2_DATLAT_3                 = 0x08000000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BWTR2_ACCMOD                   = 0x30000000;        /*!<ACCMOD[1:0] bits = Access mode; */
constexpr uint32_t  FMC_BWTR2_ACCMOD_0                 = 0x10000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR2_ACCMOD_1                 = 0x20000000;        /*!<Bit 1 */

/******************  Bit definition for FMC_BWTR3 register  ******************/
constexpr uint32_t  FMC_BWTR3_ADDSET                   = 0x0000000F;        /*!<ADDSET[3:0] bits = Address setup phase duration; */
constexpr uint32_t  FMC_BWTR3_ADDSET_0                 = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR3_ADDSET_1                 = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR3_ADDSET_2                 = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR3_ADDSET_3                 = 0x00000008;        /*!<Bit 3 */

constexpr uint32_t  FMC_BWTR3_ADDHLD                   = 0x000000F0;        /*!<ADDHLD[3:0] bits = Address-hold phase duration; */
constexpr uint32_t  FMC_BWTR3_ADDHLD_0                 = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR3_ADDHLD_1                 = 0x00000020;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR3_ADDHLD_2                 = 0x00000040;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR3_ADDHLD_3                 = 0x00000080;        /*!<Bit 3 */

constexpr uint32_t  FMC_BWTR3_DATAST                   = 0x0000FF00;        /*!<DATAST [3:0] bits = Data-phase duration; */
constexpr uint32_t  FMC_BWTR3_DATAST_0                 = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR3_DATAST_1                 = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR3_DATAST_2                 = 0x00000400;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR3_DATAST_3                 = 0x00000800;        /*!<Bit 3 */
constexpr uint32_t  FMC_BWTR3_DATAST_4                 = 0x00001000;        /*!<Bit 4 */
constexpr uint32_t  FMC_BWTR3_DATAST_5                 = 0x00002000;        /*!<Bit 5 */
constexpr uint32_t  FMC_BWTR3_DATAST_6                 = 0x00004000;        /*!<Bit 6 */
constexpr uint32_t  FMC_BWTR3_DATAST_7                 = 0x00008000;        /*!<Bit 7 */

constexpr uint32_t  FMC_BWTR3_CLKDIV                   = 0x00F00000;        /*!<CLKDIV[3:0] bits = Clock divide ratio; */
constexpr uint32_t  FMC_BWTR3_CLKDIV_0                 = 0x00100000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR3_CLKDIV_1                 = 0x00200000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR3_CLKDIV_2                 = 0x00400000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR3_CLKDIV_3                 = 0x00800000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BWTR3_DATLAT                   = 0x0F000000;        /*!<DATLA[3:0] bits = Data latency; */
constexpr uint32_t  FMC_BWTR3_DATLAT_0                 = 0x01000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR3_DATLAT_1                 = 0x02000000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR3_DATLAT_2                 = 0x04000000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR3_DATLAT_3                 = 0x08000000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BWTR3_ACCMOD                   = 0x30000000;        /*!<ACCMOD[1:0] bits = Access mode; */
constexpr uint32_t  FMC_BWTR3_ACCMOD_0                 = 0x10000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR3_ACCMOD_1                 = 0x20000000;        /*!<Bit 1 */

/******************  Bit definition for FMC_BWTR4 register  ******************/
constexpr uint32_t  FMC_BWTR4_ADDSET                   = 0x0000000F;        /*!<ADDSET[3:0] bits = Address setup phase duration; */
constexpr uint32_t  FMC_BWTR4_ADDSET_0                 = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR4_ADDSET_1                 = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR4_ADDSET_2                 = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR4_ADDSET_3                 = 0x00000008;        /*!<Bit 3 */

constexpr uint32_t  FMC_BWTR4_ADDHLD                   = 0x000000F0;        /*!<ADDHLD[3:0] bits = Address-hold phase duration; */
constexpr uint32_t  FMC_BWTR4_ADDHLD_0                 = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR4_ADDHLD_1                 = 0x00000020;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR4_ADDHLD_2                 = 0x00000040;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR4_ADDHLD_3                 = 0x00000080;        /*!<Bit 3 */

constexpr uint32_t  FMC_BWTR4_DATAST                   = 0x0000FF00;        /*!<DATAST [3:0] bits = Data-phase duration; */
constexpr uint32_t  FMC_BWTR4_DATAST_0                 = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR4_DATAST_1                 = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR4_DATAST_2                 = 0x00000400;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR4_DATAST_3                 = 0x00000800;        /*!<Bit 3 */
constexpr uint32_t  FMC_BWTR4_DATAST_4                 = 0x00001000;        /*!<Bit 4 */
constexpr uint32_t  FMC_BWTR4_DATAST_5                 = 0x00002000;        /*!<Bit 5 */
constexpr uint32_t  FMC_BWTR4_DATAST_6                 = 0x00004000;        /*!<Bit 6 */
constexpr uint32_t  FMC_BWTR4_DATAST_7                 = 0x00008000;        /*!<Bit 7 */

constexpr uint32_t  FMC_BWTR4_CLKDIV                   = 0x00F00000;        /*!<CLKDIV[3:0] bits = Clock divide ratio; */
constexpr uint32_t  FMC_BWTR4_CLKDIV_0                 = 0x00100000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR4_CLKDIV_1                 = 0x00200000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR4_CLKDIV_2                 = 0x00400000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR4_CLKDIV_3                 = 0x00800000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BWTR4_DATLAT                   = 0x0F000000;        /*!<DATLA[3:0] bits = Data latency; */
constexpr uint32_t  FMC_BWTR4_DATLAT_0                 = 0x01000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR4_DATLAT_1                 = 0x02000000;        /*!<Bit 1 */
constexpr uint32_t  FMC_BWTR4_DATLAT_2                 = 0x04000000;        /*!<Bit 2 */
constexpr uint32_t  FMC_BWTR4_DATLAT_3                 = 0x08000000;        /*!<Bit 3 */

constexpr uint32_t  FMC_BWTR4_ACCMOD                   = 0x30000000;        /*!<ACCMOD[1:0] bits = Access mode; */
constexpr uint32_t  FMC_BWTR4_ACCMOD_0                 = 0x10000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_BWTR4_ACCMOD_1                 = 0x20000000;        /*!<Bit 1 */

/******************  Bit definition for FMC_PCR2 register  *******************/
constexpr uint32_t  FMC_PCR2_PWAITEN                   = 0x00000002;        /*!<Wait feature enable bit                   */
constexpr uint32_t  FMC_PCR2_PBKEN                     = 0x00000004;        /*!<PC Card/NAND Flash memory bank enable bit */
constexpr uint32_t  FMC_PCR2_PTYP                      = 0x00000008;        /*!<Memory type                               */

constexpr uint32_t  FMC_PCR2_PWID                      = 0x00000030;        /*!<PWID[1:0] bits = NAND Flash databus width; */
constexpr uint32_t  FMC_PCR2_PWID_0                    = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_PCR2_PWID_1                    = 0x00000020;        /*!<Bit 1 */

constexpr uint32_t  FMC_PCR2_ECCEN                     = 0x00000040;        /*!<ECC computation logic enable bit          */

constexpr uint32_t  FMC_PCR2_TCLR                      = 0x00001E00;        /*!<TCLR[3:0] bits = CLE to RE delay;          */
constexpr uint32_t  FMC_PCR2_TCLR_0                    = 0x00000200;        /*!<Bit 0 */
constexpr uint32_t  FMC_PCR2_TCLR_1                    = 0x00000400;        /*!<Bit 1 */
constexpr uint32_t  FMC_PCR2_TCLR_2                    = 0x00000800;        /*!<Bit 2 */
constexpr uint32_t  FMC_PCR2_TCLR_3                    = 0x00001000;        /*!<Bit 3 */

constexpr uint32_t  FMC_PCR2_TAR                       = 0x0001E000;        /*!<TAR[3:0] bits = ALE to RE delay;           */
constexpr uint32_t  FMC_PCR2_TAR_0                     = 0x00002000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PCR2_TAR_1                     = 0x00004000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PCR2_TAR_2                     = 0x00008000;        /*!<Bit 2 */
constexpr uint32_t  FMC_PCR2_TAR_3                     = 0x00010000;        /*!<Bit 3 */

constexpr uint32_t  FMC_PCR2_ECCPS                     = 0x000E0000;        /*!<ECCPS[1:0] bits = ECC page size;           */
constexpr uint32_t  FMC_PCR2_ECCPS_0                   = 0x00020000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PCR2_ECCPS_1                   = 0x00040000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PCR2_ECCPS_2                   = 0x00080000;        /*!<Bit 2 */

/******************  Bit definition for FMC_PCR3 register  *******************/
constexpr uint32_t  FMC_PCR3_PWAITEN                   = 0x00000002;        /*!<Wait feature enable bit                   */
constexpr uint32_t  FMC_PCR3_PBKEN                     = 0x00000004;        /*!<PC Card/NAND Flash memory bank enable bit */
constexpr uint32_t  FMC_PCR3_PTYP                      = 0x00000008;        /*!<Memory type                               */

constexpr uint32_t  FMC_PCR3_PWID                      = 0x00000030;        /*!<PWID[1:0] bits = NAND Flash databus width; */
constexpr uint32_t  FMC_PCR3_PWID_0                    = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_PCR3_PWID_1                    = 0x00000020;        /*!<Bit 1 */

constexpr uint32_t  FMC_PCR3_ECCEN                     = 0x00000040;        /*!<ECC computation logic enable bit          */

constexpr uint32_t  FMC_PCR3_TCLR                      = 0x00001E00;        /*!<TCLR[3:0] bits = CLE to RE delay;          */
constexpr uint32_t  FMC_PCR3_TCLR_0                    = 0x00000200;        /*!<Bit 0 */
constexpr uint32_t  FMC_PCR3_TCLR_1                    = 0x00000400;        /*!<Bit 1 */
constexpr uint32_t  FMC_PCR3_TCLR_2                    = 0x00000800;        /*!<Bit 2 */
constexpr uint32_t  FMC_PCR3_TCLR_3                    = 0x00001000;        /*!<Bit 3 */

constexpr uint32_t  FMC_PCR3_TAR                       = 0x0001E000;        /*!<TAR[3:0] bits = ALE to RE delay;           */
constexpr uint32_t  FMC_PCR3_TAR_0                     = 0x00002000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PCR3_TAR_1                     = 0x00004000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PCR3_TAR_2                     = 0x00008000;        /*!<Bit 2 */
constexpr uint32_t  FMC_PCR3_TAR_3                     = 0x00010000;        /*!<Bit 3 */

constexpr uint32_t  FMC_PCR3_ECCPS                     = 0x000E0000;        /*!<ECCPS[2:0] bits = ECC page size;           */
constexpr uint32_t  FMC_PCR3_ECCPS_0                   = 0x00020000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PCR3_ECCPS_1                   = 0x00040000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PCR3_ECCPS_2                   = 0x00080000;        /*!<Bit 2 */

/******************  Bit definition for FMC_PCR4 register  *******************/
constexpr uint32_t  FMC_PCR4_PWAITEN                   = 0x00000002;        /*!<Wait feature enable bit                   */
constexpr uint32_t  FMC_PCR4_PBKEN                     = 0x00000004;        /*!<PC Card/NAND Flash memory bank enable bit */
constexpr uint32_t  FMC_PCR4_PTYP                      = 0x00000008;        /*!<Memory type                               */

constexpr uint32_t  FMC_PCR4_PWID                      = 0x00000030;        /*!<PWID[1:0] bits = NAND Flash databus width; */
constexpr uint32_t  FMC_PCR4_PWID_0                    = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_PCR4_PWID_1                    = 0x00000020;        /*!<Bit 1 */

constexpr uint32_t  FMC_PCR4_ECCEN                     = 0x00000040;        /*!<ECC computation logic enable bit          */

constexpr uint32_t  FMC_PCR4_TCLR                      = 0x00001E00;        /*!<TCLR[3:0] bits = CLE to RE delay;          */
constexpr uint32_t  FMC_PCR4_TCLR_0                    = 0x00000200;        /*!<Bit 0 */
constexpr uint32_t  FMC_PCR4_TCLR_1                    = 0x00000400;        /*!<Bit 1 */
constexpr uint32_t  FMC_PCR4_TCLR_2                    = 0x00000800;        /*!<Bit 2 */
constexpr uint32_t  FMC_PCR4_TCLR_3                    = 0x00001000;        /*!<Bit 3 */

constexpr uint32_t  FMC_PCR4_TAR                       = 0x0001E000;        /*!<TAR[3:0] bits = ALE to RE delay;           */
constexpr uint32_t  FMC_PCR4_TAR_0                     = 0x00002000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PCR4_TAR_1                     = 0x00004000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PCR4_TAR_2                     = 0x00008000;        /*!<Bit 2 */
constexpr uint32_t  FMC_PCR4_TAR_3                     = 0x00010000;        /*!<Bit 3 */

constexpr uint32_t  FMC_PCR4_ECCPS                     = 0x000E0000;        /*!<ECCPS[2:0] bits = ECC page size;           */
constexpr uint32_t  FMC_PCR4_ECCPS_0                   = 0x00020000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PCR4_ECCPS_1                   = 0x00040000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PCR4_ECCPS_2                   = 0x00080000;        /*!<Bit 2 */

/*******************  Bit definition for FMC_SR2 register  *******************/
constexpr uint32_t  FMC_SR2_IRS                        = 0x01;               /*!<Interrupt Rising Edge status                */
constexpr uint32_t  FMC_SR2_ILS                        = 0x02;               /*!<Interrupt Level status                      */
constexpr uint32_t  FMC_SR2_IFS                        = 0x04;               /*!<Interrupt Falling Edge status               */
constexpr uint32_t  FMC_SR2_IREN                       = 0x08;               /*!<Interrupt Rising Edge detection Enable bit  */
constexpr uint32_t  FMC_SR2_ILEN                       = 0x10;               /*!<Interrupt Level detection Enable bit        */
constexpr uint32_t  FMC_SR2_IFEN                       = 0x20;               /*!<Interrupt Falling Edge detection Enable bit */
constexpr uint32_t  FMC_SR2_FEMPT                      = 0x40;               /*!<FIFO empty                                  */

/*******************  Bit definition for FMC_SR3 register  *******************/
constexpr uint32_t  FMC_SR3_IRS                        = 0x01;               /*!<Interrupt Rising Edge status                */
constexpr uint32_t  FMC_SR3_ILS                        = 0x02;               /*!<Interrupt Level status                      */
constexpr uint32_t  FMC_SR3_IFS                        = 0x04;               /*!<Interrupt Falling Edge status               */
constexpr uint32_t  FMC_SR3_IREN                       = 0x08;               /*!<Interrupt Rising Edge detection Enable bit  */
constexpr uint32_t  FMC_SR3_ILEN                       = 0x10;               /*!<Interrupt Level detection Enable bit        */
constexpr uint32_t  FMC_SR3_IFEN                       = 0x20;               /*!<Interrupt Falling Edge detection Enable bit */
constexpr uint32_t  FMC_SR3_FEMPT                      = 0x40;               /*!<FIFO empty                                  */

/*******************  Bit definition for FMC_SR4 register  *******************/
constexpr uint32_t  FMC_SR4_IRS                        = 0x01;               /*!<Interrupt Rising Edge status                */
constexpr uint32_t  FMC_SR4_ILS                        = 0x02;               /*!<Interrupt Level status                      */
constexpr uint32_t  FMC_SR4_IFS                        = 0x04;               /*!<Interrupt Falling Edge status               */
constexpr uint32_t  FMC_SR4_IREN                       = 0x08;               /*!<Interrupt Rising Edge detection Enable bit  */
constexpr uint32_t  FMC_SR4_ILEN                       = 0x10;               /*!<Interrupt Level detection Enable bit        */
constexpr uint32_t  FMC_SR4_IFEN                       = 0x20;               /*!<Interrupt Falling Edge detection Enable bit */
constexpr uint32_t  FMC_SR4_FEMPT                      = 0x40;               /*!<FIFO empty                                  */

/******************  Bit definition for FMC_PMEM2 register  ******************/
constexpr uint32_t  FMC_PMEM2_MEMSET2                  = 0x000000FF;        /*!<MEMSET2[7:0] bits = Common memory 2 setup time; */
constexpr uint32_t  FMC_PMEM2_MEMSET2_0                = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_PMEM2_MEMSET2_1                = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_PMEM2_MEMSET2_2                = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  FMC_PMEM2_MEMSET2_3                = 0x00000008;        /*!<Bit 3 */
constexpr uint32_t  FMC_PMEM2_MEMSET2_4                = 0x00000010;        /*!<Bit 4 */
constexpr uint32_t  FMC_PMEM2_MEMSET2_5                = 0x00000020;        /*!<Bit 5 */
constexpr uint32_t  FMC_PMEM2_MEMSET2_6                = 0x00000040;        /*!<Bit 6 */
constexpr uint32_t  FMC_PMEM2_MEMSET2_7                = 0x00000080;        /*!<Bit 7 */

constexpr uint32_t  FMC_PMEM2_MEMWAIT2                 = 0x0000FF00;        /*!<MEMWAIT2[7:0] bits = Common memory 2 wait time; */
constexpr uint32_t  FMC_PMEM2_MEMWAIT2_0               = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  FMC_PMEM2_MEMWAIT2_1               = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  FMC_PMEM2_MEMWAIT2_2               = 0x00000400;        /*!<Bit 2 */
constexpr uint32_t  FMC_PMEM2_MEMWAIT2_3               = 0x00000800;        /*!<Bit 3 */
constexpr uint32_t  FMC_PMEM2_MEMWAIT2_4               = 0x00001000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PMEM2_MEMWAIT2_5               = 0x00002000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PMEM2_MEMWAIT2_6               = 0x00004000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PMEM2_MEMWAIT2_7               = 0x00008000;        /*!<Bit 7 */

constexpr uint32_t  FMC_PMEM2_MEMHOLD2                 = 0x00FF0000;        /*!<MEMHOLD2[7:0] bits = Common memory 2 hold time; */
constexpr uint32_t  FMC_PMEM2_MEMHOLD2_0               = 0x00010000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PMEM2_MEMHOLD2_1               = 0x00020000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PMEM2_MEMHOLD2_2               = 0x00040000;        /*!<Bit 2 */
constexpr uint32_t  FMC_PMEM2_MEMHOLD2_3               = 0x00080000;        /*!<Bit 3 */
constexpr uint32_t  FMC_PMEM2_MEMHOLD2_4               = 0x00100000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PMEM2_MEMHOLD2_5               = 0x00200000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PMEM2_MEMHOLD2_6               = 0x00400000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PMEM2_MEMHOLD2_7               = 0x00800000;        /*!<Bit 7 */

constexpr uint32_t  FMC_PMEM2_MEMHIZ2                  = 0xFF000000;        /*!<MEMHIZ2[7:0] bits = Common memory 2 databus HiZ time; */
constexpr uint32_t  FMC_PMEM2_MEMHIZ2_0                = 0x01000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PMEM2_MEMHIZ2_1                = 0x02000000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PMEM2_MEMHIZ2_2                = 0x04000000;        /*!<Bit 2 */
constexpr uint32_t  FMC_PMEM2_MEMHIZ2_3                = 0x08000000;        /*!<Bit 3 */
constexpr uint32_t  FMC_PMEM2_MEMHIZ2_4                = 0x10000000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PMEM2_MEMHIZ2_5                = 0x20000000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PMEM2_MEMHIZ2_6                = 0x40000000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PMEM2_MEMHIZ2_7                = 0x80000000;        /*!<Bit 7 */

/******************  Bit definition for FMC_PMEM3 register  ******************/
constexpr uint32_t  FMC_PMEM3_MEMSET3                  = 0x000000FF;        /*!<MEMSET3[7:0] bits = Common memory 3 setup time; */
constexpr uint32_t  FMC_PMEM3_MEMSET3_0                = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_PMEM3_MEMSET3_1                = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_PMEM3_MEMSET3_2                = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  FMC_PMEM3_MEMSET3_3                = 0x00000008;        /*!<Bit 3 */
constexpr uint32_t  FMC_PMEM3_MEMSET3_4                = 0x00000010;        /*!<Bit 4 */
constexpr uint32_t  FMC_PMEM3_MEMSET3_5                = 0x00000020;        /*!<Bit 5 */
constexpr uint32_t  FMC_PMEM3_MEMSET3_6                = 0x00000040;        /*!<Bit 6 */
constexpr uint32_t  FMC_PMEM3_MEMSET3_7                = 0x00000080;        /*!<Bit 7 */

constexpr uint32_t  FMC_PMEM3_MEMWAIT3                 = 0x0000FF00;        /*!<MEMWAIT3[7:0] bits = Common memory 3 wait time; */
constexpr uint32_t  FMC_PMEM3_MEMWAIT3_0               = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  FMC_PMEM3_MEMWAIT3_1               = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  FMC_PMEM3_MEMWAIT3_2               = 0x00000400;        /*!<Bit 2 */
constexpr uint32_t  FMC_PMEM3_MEMWAIT3_3               = 0x00000800;        /*!<Bit 3 */
constexpr uint32_t  FMC_PMEM3_MEMWAIT3_4               = 0x00001000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PMEM3_MEMWAIT3_5               = 0x00002000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PMEM3_MEMWAIT3_6               = 0x00004000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PMEM3_MEMWAIT3_7               = 0x00008000;        /*!<Bit 7 */

constexpr uint32_t  FMC_PMEM3_MEMHOLD3                 = 0x00FF0000;        /*!<MEMHOLD3[7:0] bits = Common memory 3 hold time; */
constexpr uint32_t  FMC_PMEM3_MEMHOLD3_0               = 0x00010000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PMEM3_MEMHOLD3_1               = 0x00020000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PMEM3_MEMHOLD3_2               = 0x00040000;        /*!<Bit 2 */
constexpr uint32_t  FMC_PMEM3_MEMHOLD3_3               = 0x00080000;        /*!<Bit 3 */
constexpr uint32_t  FMC_PMEM3_MEMHOLD3_4               = 0x00100000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PMEM3_MEMHOLD3_5               = 0x00200000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PMEM3_MEMHOLD3_6               = 0x00400000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PMEM3_MEMHOLD3_7               = 0x00800000;        /*!<Bit 7 */

constexpr uint32_t  FMC_PMEM3_MEMHIZ3                  = 0xFF000000;        /*!<MEMHIZ3[7:0] bits = Common memory 3 databus HiZ time; */
constexpr uint32_t  FMC_PMEM3_MEMHIZ3_0                = 0x01000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PMEM3_MEMHIZ3_1                = 0x02000000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PMEM3_MEMHIZ3_2                = 0x04000000;        /*!<Bit 2 */
constexpr uint32_t  FMC_PMEM3_MEMHIZ3_3                = 0x08000000;        /*!<Bit 3 */
constexpr uint32_t  FMC_PMEM3_MEMHIZ3_4                = 0x10000000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PMEM3_MEMHIZ3_5                = 0x20000000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PMEM3_MEMHIZ3_6                = 0x40000000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PMEM3_MEMHIZ3_7                = 0x80000000;        /*!<Bit 7 */

/******************  Bit definition for FMC_PMEM4 register  ******************/
constexpr uint32_t  FMC_PMEM4_MEMSET4                  = 0x000000FF;        /*!<MEMSET4[7:0] bits = Common memory 4 setup time; */
constexpr uint32_t  FMC_PMEM4_MEMSET4_0                = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_PMEM4_MEMSET4_1                = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_PMEM4_MEMSET4_2                = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  FMC_PMEM4_MEMSET4_3                = 0x00000008;        /*!<Bit 3 */
constexpr uint32_t  FMC_PMEM4_MEMSET4_4                = 0x00000010;        /*!<Bit 4 */
constexpr uint32_t  FMC_PMEM4_MEMSET4_5                = 0x00000020;        /*!<Bit 5 */
constexpr uint32_t  FMC_PMEM4_MEMSET4_6                = 0x00000040;        /*!<Bit 6 */
constexpr uint32_t  FMC_PMEM4_MEMSET4_7                = 0x00000080;        /*!<Bit 7 */

constexpr uint32_t  FMC_PMEM4_MEMWAIT4                 = 0x0000FF00;        /*!<MEMWAIT4[7:0] bits = Common memory 4 wait time; */
constexpr uint32_t  FMC_PMEM4_MEMWAIT4_0               = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  FMC_PMEM4_MEMWAIT4_1               = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  FMC_PMEM4_MEMWAIT4_2               = 0x00000400;        /*!<Bit 2 */
constexpr uint32_t  FMC_PMEM4_MEMWAIT4_3               = 0x00000800;        /*!<Bit 3 */
constexpr uint32_t  FMC_PMEM4_MEMWAIT4_4               = 0x00001000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PMEM4_MEMWAIT4_5               = 0x00002000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PMEM4_MEMWAIT4_6               = 0x00004000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PMEM4_MEMWAIT4_7               = 0x00008000;        /*!<Bit 7 */

constexpr uint32_t  FMC_PMEM4_MEMHOLD4                 = 0x00FF0000;        /*!<MEMHOLD4[7:0] bits = Common memory 4 hold time; */
constexpr uint32_t  FMC_PMEM4_MEMHOLD4_0               = 0x00010000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PMEM4_MEMHOLD4_1               = 0x00020000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PMEM4_MEMHOLD4_2               = 0x00040000;        /*!<Bit 2 */
constexpr uint32_t  FMC_PMEM4_MEMHOLD4_3               = 0x00080000;        /*!<Bit 3 */
constexpr uint32_t  FMC_PMEM4_MEMHOLD4_4               = 0x00100000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PMEM4_MEMHOLD4_5               = 0x00200000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PMEM4_MEMHOLD4_6               = 0x00400000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PMEM4_MEMHOLD4_7               = 0x00800000;        /*!<Bit 7 */

constexpr uint32_t  FMC_PMEM4_MEMHIZ4                  = 0xFF000000;        /*!<MEMHIZ4[7:0] bits = Common memory 4 databus HiZ time; */
constexpr uint32_t  FMC_PMEM4_MEMHIZ4_0                = 0x01000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PMEM4_MEMHIZ4_1                = 0x02000000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PMEM4_MEMHIZ4_2                = 0x04000000;        /*!<Bit 2 */
constexpr uint32_t  FMC_PMEM4_MEMHIZ4_3                = 0x08000000;        /*!<Bit 3 */
constexpr uint32_t  FMC_PMEM4_MEMHIZ4_4                = 0x10000000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PMEM4_MEMHIZ4_5                = 0x20000000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PMEM4_MEMHIZ4_6                = 0x40000000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PMEM4_MEMHIZ4_7                = 0x80000000;        /*!<Bit 7 */

/******************  Bit definition for FMC_PATT2 register  ******************/
constexpr uint32_t  FMC_PATT2_ATTSET2                  = 0x000000FF;        /*!<ATTSET2[7:0] bits = Attribute memory 2 setup time; */
constexpr uint32_t  FMC_PATT2_ATTSET2_0                = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_PATT2_ATTSET2_1                = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_PATT2_ATTSET2_2                = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  FMC_PATT2_ATTSET2_3                = 0x00000008;        /*!<Bit 3 */
constexpr uint32_t  FMC_PATT2_ATTSET2_4                = 0x00000010;        /*!<Bit 4 */
constexpr uint32_t  FMC_PATT2_ATTSET2_5                = 0x00000020;        /*!<Bit 5 */
constexpr uint32_t  FMC_PATT2_ATTSET2_6                = 0x00000040;        /*!<Bit 6 */
constexpr uint32_t  FMC_PATT2_ATTSET2_7                = 0x00000080;        /*!<Bit 7 */

constexpr uint32_t  FMC_PATT2_ATTWAIT2                 = 0x0000FF00;        /*!<ATTWAIT2[7:0] bits = Attribute memory 2 wait time; */
constexpr uint32_t  FMC_PATT2_ATTWAIT2_0               = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  FMC_PATT2_ATTWAIT2_1               = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  FMC_PATT2_ATTWAIT2_2               = 0x00000400;        /*!<Bit 2 */
constexpr uint32_t  FMC_PATT2_ATTWAIT2_3               = 0x00000800;        /*!<Bit 3 */
constexpr uint32_t  FMC_PATT2_ATTWAIT2_4               = 0x00001000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PATT2_ATTWAIT2_5               = 0x00002000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PATT2_ATTWAIT2_6               = 0x00004000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PATT2_ATTWAIT2_7               = 0x00008000;        /*!<Bit 7 */

constexpr uint32_t  FMC_PATT2_ATTHOLD2                 = 0x00FF0000;        /*!<ATTHOLD2[7:0] bits = Attribute memory 2 hold time; */
constexpr uint32_t  FMC_PATT2_ATTHOLD2_0               = 0x00010000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PATT2_ATTHOLD2_1               = 0x00020000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PATT2_ATTHOLD2_2               = 0x00040000;        /*!<Bit 2 */
constexpr uint32_t  FMC_PATT2_ATTHOLD2_3               = 0x00080000;        /*!<Bit 3 */
constexpr uint32_t  FMC_PATT2_ATTHOLD2_4               = 0x00100000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PATT2_ATTHOLD2_5               = 0x00200000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PATT2_ATTHOLD2_6               = 0x00400000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PATT2_ATTHOLD2_7               = 0x00800000;        /*!<Bit 7 */

constexpr uint32_t  FMC_PATT2_ATTHIZ2                  = 0xFF000000;        /*!<ATTHIZ2[7:0] bits = Attribute memory 2 databus HiZ time; */
constexpr uint32_t  FMC_PATT2_ATTHIZ2_0                = 0x01000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PATT2_ATTHIZ2_1                = 0x02000000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PATT2_ATTHIZ2_2                = 0x04000000;        /*!<Bit 2 */
constexpr uint32_t  FMC_PATT2_ATTHIZ2_3                = 0x08000000;        /*!<Bit 3 */
constexpr uint32_t  FMC_PATT2_ATTHIZ2_4                = 0x10000000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PATT2_ATTHIZ2_5                = 0x20000000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PATT2_ATTHIZ2_6                = 0x40000000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PATT2_ATTHIZ2_7                = 0x80000000;        /*!<Bit 7 */

/******************  Bit definition for FMC_PATT3 register  ******************/
constexpr uint32_t  FMC_PATT3_ATTSET3                  = 0x000000FF;        /*!<ATTSET3[7:0] bits = Attribute memory 3 setup time; */
constexpr uint32_t  FMC_PATT3_ATTSET3_0                = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_PATT3_ATTSET3_1                = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_PATT3_ATTSET3_2                = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  FMC_PATT3_ATTSET3_3                = 0x00000008;        /*!<Bit 3 */
constexpr uint32_t  FMC_PATT3_ATTSET3_4                = 0x00000010;        /*!<Bit 4 */
constexpr uint32_t  FMC_PATT3_ATTSET3_5                = 0x00000020;        /*!<Bit 5 */
constexpr uint32_t  FMC_PATT3_ATTSET3_6                = 0x00000040;        /*!<Bit 6 */
constexpr uint32_t  FMC_PATT3_ATTSET3_7                = 0x00000080;        /*!<Bit 7 */

constexpr uint32_t  FMC_PATT3_ATTWAIT3                 = 0x0000FF00;        /*!<ATTWAIT3[7:0] bits = Attribute memory 3 wait time; */
constexpr uint32_t  FMC_PATT3_ATTWAIT3_0               = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  FMC_PATT3_ATTWAIT3_1               = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  FMC_PATT3_ATTWAIT3_2               = 0x00000400;        /*!<Bit 2 */
constexpr uint32_t  FMC_PATT3_ATTWAIT3_3               = 0x00000800;        /*!<Bit 3 */
constexpr uint32_t  FMC_PATT3_ATTWAIT3_4               = 0x00001000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PATT3_ATTWAIT3_5               = 0x00002000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PATT3_ATTWAIT3_6               = 0x00004000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PATT3_ATTWAIT3_7               = 0x00008000;        /*!<Bit 7 */

constexpr uint32_t  FMC_PATT3_ATTHOLD3                 = 0x00FF0000;        /*!<ATTHOLD3[7:0] bits = Attribute memory 3 hold time; */
constexpr uint32_t  FMC_PATT3_ATTHOLD3_0               = 0x00010000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PATT3_ATTHOLD3_1               = 0x00020000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PATT3_ATTHOLD3_2               = 0x00040000;        /*!<Bit 2 */
constexpr uint32_t  FMC_PATT3_ATTHOLD3_3               = 0x00080000;        /*!<Bit 3 */
constexpr uint32_t  FMC_PATT3_ATTHOLD3_4               = 0x00100000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PATT3_ATTHOLD3_5               = 0x00200000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PATT3_ATTHOLD3_6               = 0x00400000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PATT3_ATTHOLD3_7               = 0x00800000;        /*!<Bit 7 */

constexpr uint32_t  FMC_PATT3_ATTHIZ3                  = 0xFF000000;        /*!<ATTHIZ3[7:0] bits = Attribute memory 3 databus HiZ time; */
constexpr uint32_t  FMC_PATT3_ATTHIZ3_0                = 0x01000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PATT3_ATTHIZ3_1                = 0x02000000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PATT3_ATTHIZ3_2                = 0x04000000;        /*!<Bit 2 */
constexpr uint32_t  FMC_PATT3_ATTHIZ3_3                = 0x08000000;        /*!<Bit 3 */
constexpr uint32_t  FMC_PATT3_ATTHIZ3_4                = 0x10000000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PATT3_ATTHIZ3_5                = 0x20000000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PATT3_ATTHIZ3_6                = 0x40000000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PATT3_ATTHIZ3_7                = 0x80000000;        /*!<Bit 7 */

/******************  Bit definition for FMC_PATT4 register  ******************/
constexpr uint32_t  FMC_PATT4_ATTSET4                  = 0x000000FF;        /*!<ATTSET4[7:0] bits = Attribute memory 4 setup time; */
constexpr uint32_t  FMC_PATT4_ATTSET4_0                = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_PATT4_ATTSET4_1                = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_PATT4_ATTSET4_2                = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  FMC_PATT4_ATTSET4_3                = 0x00000008;        /*!<Bit 3 */
constexpr uint32_t  FMC_PATT4_ATTSET4_4                = 0x00000010;        /*!<Bit 4 */
constexpr uint32_t  FMC_PATT4_ATTSET4_5                = 0x00000020;        /*!<Bit 5 */
constexpr uint32_t  FMC_PATT4_ATTSET4_6                = 0x00000040;        /*!<Bit 6 */
constexpr uint32_t  FMC_PATT4_ATTSET4_7                = 0x00000080;        /*!<Bit 7 */

constexpr uint32_t  FMC_PATT4_ATTWAIT4                 = 0x0000FF00;        /*!<ATTWAIT4[7:0] bits = Attribute memory 4 wait time; */
constexpr uint32_t  FMC_PATT4_ATTWAIT4_0               = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  FMC_PATT4_ATTWAIT4_1               = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  FMC_PATT4_ATTWAIT4_2               = 0x00000400;        /*!<Bit 2 */
constexpr uint32_t  FMC_PATT4_ATTWAIT4_3               = 0x00000800;        /*!<Bit 3 */
constexpr uint32_t  FMC_PATT4_ATTWAIT4_4               = 0x00001000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PATT4_ATTWAIT4_5               = 0x00002000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PATT4_ATTWAIT4_6               = 0x00004000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PATT4_ATTWAIT4_7               = 0x00008000;        /*!<Bit 7 */

constexpr uint32_t  FMC_PATT4_ATTHOLD4                 = 0x00FF0000;        /*!<ATTHOLD4[7:0] bits = Attribute memory 4 hold time; */
constexpr uint32_t  FMC_PATT4_ATTHOLD4_0               = 0x00010000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PATT4_ATTHOLD4_1               = 0x00020000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PATT4_ATTHOLD4_2               = 0x00040000;        /*!<Bit 2 */
constexpr uint32_t  FMC_PATT4_ATTHOLD4_3               = 0x00080000;        /*!<Bit 3 */
constexpr uint32_t  FMC_PATT4_ATTHOLD4_4               = 0x00100000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PATT4_ATTHOLD4_5               = 0x00200000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PATT4_ATTHOLD4_6               = 0x00400000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PATT4_ATTHOLD4_7               = 0x00800000;        /*!<Bit 7 */

constexpr uint32_t  FMC_PATT4_ATTHIZ4                  = 0xFF000000;        /*!<ATTHIZ4[7:0] bits = Attribute memory 4 databus HiZ time; */
constexpr uint32_t  FMC_PATT4_ATTHIZ4_0                = 0x01000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PATT4_ATTHIZ4_1                = 0x02000000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PATT4_ATTHIZ4_2                = 0x04000000;        /*!<Bit 2 */
constexpr uint32_t  FMC_PATT4_ATTHIZ4_3                = 0x08000000;        /*!<Bit 3 */
constexpr uint32_t  FMC_PATT4_ATTHIZ4_4                = 0x10000000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PATT4_ATTHIZ4_5                = 0x20000000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PATT4_ATTHIZ4_6                = 0x40000000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PATT4_ATTHIZ4_7                = 0x80000000;        /*!<Bit 7 */

/******************  Bit definition for FMC_PIO4 register  *******************/
constexpr uint32_t  FMC_PIO4_IOSET4                    = 0x000000FF;        /*!<IOSET4[7:0] bits = I/O 4 setup time; */
constexpr uint32_t  FMC_PIO4_IOSET4_0                  = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_PIO4_IOSET4_1                  = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_PIO4_IOSET4_2                  = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  FMC_PIO4_IOSET4_3                  = 0x00000008;        /*!<Bit 3 */
constexpr uint32_t  FMC_PIO4_IOSET4_4                  = 0x00000010;        /*!<Bit 4 */
constexpr uint32_t  FMC_PIO4_IOSET4_5                  = 0x00000020;        /*!<Bit 5 */
constexpr uint32_t  FMC_PIO4_IOSET4_6                  = 0x00000040;        /*!<Bit 6 */
constexpr uint32_t  FMC_PIO4_IOSET4_7                  = 0x00000080;        /*!<Bit 7 */

constexpr uint32_t  FMC_PIO4_IOWAIT4                   = 0x0000FF00;        /*!<IOWAIT4[7:0] bits = I/O 4 wait time; */
constexpr uint32_t  FMC_PIO4_IOWAIT4_0                 = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  FMC_PIO4_IOWAIT4_1                 = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  FMC_PIO4_IOWAIT4_2                 = 0x00000400;        /*!<Bit 2 */
constexpr uint32_t  FMC_PIO4_IOWAIT4_3                 = 0x00000800;        /*!<Bit 3 */
constexpr uint32_t  FMC_PIO4_IOWAIT4_4                 = 0x00001000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PIO4_IOWAIT4_5                 = 0x00002000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PIO4_IOWAIT4_6                 = 0x00004000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PIO4_IOWAIT4_7                 = 0x00008000;        /*!<Bit 7 */

constexpr uint32_t  FMC_PIO4_IOHOLD4                   = 0x00FF0000;        /*!<IOHOLD4[7:0] bits = I/O 4 hold time; */
constexpr uint32_t  FMC_PIO4_IOHOLD4_0                 = 0x00010000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PIO4_IOHOLD4_1                 = 0x00020000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PIO4_IOHOLD4_2                 = 0x00040000;        /*!<Bit 2 */
constexpr uint32_t  FMC_PIO4_IOHOLD4_3                 = 0x00080000;        /*!<Bit 3 */
constexpr uint32_t  FMC_PIO4_IOHOLD4_4                 = 0x00100000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PIO4_IOHOLD4_5                 = 0x00200000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PIO4_IOHOLD4_6                 = 0x00400000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PIO4_IOHOLD4_7                 = 0x00800000;        /*!<Bit 7 */

constexpr uint32_t  FMC_PIO4_IOHIZ4                    = 0xFF000000;        /*!<IOHIZ4[7:0] bits = I/O 4 databus HiZ time; */
constexpr uint32_t  FMC_PIO4_IOHIZ4_0                  = 0x01000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_PIO4_IOHIZ4_1                  = 0x02000000;        /*!<Bit 1 */
constexpr uint32_t  FMC_PIO4_IOHIZ4_2                  = 0x04000000;        /*!<Bit 2 */
constexpr uint32_t  FMC_PIO4_IOHIZ4_3                  = 0x08000000;        /*!<Bit 3 */
constexpr uint32_t  FMC_PIO4_IOHIZ4_4                  = 0x10000000;        /*!<Bit 4 */
constexpr uint32_t  FMC_PIO4_IOHIZ4_5                  = 0x20000000;        /*!<Bit 5 */
constexpr uint32_t  FMC_PIO4_IOHIZ4_6                  = 0x40000000;        /*!<Bit 6 */
constexpr uint32_t  FMC_PIO4_IOHIZ4_7                  = 0x80000000;        /*!<Bit 7 */

/******************  Bit definition for FMC_ECCR2 register  ******************/
constexpr uint32_t  FMC_ECCR2_ECC2                     = 0xFFFFFFFF;        /*!<ECC result */

/******************  Bit definition for FMC_ECCR3 register  ******************/
constexpr uint32_t  FMC_ECCR3_ECC3                     = 0xFFFFFFFF;        /*!<ECC result */

/******************  Bit definition for FMC_SDCR1 register  ******************/
constexpr uint32_t  FMC_SDCR1_NC                       = 0x00000003;        /*!<NC[1:0] bits = Number of column bits; */
constexpr uint32_t  FMC_SDCR1_NC_0                     = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDCR1_NC_1                     = 0x00000002;        /*!<Bit 1 */

constexpr uint32_t  FMC_SDCR1_NR                       = 0x0000000C;        /*!<NR[1:0] bits = Number of row bits; */
constexpr uint32_t  FMC_SDCR1_NR_0                     = 0x00000004;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDCR1_NR_1                     = 0x00000008;        /*!<Bit 1 */

constexpr uint32_t  FMC_SDCR1_MWID                     = 0x00000030;        /*!<NR[1:0] bits = Number of row bits; */
constexpr uint32_t  FMC_SDCR1_MWID_0                   = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDCR1_MWID_1                   = 0x00000020;        /*!<Bit 1 */

constexpr uint32_t  FMC_SDCR1_NB                       = 0x00000040;        /*!<Number of internal bank */

constexpr uint32_t  FMC_SDCR1_CAS                      = 0x00000180;        /*!<CAS[1:0] bits = CAS latency; */
constexpr uint32_t  FMC_SDCR1_CAS_0                    = 0x00000080;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDCR1_CAS_1                    = 0x00000100;        /*!<Bit 1 */

constexpr uint32_t  FMC_SDCR1_WP                       = 0x00000200;        /*!<Write protection */

constexpr uint32_t  FMC_SDCR1_SDCLK                    = 0x00000C00;        /*!<SDRAM clock configuration */
constexpr uint32_t  FMC_SDCR1_SDCLK_0                  = 0x00000400;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDCR1_SDCLK_1                  = 0x00000800;        /*!<Bit 1 */

constexpr uint32_t  FMC_SDCR1_RBURST                   = 0x00001000;        /*!<Read burst */

constexpr uint32_t  FMC_SDCR1_RPIPE                    = 0x00006000;        /*!<Write protection */
constexpr uint32_t  FMC_SDCR1_RPIPE_0                  = 0x00002000;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDCR1_RPIPE_1                  = 0x00004000;        /*!<Bit 1 */

/******************  Bit definition for FMC_SDCR2 register  ******************/
constexpr uint32_t  FMC_SDCR2_NC                       = 0x00000003;        /*!<NC[1:0] bits = Number of column bits; */
constexpr uint32_t  FMC_SDCR2_NC_0                     = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDCR2_NC_1                     = 0x00000002;        /*!<Bit 1 */

constexpr uint32_t  FMC_SDCR2_NR                       = 0x0000000C;        /*!<NR[1:0] bits = Number of row bits; */
constexpr uint32_t  FMC_SDCR2_NR_0                     = 0x00000004;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDCR2_NR_1                     = 0x00000008;        /*!<Bit 1 */

constexpr uint32_t  FMC_SDCR2_MWID                     = 0x00000030;        /*!<NR[1:0] bits = Number of row bits; */
constexpr uint32_t  FMC_SDCR2_MWID_0                   = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDCR2_MWID_1                   = 0x00000020;        /*!<Bit 1 */

constexpr uint32_t  FMC_SDCR2_NB                       = 0x00000040;        /*!<Number of internal bank */

constexpr uint32_t  FMC_SDCR2_CAS                      = 0x00000180;        /*!<CAS[1:0] bits = CAS latency; */
constexpr uint32_t  FMC_SDCR2_CAS_0                    = 0x00000080;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDCR2_CAS_1                    = 0x00000100;        /*!<Bit 1 */

constexpr uint32_t  FMC_SDCR2_WP                       = 0x00000200;        /*!<Write protection */

constexpr uint32_t  FMC_SDCR2_SDCLK                    = 0x00000C00;        /*!<SDCLK[1:0] = SDRAM clock configuration; */
constexpr uint32_t  FMC_SDCR2_SDCLK_0                  = 0x00000400;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDCR2_SDCLK_1                  = 0x00000800;        /*!<Bit 1 */

constexpr uint32_t  FMC_SDCR2_RBURST                   = 0x00001000;        /*!<Read burst */

constexpr uint32_t  FMC_SDCR2_RPIPE                    = 0x00006000;        /*!<RPIPE[1:0]= Read pipe; */
constexpr uint32_t  FMC_SDCR2_RPIPE_0                  = 0x00002000;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDCR2_RPIPE_1                  = 0x00004000;        /*!<Bit 1 */

/******************  Bit definition for FMC_SDTR1 register  ******************/
constexpr uint32_t  FMC_SDTR1_TMRD                     = 0x0000000F;        /*!<TMRD[3:0] bits = Load mode register to active; */
constexpr uint32_t  FMC_SDTR1_TMRD_0                   = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDTR1_TMRD_1                   = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_SDTR1_TMRD_2                   = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  FMC_SDTR1_TMRD_3                   = 0x00000008;        /*!<Bit 3 */

constexpr uint32_t  FMC_SDTR1_TXSR                     = 0x000000F0;        /*!<TXSR[3:0] bits = Exit self refresh; */
constexpr uint32_t  FMC_SDTR1_TXSR_0                   = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDTR1_TXSR_1                   = 0x00000020;        /*!<Bit 1 */
constexpr uint32_t  FMC_SDTR1_TXSR_2                   = 0x00000040;        /*!<Bit 2 */
constexpr uint32_t  FMC_SDTR1_TXSR_3                   = 0x00000080;        /*!<Bit 3 */

constexpr uint32_t  FMC_SDTR1_TRAS                     = 0x00000F00;        /*!<TRAS[3:0] bits = Self refresh time; */
constexpr uint32_t  FMC_SDTR1_TRAS_0                   = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDTR1_TRAS_1                   = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  FMC_SDTR1_TRAS_2                   = 0x00000400;        /*!<Bit 2 */
constexpr uint32_t  FMC_SDTR1_TRAS_3                   = 0x00000800;        /*!<Bit 3 */

constexpr uint32_t  FMC_SDTR1_TRC                      = 0x0000F000;        /*!<TRC[2:0] bits = Row cycle delay; */
constexpr uint32_t  FMC_SDTR1_TRC_0                    = 0x00001000;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDTR1_TRC_1                    = 0x00002000;        /*!<Bit 1 */
constexpr uint32_t  FMC_SDTR1_TRC_2                    = 0x00004000;        /*!<Bit 2 */

constexpr uint32_t  FMC_SDTR1_TWR                      = 0x000F0000;        /*!<TRC[2:0] bits = Write recovery delay; */
constexpr uint32_t  FMC_SDTR1_TWR_0                    = 0x00010000;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDTR1_TWR_1                    = 0x00020000;        /*!<Bit 1 */
constexpr uint32_t  FMC_SDTR1_TWR_2                    = 0x00040000;        /*!<Bit 2 */

constexpr uint32_t  FMC_SDTR1_TRP                      = 0x00F00000;        /*!<TRP[2:0] bits = Row precharge delay; */
constexpr uint32_t  FMC_SDTR1_TRP_0                    = 0x00100000;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDTR1_TRP_1                    = 0x00200000;        /*!<Bit 1 */
constexpr uint32_t  FMC_SDTR1_TRP_2                    = 0x00400000;        /*!<Bit 2 */

constexpr uint32_t  FMC_SDTR1_TRCD                     = 0x0F000000;        /*!<TRP[2:0] bits = Row to column delay; */
constexpr uint32_t  FMC_SDTR1_TRCD_0                   = 0x01000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDTR1_TRCD_1                   = 0x02000000;        /*!<Bit 1 */
constexpr uint32_t  FMC_SDTR1_TRCD_2                   = 0x04000000;        /*!<Bit 2 */

/******************  Bit definition for FMC_SDTR2 register  ******************/
constexpr uint32_t  FMC_SDTR2_TMRD                     = 0x0000000F;        /*!<TMRD[3:0] bits = Load mode register to active; */
constexpr uint32_t  FMC_SDTR2_TMRD_0                   = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDTR2_TMRD_1                   = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_SDTR2_TMRD_2                   = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  FMC_SDTR2_TMRD_3                   = 0x00000008;        /*!<Bit 3 */

constexpr uint32_t  FMC_SDTR2_TXSR                     = 0x000000F0;        /*!<TXSR[3:0] bits = Exit self refresh; */
constexpr uint32_t  FMC_SDTR2_TXSR_0                   = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDTR2_TXSR_1                   = 0x00000020;        /*!<Bit 1 */
constexpr uint32_t  FMC_SDTR2_TXSR_2                   = 0x00000040;        /*!<Bit 2 */
constexpr uint32_t  FMC_SDTR2_TXSR_3                   = 0x00000080;        /*!<Bit 3 */

constexpr uint32_t  FMC_SDTR2_TRAS                     = 0x00000F00;        /*!<TRAS[3:0] bits = Self refresh time; */
constexpr uint32_t  FMC_SDTR2_TRAS_0                   = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDTR2_TRAS_1                   = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  FMC_SDTR2_TRAS_2                   = 0x00000400;        /*!<Bit 2 */
constexpr uint32_t  FMC_SDTR2_TRAS_3                   = 0x00000800;        /*!<Bit 3 */

constexpr uint32_t  FMC_SDTR2_TRC                      = 0x0000F000;        /*!<TRC[2:0] bits = Row cycle delay; */
constexpr uint32_t  FMC_SDTR2_TRC_0                    = 0x00001000;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDTR2_TRC_1                    = 0x00002000;        /*!<Bit 1 */
constexpr uint32_t  FMC_SDTR2_TRC_2                    = 0x00004000;        /*!<Bit 2 */

constexpr uint32_t  FMC_SDTR2_TWR                      = 0x000F0000;        /*!<TRC[2:0] bits = Write recovery delay; */
constexpr uint32_t  FMC_SDTR2_TWR_0                    = 0x00010000;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDTR2_TWR_1                    = 0x00020000;        /*!<Bit 1 */
constexpr uint32_t  FMC_SDTR2_TWR_2                    = 0x00040000;        /*!<Bit 2 */

constexpr uint32_t  FMC_SDTR2_TRP                      = 0x00F00000;        /*!<TRP[2:0] bits = Row precharge delay; */
constexpr uint32_t  FMC_SDTR2_TRP_0                    = 0x00100000;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDTR2_TRP_1                    = 0x00200000;        /*!<Bit 1 */
constexpr uint32_t  FMC_SDTR2_TRP_2                    = 0x00400000;        /*!<Bit 2 */

constexpr uint32_t  FMC_SDTR2_TRCD                     = 0x0F000000;        /*!<TRP[2:0] bits = Row to column delay; */
constexpr uint32_t  FMC_SDTR2_TRCD_0                   = 0x01000000;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDTR2_TRCD_1                   = 0x02000000;        /*!<Bit 1 */
constexpr uint32_t  FMC_SDTR2_TRCD_2                   = 0x04000000;        /*!<Bit 2 */

/******************  Bit definition for FMC_SDCMR register  ******************/
constexpr uint32_t  FMC_SDCMR_MODE                     = 0x00000007;        /*!<MODE[2:0] bits = Command mode; */
constexpr uint32_t  FMC_SDCMR_MODE_0                   = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDCMR_MODE_1                   = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  FMC_SDCMR_MODE_2                   = 0x00000003;        /*!<Bit 2 */

constexpr uint32_t  FMC_SDCMR_CTB2                     = 0x00000008;        /*!<Command target 2 */

constexpr uint32_t  FMC_SDCMR_CTB1                     = 0x00000010;        /*!<Command target 1 */

constexpr uint32_t  FMC_SDCMR_NRFS                     = 0x000001E0;        /*!<NRFS[3:0] bits = Number of auto-refresh; */
constexpr uint32_t  FMC_SDCMR_NRFS_0                   = 0x00000020;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDCMR_NRFS_1                   = 0x00000040;        /*!<Bit 1 */
constexpr uint32_t  FMC_SDCMR_NRFS_2                   = 0x00000080;        /*!<Bit 2 */
constexpr uint32_t  FMC_SDCMR_NRFS_3                   = 0x00000100;        /*!<Bit 3 */

constexpr uint32_t  FMC_SDCMR_MRD                      = 0x003FFE00;        /*!<MRD[12:0] bits = Mode register definition; */

/******************  Bit definition for FMC_SDRTR register  ******************/
constexpr uint32_t  FMC_SDRTR_CRE                      = 0x00000001;        /*!<Clear refresh error flag */

constexpr uint32_t  FMC_SDRTR_COUNT                    = 0x00003FFE;        /*!<COUNT[12:0] bits = Refresh timer count; */

constexpr uint32_t  FMC_SDRTR_REIE                     = 0x00004000;        /*!<RES interupt enable */

/******************  Bit definition for FMC_SDSR register  ******************/
constexpr uint32_t  FMC_SDSR_RE                        = 0x00000001;        /*!<Refresh error flag */

constexpr uint32_t  FMC_SDSR_MODES1                    = 0x00000006;        /*!<MODES1[1:0]bits = Status mode for bank 1; */
constexpr uint32_t  FMC_SDSR_MODES1_0                  = 0x00000002;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDSR_MODES1_1                  = 0x00000004;        /*!<Bit 1 */

constexpr uint32_t  FMC_SDSR_MODES2                    = 0x00000018;        /*!<MODES2[1:0]bits = Status mode for bank 2; */
constexpr uint32_t  FMC_SDSR_MODES2_0                  = 0x00000008;        /*!<Bit 0 */
constexpr uint32_t  FMC_SDSR_MODES2_1                  = 0x00000010;        /*!<Bit 1 */
constexpr uint32_t  FMC_SDSR_BUSY                      = 0x00000020;        /*!<Busy status */



/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O = GPIO;                      */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for GPIO_MODER register  *****************/
constexpr uint32_t GPIO_MODER_MODER0          = 0x00000003;
constexpr uint32_t GPIO_MODER_MODER0_0        = 0x00000001;
constexpr uint32_t GPIO_MODER_MODER0_1        = 0x00000002;
constexpr uint32_t GPIO_MODER_MODER1          = 0x0000000C;
constexpr uint32_t GPIO_MODER_MODER1_0        = 0x00000004;
constexpr uint32_t GPIO_MODER_MODER1_1        = 0x00000008;
constexpr uint32_t GPIO_MODER_MODER2          = 0x00000030;
constexpr uint32_t GPIO_MODER_MODER2_0        = 0x00000010;
constexpr uint32_t GPIO_MODER_MODER2_1        = 0x00000020;
constexpr uint32_t GPIO_MODER_MODER3          = 0x000000C0;
constexpr uint32_t GPIO_MODER_MODER3_0        = 0x00000040;
constexpr uint32_t GPIO_MODER_MODER3_1        = 0x00000080;
constexpr uint32_t GPIO_MODER_MODER4          = 0x00000300;
constexpr uint32_t GPIO_MODER_MODER4_0        = 0x00000100;
constexpr uint32_t GPIO_MODER_MODER4_1        = 0x00000200;
constexpr uint32_t GPIO_MODER_MODER5          = 0x00000C00;
constexpr uint32_t GPIO_MODER_MODER5_0        = 0x00000400;
constexpr uint32_t GPIO_MODER_MODER5_1        = 0x00000800;
constexpr uint32_t GPIO_MODER_MODER6          = 0x00003000;
constexpr uint32_t GPIO_MODER_MODER6_0        = 0x00001000;
constexpr uint32_t GPIO_MODER_MODER6_1        = 0x00002000;
constexpr uint32_t GPIO_MODER_MODER7          = 0x0000C000;
constexpr uint32_t GPIO_MODER_MODER7_0        = 0x00004000;
constexpr uint32_t GPIO_MODER_MODER7_1        = 0x00008000;
constexpr uint32_t GPIO_MODER_MODER8          = 0x00030000;
constexpr uint32_t GPIO_MODER_MODER8_0        = 0x00010000;
constexpr uint32_t GPIO_MODER_MODER8_1        = 0x00020000;
constexpr uint32_t GPIO_MODER_MODER9          = 0x000C0000;
constexpr uint32_t GPIO_MODER_MODER9_0        = 0x00040000;
constexpr uint32_t GPIO_MODER_MODER9_1        = 0x00080000;
constexpr uint32_t GPIO_MODER_MODER10         = 0x00300000;
constexpr uint32_t GPIO_MODER_MODER10_0       = 0x00100000;
constexpr uint32_t GPIO_MODER_MODER10_1       = 0x00200000;
constexpr uint32_t GPIO_MODER_MODER11         = 0x00C00000;
constexpr uint32_t GPIO_MODER_MODER11_0       = 0x00400000;
constexpr uint32_t GPIO_MODER_MODER11_1       = 0x00800000;
constexpr uint32_t GPIO_MODER_MODER12         = 0x03000000;
constexpr uint32_t GPIO_MODER_MODER12_0       = 0x01000000;
constexpr uint32_t GPIO_MODER_MODER12_1       = 0x02000000;
constexpr uint32_t GPIO_MODER_MODER13         = 0x0C000000;
constexpr uint32_t GPIO_MODER_MODER13_0       = 0x04000000;
constexpr uint32_t GPIO_MODER_MODER13_1       = 0x08000000;
constexpr uint32_t GPIO_MODER_MODER14         = 0x30000000;
constexpr uint32_t GPIO_MODER_MODER14_0       = 0x10000000;
constexpr uint32_t GPIO_MODER_MODER14_1       = 0x20000000;
constexpr uint32_t GPIO_MODER_MODER15         = 0xC0000000;
constexpr uint32_t GPIO_MODER_MODER15_0       = 0x40000000;
constexpr uint32_t GPIO_MODER_MODER15_1       = 0x80000000;

/******************  Bit definition for GPIO_OTYPER register  *****************/
constexpr uint32_t GPIO_OTYPER_OT_0           = 0x00000001;
constexpr uint32_t GPIO_OTYPER_OT_1           = 0x00000002;
constexpr uint32_t GPIO_OTYPER_OT_2           = 0x00000004;
constexpr uint32_t GPIO_OTYPER_OT_3           = 0x00000008;
constexpr uint32_t GPIO_OTYPER_OT_4           = 0x00000010;
constexpr uint32_t GPIO_OTYPER_OT_5           = 0x00000020;
constexpr uint32_t GPIO_OTYPER_OT_6           = 0x00000040;
constexpr uint32_t GPIO_OTYPER_OT_7           = 0x00000080;
constexpr uint32_t GPIO_OTYPER_OT_8           = 0x00000100;
constexpr uint32_t GPIO_OTYPER_OT_9           = 0x00000200;
constexpr uint32_t GPIO_OTYPER_OT_10          = 0x00000400;
constexpr uint32_t GPIO_OTYPER_OT_11          = 0x00000800;
constexpr uint32_t GPIO_OTYPER_OT_12          = 0x00001000;
constexpr uint32_t GPIO_OTYPER_OT_13          = 0x00002000;
constexpr uint32_t GPIO_OTYPER_OT_14          = 0x00004000;
constexpr uint32_t GPIO_OTYPER_OT_15          = 0x00008000;

/****************  Bit definition for GPIO_OSPEEDR register  ******************/
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR0     = 0x00000003;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR0_0   = 0x00000001;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR0_1   = 0x00000002;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR1     = 0x0000000C;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR1_0   = 0x00000004;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR1_1   = 0x00000008;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR2     = 0x00000030;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR2_0   = 0x00000010;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR2_1   = 0x00000020;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR3     = 0x000000C0;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR3_0   = 0x00000040;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR3_1   = 0x00000080;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR4     = 0x00000300;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR4_0   = 0x00000100;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR4_1   = 0x00000200;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR5     = 0x00000C00;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR5_0   = 0x00000400;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR5_1   = 0x00000800;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR6     = 0x00003000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR6_0   = 0x00001000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR6_1   = 0x00002000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR7     = 0x0000C000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR7_0   = 0x00004000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR7_1   = 0x00008000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR8     = 0x00030000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR8_0   = 0x00010000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR8_1   = 0x00020000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR9     = 0x000C0000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR9_0   = 0x00040000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR9_1   = 0x00080000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR10    = 0x00300000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR10_0  = 0x00100000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR10_1  = 0x00200000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR11    = 0x00C00000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR11_0  = 0x00400000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR11_1  = 0x00800000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR12    = 0x03000000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR12_0  = 0x01000000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR12_1  = 0x02000000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR13    = 0x0C000000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR13_0  = 0x04000000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR13_1  = 0x08000000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR14    = 0x30000000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR14_0  = 0x10000000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR14_1  = 0x20000000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR15    = 0xC0000000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR15_0  = 0x40000000;
constexpr uint32_t GPIO_OSPEEDER_OSPEEDR15_1  = 0x80000000;

/*******************  Bit definition for GPIO_PUPDR register ******************/
constexpr uint32_t GPIO_PUPDR_PUPDR0          = 0x00000003;
constexpr uint32_t GPIO_PUPDR_PUPDR0_0        = 0x00000001;
constexpr uint32_t GPIO_PUPDR_PUPDR0_1        = 0x00000002;
constexpr uint32_t GPIO_PUPDR_PUPDR1          = 0x0000000C;
constexpr uint32_t GPIO_PUPDR_PUPDR1_0        = 0x00000004;
constexpr uint32_t GPIO_PUPDR_PUPDR1_1        = 0x00000008;
constexpr uint32_t GPIO_PUPDR_PUPDR2          = 0x00000030;
constexpr uint32_t GPIO_PUPDR_PUPDR2_0        = 0x00000010;
constexpr uint32_t GPIO_PUPDR_PUPDR2_1        = 0x00000020;
constexpr uint32_t GPIO_PUPDR_PUPDR3          = 0x000000C0;
constexpr uint32_t GPIO_PUPDR_PUPDR3_0        = 0x00000040;
constexpr uint32_t GPIO_PUPDR_PUPDR3_1        = 0x00000080;
constexpr uint32_t GPIO_PUPDR_PUPDR4          = 0x00000300;
constexpr uint32_t GPIO_PUPDR_PUPDR4_0        = 0x00000100;
constexpr uint32_t GPIO_PUPDR_PUPDR4_1        = 0x00000200;
constexpr uint32_t GPIO_PUPDR_PUPDR5          = 0x00000C00;
constexpr uint32_t GPIO_PUPDR_PUPDR5_0        = 0x00000400;
constexpr uint32_t GPIO_PUPDR_PUPDR5_1        = 0x00000800;
constexpr uint32_t GPIO_PUPDR_PUPDR6          = 0x00003000;
constexpr uint32_t GPIO_PUPDR_PUPDR6_0        = 0x00001000;
constexpr uint32_t GPIO_PUPDR_PUPDR6_1        = 0x00002000;
constexpr uint32_t GPIO_PUPDR_PUPDR7          = 0x0000C000;
constexpr uint32_t GPIO_PUPDR_PUPDR7_0        = 0x00004000;
constexpr uint32_t GPIO_PUPDR_PUPDR7_1        = 0x00008000;
constexpr uint32_t GPIO_PUPDR_PUPDR8          = 0x00030000;
constexpr uint32_t GPIO_PUPDR_PUPDR8_0        = 0x00010000;
constexpr uint32_t GPIO_PUPDR_PUPDR8_1        = 0x00020000;
constexpr uint32_t GPIO_PUPDR_PUPDR9          = 0x000C0000;
constexpr uint32_t GPIO_PUPDR_PUPDR9_0        = 0x00040000;
constexpr uint32_t GPIO_PUPDR_PUPDR9_1        = 0x00080000;
constexpr uint32_t GPIO_PUPDR_PUPDR10         = 0x00300000;
constexpr uint32_t GPIO_PUPDR_PUPDR10_0       = 0x00100000;
constexpr uint32_t GPIO_PUPDR_PUPDR10_1       = 0x00200000;
constexpr uint32_t GPIO_PUPDR_PUPDR11         = 0x00C00000;
constexpr uint32_t GPIO_PUPDR_PUPDR11_0       = 0x00400000;
constexpr uint32_t GPIO_PUPDR_PUPDR11_1       = 0x00800000;
constexpr uint32_t GPIO_PUPDR_PUPDR12         = 0x03000000;
constexpr uint32_t GPIO_PUPDR_PUPDR12_0       = 0x01000000;
constexpr uint32_t GPIO_PUPDR_PUPDR12_1       = 0x02000000;
constexpr uint32_t GPIO_PUPDR_PUPDR13         = 0x0C000000;
constexpr uint32_t GPIO_PUPDR_PUPDR13_0       = 0x04000000;
constexpr uint32_t GPIO_PUPDR_PUPDR13_1       = 0x08000000;
constexpr uint32_t GPIO_PUPDR_PUPDR14         = 0x30000000;
constexpr uint32_t GPIO_PUPDR_PUPDR14_0       = 0x10000000;
constexpr uint32_t GPIO_PUPDR_PUPDR14_1       = 0x20000000;
constexpr uint32_t GPIO_PUPDR_PUPDR15         = 0xC0000000;
constexpr uint32_t GPIO_PUPDR_PUPDR15_0       = 0x40000000;
constexpr uint32_t GPIO_PUPDR_PUPDR15_1       = 0x80000000;

/*******************  Bit definition for GPIO_IDR register  *******************/
constexpr uint32_t GPIO_IDR_0                 = 0x00000001;
constexpr uint32_t GPIO_IDR_1                 = 0x00000002;
constexpr uint32_t GPIO_IDR_2                 = 0x00000004;
constexpr uint32_t GPIO_IDR_3                 = 0x00000008;
constexpr uint32_t GPIO_IDR_4                 = 0x00000010;
constexpr uint32_t GPIO_IDR_5                 = 0x00000020;
constexpr uint32_t GPIO_IDR_6                 = 0x00000040;
constexpr uint32_t GPIO_IDR_7                 = 0x00000080;
constexpr uint32_t GPIO_IDR_8                 = 0x00000100;
constexpr uint32_t GPIO_IDR_9                 = 0x00000200;
constexpr uint32_t GPIO_IDR_10                = 0x00000400;
constexpr uint32_t GPIO_IDR_11                = 0x00000800;
constexpr uint32_t GPIO_IDR_12                = 0x00001000;
constexpr uint32_t GPIO_IDR_13                = 0x00002000;
constexpr uint32_t GPIO_IDR_14                = 0x00004000;
constexpr uint32_t GPIO_IDR_15                = 0x00008000;

/******************  Bit definition for GPIO_ODR register  ********************/
constexpr uint32_t GPIO_ODR_0                 = 0x00000001;
constexpr uint32_t GPIO_ODR_1                 = 0x00000002;
constexpr uint32_t GPIO_ODR_2                 = 0x00000004;
constexpr uint32_t GPIO_ODR_3                 = 0x00000008;
constexpr uint32_t GPIO_ODR_4                 = 0x00000010;
constexpr uint32_t GPIO_ODR_5                 = 0x00000020;
constexpr uint32_t GPIO_ODR_6                 = 0x00000040;
constexpr uint32_t GPIO_ODR_7                 = 0x00000080;
constexpr uint32_t GPIO_ODR_8                 = 0x00000100;
constexpr uint32_t GPIO_ODR_9                 = 0x00000200;
constexpr uint32_t GPIO_ODR_10                = 0x00000400;
constexpr uint32_t GPIO_ODR_11                = 0x00000800;
constexpr uint32_t GPIO_ODR_12                = 0x00001000;
constexpr uint32_t GPIO_ODR_13                = 0x00002000;
constexpr uint32_t GPIO_ODR_14                = 0x00004000;
constexpr uint32_t GPIO_ODR_15                = 0x00008000;

/****************** Bit definition for GPIO_BSRR register  ********************/
constexpr uint32_t GPIO_BSRR_BS_0             = 0x00000001;
constexpr uint32_t GPIO_BSRR_BS_1             = 0x00000002;
constexpr uint32_t GPIO_BSRR_BS_2             = 0x00000004;
constexpr uint32_t GPIO_BSRR_BS_3             = 0x00000008;
constexpr uint32_t GPIO_BSRR_BS_4             = 0x00000010;
constexpr uint32_t GPIO_BSRR_BS_5             = 0x00000020;
constexpr uint32_t GPIO_BSRR_BS_6             = 0x00000040;
constexpr uint32_t GPIO_BSRR_BS_7             = 0x00000080;
constexpr uint32_t GPIO_BSRR_BS_8             = 0x00000100;
constexpr uint32_t GPIO_BSRR_BS_9             = 0x00000200;
constexpr uint32_t GPIO_BSRR_BS_10            = 0x00000400;
constexpr uint32_t GPIO_BSRR_BS_11            = 0x00000800;
constexpr uint32_t GPIO_BSRR_BS_12            = 0x00001000;
constexpr uint32_t GPIO_BSRR_BS_13            = 0x00002000;
constexpr uint32_t GPIO_BSRR_BS_14            = 0x00004000;
constexpr uint32_t GPIO_BSRR_BS_15            = 0x00008000;
constexpr uint32_t GPIO_BSRR_BR_0             = 0x00010000;
constexpr uint32_t GPIO_BSRR_BR_1             = 0x00020000;
constexpr uint32_t GPIO_BSRR_BR_2             = 0x00040000;
constexpr uint32_t GPIO_BSRR_BR_3             = 0x00080000;
constexpr uint32_t GPIO_BSRR_BR_4             = 0x00100000;
constexpr uint32_t GPIO_BSRR_BR_5             = 0x00200000;
constexpr uint32_t GPIO_BSRR_BR_6             = 0x00400000;
constexpr uint32_t GPIO_BSRR_BR_7             = 0x00800000;
constexpr uint32_t GPIO_BSRR_BR_8             = 0x01000000;
constexpr uint32_t GPIO_BSRR_BR_9             = 0x02000000;
constexpr uint32_t GPIO_BSRR_BR_10            = 0x04000000;
constexpr uint32_t GPIO_BSRR_BR_11            = 0x08000000;
constexpr uint32_t GPIO_BSRR_BR_12            = 0x10000000;
constexpr uint32_t GPIO_BSRR_BR_13            = 0x20000000;
constexpr uint32_t GPIO_BSRR_BR_14            = 0x40000000;
constexpr uint32_t GPIO_BSRR_BR_15            = 0x80000000;

/****************** Bit definition for GPIO_LCKR register  ********************/
constexpr uint32_t GPIO_LCKR_LCK0             = 0x00000001;
constexpr uint32_t GPIO_LCKR_LCK1             = 0x00000002;
constexpr uint32_t GPIO_LCKR_LCK2             = 0x00000004;
constexpr uint32_t GPIO_LCKR_LCK3             = 0x00000008;
constexpr uint32_t GPIO_LCKR_LCK4             = 0x00000010;
constexpr uint32_t GPIO_LCKR_LCK5             = 0x00000020;
constexpr uint32_t GPIO_LCKR_LCK6             = 0x00000040;
constexpr uint32_t GPIO_LCKR_LCK7             = 0x00000080;
constexpr uint32_t GPIO_LCKR_LCK8             = 0x00000100;
constexpr uint32_t GPIO_LCKR_LCK9             = 0x00000200;
constexpr uint32_t GPIO_LCKR_LCK10            = 0x00000400;
constexpr uint32_t GPIO_LCKR_LCK11            = 0x00000800;
constexpr uint32_t GPIO_LCKR_LCK12            = 0x00001000;
constexpr uint32_t GPIO_LCKR_LCK13            = 0x00002000;
constexpr uint32_t GPIO_LCKR_LCK14            = 0x00004000;
constexpr uint32_t GPIO_LCKR_LCK15            = 0x00008000;
constexpr uint32_t GPIO_LCKR_LCKK             = 0x00010000;

/****************** Bit definition for GPIO_AFRL register  ********************/
constexpr uint32_t GPIO_AFRL_AFRL0            = 0x0000000F;
constexpr uint32_t GPIO_AFRL_AFRL1            = 0x000000F0;
constexpr uint32_t GPIO_AFRL_AFRL2            = 0x00000F00;
constexpr uint32_t GPIO_AFRL_AFRL3            = 0x0000F000;
constexpr uint32_t GPIO_AFRL_AFRL4            = 0x000F0000;
constexpr uint32_t GPIO_AFRL_AFRL5            = 0x00F00000;
constexpr uint32_t GPIO_AFRL_AFRL6            = 0x0F000000;
constexpr uint32_t GPIO_AFRL_AFRL7            = 0xF0000000;

/****************** Bit definition for GPIO_AFRH register  ********************/
constexpr uint32_t GPIO_AFRH_AFRH0            = 0x0000000F;
constexpr uint32_t GPIO_AFRH_AFRH1            = 0x000000F0;
constexpr uint32_t GPIO_AFRH_AFRH2            = 0x00000F00;
constexpr uint32_t GPIO_AFRH_AFRH3            = 0x0000F000;
constexpr uint32_t GPIO_AFRH_AFRH4            = 0x000F0000;
constexpr uint32_t GPIO_AFRH_AFRH5            = 0x00F00000;
constexpr uint32_t GPIO_AFRH_AFRH6            = 0x0F000000;
constexpr uint32_t GPIO_AFRH_AFRH7            = 0xF0000000;

/****************** Bit definition for GPIO_BRR register  *********************/
constexpr uint32_t GPIO_BRR_BR_0              = 0x00000001;
constexpr uint32_t GPIO_BRR_BR_1              = 0x00000002;
constexpr uint32_t GPIO_BRR_BR_2              = 0x00000004;
constexpr uint32_t GPIO_BRR_BR_3              = 0x00000008;
constexpr uint32_t GPIO_BRR_BR_4              = 0x00000010;
constexpr uint32_t GPIO_BRR_BR_5              = 0x00000020;
constexpr uint32_t GPIO_BRR_BR_6              = 0x00000040;
constexpr uint32_t GPIO_BRR_BR_7              = 0x00000080;
constexpr uint32_t GPIO_BRR_BR_8              = 0x00000100;
constexpr uint32_t GPIO_BRR_BR_9              = 0x00000200;
constexpr uint32_t GPIO_BRR_BR_10             = 0x00000400;
constexpr uint32_t GPIO_BRR_BR_11             = 0x00000800;
constexpr uint32_t GPIO_BRR_BR_12             = 0x00001000;
constexpr uint32_t GPIO_BRR_BR_13             = 0x00002000;
constexpr uint32_t GPIO_BRR_BR_14             = 0x00004000;
constexpr uint32_t GPIO_BRR_BR_15             = 0x00008000;

/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface = I2C;              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CR1 register  *******************/
constexpr uint32_t  I2C_CR1_PE                          = 0x00000001;        /*!< Peripheral enable */
constexpr uint32_t  I2C_CR1_TXIE                        = 0x00000002;        /*!< TX interrupt enable */
constexpr uint32_t  I2C_CR1_RXIE                        = 0x00000004;        /*!< RX interrupt enable */
constexpr uint32_t  I2C_CR1_ADDRIE                      = 0x00000008;        /*!< Address match interrupt enable */
constexpr uint32_t  I2C_CR1_NACKIE                      = 0x00000010;        /*!< NACK received interrupt enable */
constexpr uint32_t  I2C_CR1_STOPIE                      = 0x00000020;        /*!< STOP detection interrupt enable */
constexpr uint32_t  I2C_CR1_TCIE                        = 0x00000040;        /*!< Transfer complete interrupt enable */
constexpr uint32_t  I2C_CR1_ERRIE                       = 0x00000080;        /*!< Errors interrupt enable */
constexpr uint32_t  I2C_CR1_DNF                        = 0x00000F00;        /*!< Digital noise filter */
constexpr uint32_t  I2C_CR1_ANFOFF                      = 0x00001000;        /*!< Analog noise filter OFF */
constexpr uint32_t  I2C_CR1_SWRST                       = 0x00002000;        /*!< Software reset */
constexpr uint32_t  I2C_CR1_TXDMAEN                     = 0x00004000;        /*!< DMA transmission requests enable */
constexpr uint32_t  I2C_CR1_RXDMAEN                     = 0x00008000;        /*!< DMA reception requests enable */
constexpr uint32_t  I2C_CR1_SBC                         = 0x00010000;        /*!< Slave byte control */
constexpr uint32_t  I2C_CR1_NOSTRETCH                   = 0x00020000;        /*!< Clock stretching disable */
constexpr uint32_t  I2C_CR1_WUPEN                       = 0x00040000;        /*!< Wakeup from STOP enable */
constexpr uint32_t  I2C_CR1_GCEN                        = 0x00080000;        /*!< General call enable */
constexpr uint32_t  I2C_CR1_SMBHEN                      = 0x00100000;        /*!< SMBus host address enable */
constexpr uint32_t  I2C_CR1_SMBDEN                      = 0x00200000;        /*!< SMBus device default address enable */
constexpr uint32_t  I2C_CR1_ALERTEN                     = 0x00400000;        /*!< SMBus alert enable */
constexpr uint32_t  I2C_CR1_PECEN                       = 0x00800000;        /*!< PEC enable */

/******************  Bit definition for I2C_CR2 register  ********************/
constexpr uint32_t  I2C_CR2_SADD                        = 0x000003FF;        /*!< Slave address = master mode; */
constexpr uint32_t  I2C_CR2_RD_WRN                      = 0x00000400;        /*!< Transfer direction = master mode; */
constexpr uint32_t  I2C_CR2_ADD10                       = 0x00000800;        /*!< 10-bit addressing mode = master mode; */
constexpr uint32_t  I2C_CR2_HEAD10R                     = 0x00001000;        /*!< 10-bit address header only read direction = master mode; */
constexpr uint32_t  I2C_CR2_START                       = 0x00002000;        /*!< START generation */
constexpr uint32_t  I2C_CR2_STOP                        = 0x00004000;        /*!< STOP generation = master mode; */
constexpr uint32_t  I2C_CR2_NACK                        = 0x00008000;        /*!< NACK generation = slave mode; */
constexpr uint32_t  I2C_CR2_NBYTES                      = 0x00FF0000;        /*!< Number of bytes */
constexpr uint32_t  I2C_CR2_RELOAD                      = 0x01000000;        /*!< NBYTES reload mode */
constexpr uint32_t  I2C_CR2_AUTOEND                     = 0x02000000;        /*!< Automatic end mode = master mode; */
constexpr uint32_t  I2C_CR2_PECBYTE                     = 0x04000000;        /*!< Packet error checking byte */

/*******************  Bit definition for I2C_OAR1 register  ******************/
constexpr uint32_t  I2C_OAR1_OA1                        = 0x000003FF;        /*!< Interface own address 1 */
constexpr uint32_t  I2C_OAR1_OA1MODE                    = 0x00000400;        /*!< Own address 1 10-bit mode */
constexpr uint32_t  I2C_OAR1_OA1EN                      = 0x00008000;        /*!< Own address 1 enable */

/*******************  Bit definition for I2C_OAR2 register  *******************/
constexpr uint32_t  I2C_OAR2_OA2                        = 0x000000FE;        /*!< Interface own address 2 */
constexpr uint32_t  I2C_OAR2_OA2MSK                     = 0x00000700;        /*!< Own address 2 masks */
constexpr uint32_t  I2C_OAR2_OA2EN                      = 0x00008000;        /*!< Own address 2 enable */

/*******************  Bit definition for I2C_TIMINGR register *****************/
constexpr uint32_t  I2C_TIMINGR_SCLL                    = 0x000000FF;        /*!< SCL low period = master mode; */
constexpr uint32_t  I2C_TIMINGR_SCLH                    = 0x0000FF00;        /*!< SCL high period = master mode; */
constexpr uint32_t  I2C_TIMINGR_SDADEL                  = 0x000F0000;        /*!< Data hold time */
constexpr uint32_t  I2C_TIMINGR_SCLDEL                  = 0x00F00000;        /*!< Data setup time */
constexpr uint32_t  I2C_TIMINGR_PRESC                   = 0xF0000000;        /*!< Timings prescaler */

/******************* Bit definition for I2C_TIMEOUTR register *****************/
constexpr uint32_t  I2C_TIMEOUTR_TIMEOUTA               = 0x00000FFF;        /*!< Bus timeout A */
constexpr uint32_t  I2C_TIMEOUTR_TIDLE                  = 0x00001000;        /*!< Idle clock timeout detection */
constexpr uint32_t  I2C_TIMEOUTR_TIMOUTEN               = 0x00008000;        /*!< Clock timeout enable */
constexpr uint32_t  I2C_TIMEOUTR_TIMEOUTB               = 0x0FFF0000;        /*!< Bus timeout B*/
constexpr uint32_t  I2C_TIMEOUTR_TEXTEN                 = 0x80000000;        /*!< Extended clock timeout enable */

/******************  Bit definition for I2C_ISR register  *********************/
constexpr uint32_t  I2C_ISR_TXE                         = 0x00000001;        /*!< Transmit data register empty */
constexpr uint32_t  I2C_ISR_TXIS                        = 0x00000002;        /*!< Transmit interrupt status */
constexpr uint32_t  I2C_ISR_RXNE                        = 0x00000004;        /*!< Receive data register not empty */
constexpr uint32_t  I2C_ISR_ADDR                        = 0x00000008;        /*!< Address matched = slave mode;*/
constexpr uint32_t  I2C_ISR_NACKF                       = 0x00000010;        /*!< NACK received flag */
constexpr uint32_t  I2C_ISR_STOPF                       = 0x00000020;        /*!< STOP detection flag */
constexpr uint32_t  I2C_ISR_TC                          = 0x00000040;        /*!< Transfer complete = master mode; */
constexpr uint32_t  I2C_ISR_TCR                         = 0x00000080;        /*!< Transfer complete reload */
constexpr uint32_t  I2C_ISR_BERR                        = 0x00000100;        /*!< Bus error */
constexpr uint32_t  I2C_ISR_ARLO                        = 0x00000200;        /*!< Arbitration lost */
constexpr uint32_t  I2C_ISR_OVR                         = 0x00000400;        /*!< Overrun/Underrun */
constexpr uint32_t  I2C_ISR_PECERR                      = 0x00000800;        /*!< PEC error in reception */
constexpr uint32_t  I2C_ISR_TIMEOUT                     = 0x00001000;        /*!< Timeout or Tlow detection flag */
constexpr uint32_t  I2C_ISR_ALERT                       = 0x00002000;        /*!< SMBus alert */
constexpr uint32_t  I2C_ISR_BUSY                        = 0x00008000;        /*!< Bus busy */
constexpr uint32_t  I2C_ISR_DIR                         = 0x00010000;        /*!< Transfer direction = slave mode; */
constexpr uint32_t  I2C_ISR_ADDCODE                     = 0x00FE0000;        /*!< Address match code = slave mode; */

/******************  Bit definition for I2C_ICR register  *********************/
constexpr uint32_t  I2C_ICR_ADDRCF                      = 0x00000008;        /*!< Address matched clear flag */
constexpr uint32_t  I2C_ICR_NACKCF                      = 0x00000010;        /*!< NACK clear flag */
constexpr uint32_t  I2C_ICR_STOPCF                      = 0x00000020;        /*!< STOP detection clear flag */
constexpr uint32_t  I2C_ICR_BERRCF                      = 0x00000100;        /*!< Bus error clear flag */
constexpr uint32_t  I2C_ICR_ARLOCF                      = 0x00000200;        /*!< Arbitration lost clear flag */
constexpr uint32_t  I2C_ICR_OVRCF                       = 0x00000400;        /*!< Overrun/Underrun clear flag */
constexpr uint32_t  I2C_ICR_PECCF                       = 0x00000800;        /*!< PAC error clear flag */
constexpr uint32_t  I2C_ICR_TIMOUTCF                    = 0x00001000;        /*!< Timeout clear flag */
constexpr uint32_t  I2C_ICR_ALERTCF                     = 0x00002000;        /*!< Alert clear flag */

/******************  Bit definition for I2C_PECR register  ********************/
constexpr uint32_t  I2C_PECR_PEC                        = 0x000000FF;        /*!< PEC register */

/******************  Bit definition for I2C_RXDR register  *********************/
constexpr uint32_t  I2C_RXDR_RXDATA                     = 0x000000FF;        /*!< 8-bit receive data */

/******************  Bit definition for I2C_TXDR register  *********************/
constexpr uint32_t  I2C_TXDR_TXDATA                     = 0x000000FF;        /*!< 8-bit transmit data */


/******************************************************************************/
/*                                                                            */
/*                           Independent WATCHDOG = IWDG;                      */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for IWDG_KR register  ********************/
constexpr uint32_t  IWDG_KR_KEY                         = 0x0000FFFF;        /*!< Key value = write only, read 0000h; */

/*******************  Bit definition for IWDG_PR register  ********************/
constexpr uint32_t  IWDG_PR_PR                          = 0x00000007;        /*!< PR[2:0] = Prescaler divider; */
constexpr uint32_t  IWDG_PR_PR_0                        = 0x00000001;        /*!< Bit 0 */
constexpr uint32_t  IWDG_PR_PR_1                        = 0x00000002;        /*!< Bit 1 */
constexpr uint32_t  IWDG_PR_PR_2                        = 0x00000004;        /*!< Bit 2 */

/*******************  Bit definition for IWDG_RLR register  *******************/
constexpr uint32_t  IWDG_RLR_RL                         = 0x00000FFF;        /*!< Watchdog counter reload value */

/*******************  Bit definition for IWDG_SR register  ********************/
constexpr uint32_t  IWDG_SR_PVU                         = 0x00000001;        /*!< Watchdog prescaler value update */
constexpr uint32_t  IWDG_SR_RVU                         = 0x00000002;        /*!< Watchdog counter reload value update */
constexpr uint32_t  IWDG_SR_WVU                         = 0x00000004;        /*!< Watchdog counter window value update */

/*******************  Bit definition for IWDG_KR register  ********************/
constexpr uint32_t  IWDG_WINR_WIN                       = 0x00000FFF;        /*!< Watchdog counter window value */

/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for PWR_CR register  ********************/
constexpr uint32_t  PWR_CR_LPDS                         = 0x00000001;     /*!< Low-power Deepsleep */
constexpr uint32_t  PWR_CR_PDDS                         = 0x00000002;     /*!< Power Down Deepsleep */
constexpr uint32_t  PWR_CR_CWUF                         = 0x00000004;     /*!< Clear Wakeup Flag */
constexpr uint32_t  PWR_CR_CSBF                         = 0x00000008;     /*!< Clear Standby Flag */
constexpr uint32_t  PWR_CR_PVDE                         = 0x00000010;     /*!< Power Voltage Detector Enable */

constexpr uint32_t  PWR_CR_PLS                          = 0x000000E0;     /*!< PLS[2:0] bits = PVD Level Selection; */
constexpr uint32_t  PWR_CR_PLS_0                        = 0x00000020;     /*!< Bit 0 */
constexpr uint32_t  PWR_CR_PLS_1                        = 0x00000040;     /*!< Bit 1 */
constexpr uint32_t  PWR_CR_PLS_2                        = 0x00000080;     /*!< Bit 2 */

/*!< PVD level configuration */
constexpr uint32_t  PWR_CR_PLS_LEV0                     = 0x00000000;     /*!< PVD level 0 */
constexpr uint32_t  PWR_CR_PLS_LEV1                     = 0x00000020;     /*!< PVD level 1 */
constexpr uint32_t  PWR_CR_PLS_LEV2                     = 0x00000040;     /*!< PVD level 2 */
constexpr uint32_t  PWR_CR_PLS_LEV3                     = 0x00000060;     /*!< PVD level 3 */
constexpr uint32_t  PWR_CR_PLS_LEV4                     = 0x00000080;     /*!< PVD level 4 */
constexpr uint32_t  PWR_CR_PLS_LEV5                     = 0x000000A0;     /*!< PVD level 5 */
constexpr uint32_t  PWR_CR_PLS_LEV6                     = 0x000000C0;     /*!< PVD level 6 */
constexpr uint32_t  PWR_CR_PLS_LEV7                     = 0x000000E0;     /*!< PVD level 7 */

constexpr uint32_t  PWR_CR_DBP                          = 0x00000100;     /*!< Disable Backup Domain write protection */

/*******************  Bit definition for PWR_CSR register  ********************/
constexpr uint32_t  PWR_CSR_WUF                         = 0x00000001;     /*!< Wakeup Flag */
constexpr uint32_t  PWR_CSR_SBF                         = 0x00000002;     /*!< Standby Flag */
constexpr uint32_t  PWR_CSR_PVDO                        = 0x00000004;     /*!< PVD Output */
constexpr uint32_t  PWR_CSR_VREFINTRDYF                 = 0x00000008;     /*!< Internal voltage reference = VREFINT; ready flag */

constexpr uint32_t  PWR_CSR_EWUP1                       = 0x00000100;     /*!< Enable WKUP pin 1 */
constexpr uint32_t  PWR_CSR_EWUP2                       = 0x00000200;     /*!< Enable WKUP pin 2 */
constexpr uint32_t  PWR_CSR_EWUP3                       = 0x00000400;     /*!< Enable WKUP pin 3 */

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
constexpr uint32_t  RCC_CR_HSION                        = 0x00000001;
constexpr uint32_t  RCC_CR_HSIRDY                       = 0x00000002;

constexpr uint32_t  RCC_CR_HSITRIM                      = 0x000000F8;
constexpr uint32_t  RCC_CR_HSITRIM_0                    = 0x00000008;/*!<Bit 0 */
constexpr uint32_t  RCC_CR_HSITRIM_1                    = 0x00000010;/*!<Bit 1 */
constexpr uint32_t  RCC_CR_HSITRIM_2                    = 0x00000020;/*!<Bit 2 */
constexpr uint32_t  RCC_CR_HSITRIM_3                    = 0x00000040;/*!<Bit 3 */
constexpr uint32_t  RCC_CR_HSITRIM_4                    = 0x00000080;/*!<Bit 4 */

constexpr uint32_t  RCC_CR_HSICAL                       = 0x0000FF00;
constexpr uint32_t  RCC_CR_HSICAL_0                     = 0x00000100;/*!<Bit 0 */
constexpr uint32_t  RCC_CR_HSICAL_1                     = 0x00000200;/*!<Bit 1 */
constexpr uint32_t  RCC_CR_HSICAL_2                     = 0x00000400;/*!<Bit 2 */
constexpr uint32_t  RCC_CR_HSICAL_3                     = 0x00000800;/*!<Bit 3 */
constexpr uint32_t  RCC_CR_HSICAL_4                     = 0x00001000;/*!<Bit 4 */
constexpr uint32_t  RCC_CR_HSICAL_5                     = 0x00002000;/*!<Bit 5 */
constexpr uint32_t  RCC_CR_HSICAL_6                     = 0x00004000;/*!<Bit 6 */
constexpr uint32_t  RCC_CR_HSICAL_7                     = 0x00008000;/*!<Bit 7 */

constexpr uint32_t  RCC_CR_HSEON                        = 0x00010000;
constexpr uint32_t  RCC_CR_HSERDY                       = 0x00020000;
constexpr uint32_t  RCC_CR_HSEBYP                       = 0x00040000;
constexpr uint32_t  RCC_CR_CSSON                        = 0x00080000;
constexpr uint32_t  RCC_CR_PLLON                        = 0x01000000;
constexpr uint32_t  RCC_CR_PLLRDY                       = 0x02000000;

/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
constexpr uint32_t  RCC_CFGR_SW                         = 0x00000003;        /*!< SW[1:0] bits = System clock Switch; */
constexpr uint32_t  RCC_CFGR_SW_0                       = 0x00000001;        /*!< Bit 0 */
constexpr uint32_t  RCC_CFGR_SW_1                       = 0x00000002;        /*!< Bit 1 */

constexpr uint32_t  RCC_CFGR_SW_HSI                     = 0x00000000;        /*!< HSI selected as system clock */
constexpr uint32_t  RCC_CFGR_SW_HSE                     = 0x00000001;        /*!< HSE selected as system clock */
constexpr uint32_t  RCC_CFGR_SW_PLL                     = 0x00000002;        /*!< PLL selected as system clock */

/*!< SWS configuration */
constexpr uint32_t  RCC_CFGR_SWS                        = 0x0000000C;        /*!< SWS[1:0] bits = System Clock Switch Status; */
constexpr uint32_t  RCC_CFGR_SWS_0                      = 0x00000004;        /*!< Bit 0 */
constexpr uint32_t  RCC_CFGR_SWS_1                      = 0x00000008;        /*!< Bit 1 */

constexpr uint32_t  RCC_CFGR_SWS_HSI                    = 0x00000000;        /*!< HSI oscillator used as system clock */
constexpr uint32_t  RCC_CFGR_SWS_HSE                    = 0x00000004;        /*!< HSE oscillator used as system clock */
constexpr uint32_t  RCC_CFGR_SWS_PLL                    = 0x00000008;        /*!< PLL used as system clock */

/*!< HPRE configuration */
constexpr uint32_t  RCC_CFGR_HPRE                       = 0x000000F0;        /*!< HPRE[3:0] bits = AHB prescaler; */
constexpr uint32_t  RCC_CFGR_HPRE_0                     = 0x00000010;        /*!< Bit 0 */
constexpr uint32_t  RCC_CFGR_HPRE_1                     = 0x00000020;        /*!< Bit 1 */
constexpr uint32_t  RCC_CFGR_HPRE_2                     = 0x00000040;        /*!< Bit 2 */
constexpr uint32_t  RCC_CFGR_HPRE_3                     = 0x00000080;        /*!< Bit 3 */

constexpr uint32_t  RCC_CFGR_HPRE_DIV1                  = 0x00000000;        /*!< SYSCLK not divided */
constexpr uint32_t  RCC_CFGR_HPRE_DIV2                  = 0x00000080;        /*!< SYSCLK divided by 2 */
constexpr uint32_t  RCC_CFGR_HPRE_DIV4                  = 0x00000090;        /*!< SYSCLK divided by 4 */
constexpr uint32_t  RCC_CFGR_HPRE_DIV8                  = 0x000000A0;        /*!< SYSCLK divided by 8 */
constexpr uint32_t  RCC_CFGR_HPRE_DIV16                 = 0x000000B0;        /*!< SYSCLK divided by 16 */
constexpr uint32_t  RCC_CFGR_HPRE_DIV64                 = 0x000000C0;        /*!< SYSCLK divided by 64 */
constexpr uint32_t  RCC_CFGR_HPRE_DIV128                = 0x000000D0;        /*!< SYSCLK divided by 128 */
constexpr uint32_t  RCC_CFGR_HPRE_DIV256                = 0x000000E0;        /*!< SYSCLK divided by 256 */
constexpr uint32_t  RCC_CFGR_HPRE_DIV512                = 0x000000F0;        /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
constexpr uint32_t  RCC_CFGR_PPRE1                      = 0x00000700;        /*!< PRE1[2:0] bits = APB1 prescaler; */
constexpr uint32_t  RCC_CFGR_PPRE1_0                    = 0x00000100;        /*!< Bit 0 */
constexpr uint32_t  RCC_CFGR_PPRE1_1                    = 0x00000200;        /*!< Bit 1 */
constexpr uint32_t  RCC_CFGR_PPRE1_2                    = 0x00000400;        /*!< Bit 2 */

constexpr uint32_t  RCC_CFGR_PPRE1_DIV1                 = 0x00000000;        /*!< HCLK not divided */
constexpr uint32_t  RCC_CFGR_PPRE1_DIV2                 = 0x00000400;        /*!< HCLK divided by 2 */
constexpr uint32_t  RCC_CFGR_PPRE1_DIV4                 = 0x00000500;        /*!< HCLK divided by 4 */
constexpr uint32_t  RCC_CFGR_PPRE1_DIV8                 = 0x00000600;        /*!< HCLK divided by 8 */
constexpr uint32_t  RCC_CFGR_PPRE1_DIV16                = 0x00000700;        /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
constexpr uint32_t  RCC_CFGR_PPRE2                      = 0x00003800;        /*!< PRE2[2:0] bits = APB2 prescaler; */
constexpr uint32_t  RCC_CFGR_PPRE2_0                    = 0x00000800;        /*!< Bit 0 */
constexpr uint32_t  RCC_CFGR_PPRE2_1                    = 0x00001000;        /*!< Bit 1 */
constexpr uint32_t  RCC_CFGR_PPRE2_2                    = 0x00002000;        /*!< Bit 2 */

constexpr uint32_t  RCC_CFGR_PPRE2_DIV1                 = 0x00000000;        /*!< HCLK not divided */
constexpr uint32_t  RCC_CFGR_PPRE2_DIV2                 = 0x00002000;        /*!< HCLK divided by 2 */
constexpr uint32_t  RCC_CFGR_PPRE2_DIV4                 = 0x00002800;        /*!< HCLK divided by 4 */
constexpr uint32_t  RCC_CFGR_PPRE2_DIV8                 = 0x00003000;        /*!< HCLK divided by 8 */
constexpr uint32_t  RCC_CFGR_PPRE2_DIV16                = 0x00003800;        /*!< HCLK divided by 16 */

constexpr uint32_t  RCC_CFGR_PLLSRC                     = 0x00018000;        /*!< PLL entry clock source */
constexpr uint32_t  RCC_CFGR_PLLSRC_HSI_PREDIV          = 0x00008000;        /*!< HSI/PREDIV clock as PLL entry clock source */
constexpr uint32_t  RCC_CFGR_PLLSRC_HSE_PREDIV          = 0x00010000;        /*!< HSE/PREDIV clock selected as PLL entry clock source */

constexpr uint32_t  RCC_CFGR_PLLXTPRE                   = 0x00020000;        /*!< HSE divider for PLL entry */
constexpr uint32_t  RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV1   = 0x00000000;        /*!< HSE/PREDIV clock not divided for PLL entry */
constexpr uint32_t  RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV2   = 0x00020000;        /*!< HSE/PREDIV clock divided by 2 for PLL entry */

/*!< PLLMUL configuration */
constexpr uint32_t  RCC_CFGR_PLLMUL                     = 0x003C0000;        /*!< PLLMUL[3:0] bits = PLL multiplication factor; */
constexpr uint32_t  RCC_CFGR_PLLMUL_0                   = 0x00040000;        /*!< Bit 0 */
constexpr uint32_t  RCC_CFGR_PLLMUL_1                   = 0x00080000;        /*!< Bit 1 */
constexpr uint32_t  RCC_CFGR_PLLMUL_2                   = 0x00100000;        /*!< Bit 2 */
constexpr uint32_t  RCC_CFGR_PLLMUL_3                   = 0x00200000;        /*!< Bit 3 */

constexpr uint32_t  RCC_CFGR_PLLMUL2                    = 0x00000000;        /*!< PLL input clock*2 */
constexpr uint32_t  RCC_CFGR_PLLMUL3                    = 0x00040000;        /*!< PLL input clock*3 */
constexpr uint32_t  RCC_CFGR_PLLMUL4                    = 0x00080000;        /*!< PLL input clock*4 */
constexpr uint32_t  RCC_CFGR_PLLMUL5                    = 0x000C0000;        /*!< PLL input clock*5 */
constexpr uint32_t  RCC_CFGR_PLLMUL6                    = 0x00100000;        /*!< PLL input clock*6 */
constexpr uint32_t  RCC_CFGR_PLLMUL7                    = 0x00140000;        /*!< PLL input clock*7 */
constexpr uint32_t  RCC_CFGR_PLLMUL8                    = 0x00180000;        /*!< PLL input clock*8 */
constexpr uint32_t  RCC_CFGR_PLLMUL9                    = 0x001C0000;        /*!< PLL input clock*9 */
constexpr uint32_t  RCC_CFGR_PLLMUL10                   = 0x00200000;        /*!< PLL input clock10 */
constexpr uint32_t  RCC_CFGR_PLLMUL11                   = 0x00240000;        /*!< PLL input clock*11 */
constexpr uint32_t  RCC_CFGR_PLLMUL12                   = 0x00280000;        /*!< PLL input clock*12 */
constexpr uint32_t  RCC_CFGR_PLLMUL13                   = 0x002C0000;        /*!< PLL input clock*13 */
constexpr uint32_t  RCC_CFGR_PLLMUL14                   = 0x00300000;        /*!< PLL input clock*14 */
constexpr uint32_t  RCC_CFGR_PLLMUL15                   = 0x00340000;        /*!< PLL input clock*15 */
constexpr uint32_t  RCC_CFGR_PLLMUL16                   = 0x00380000;        /*!< PLL input clock*16 */

/*!< USB configuration */
constexpr uint32_t  RCC_CFGR_USBPRE                     = 0x00400000;        /*!< USB prescaler */

constexpr uint32_t  RCC_CFGR_USBPRE_DIV1_5              = 0x00000000;        /*!< USB prescaler is PLL clock divided by 1.5 */
constexpr uint32_t  RCC_CFGR_USBPRE_DIV1                = 0x00400000;        /*!< USB prescaler is PLL clock divided by 1 */

/*!< I2S configuration */
constexpr uint32_t  RCC_CFGR_I2SSRC                     = 0x00800000;        /*!< I2S external clock source selection */

constexpr uint32_t  RCC_CFGR_I2SSRC_SYSCLK              = 0x00000000;        /*!< System clock selected as I2S clock source */
constexpr uint32_t  RCC_CFGR_I2SSRC_EXT                 = 0x00800000;        /*!< External clock selected as I2S clock source */

/*!< MCO configuration */
constexpr uint32_t  RCC_CFGR_MCO                        = 0x07000000;        /*!< MCO[2:0] bits = Microcontroller Clock Output; */
constexpr uint32_t  RCC_CFGR_MCO_0                      = 0x01000000;        /*!< Bit 0 */
constexpr uint32_t  RCC_CFGR_MCO_1                      = 0x02000000;        /*!< Bit 1 */
constexpr uint32_t  RCC_CFGR_MCO_2                      = 0x04000000;        /*!< Bit 2 */

constexpr uint32_t  RCC_CFGR_MCO_NOCLOCK                = 0x00000000;        /*!< No clock */
constexpr uint32_t  RCC_CFGR_MCO_LSI                    = 0x02000000;        /*!< LSI clock selected as MCO source */
constexpr uint32_t  RCC_CFGR_MCO_LSE                    = 0x03000000;        /*!< LSE clock selected as MCO source */
constexpr uint32_t  RCC_CFGR_MCO_SYSCLK                 = 0x04000000;        /*!< System clock selected as MCO source */
constexpr uint32_t  RCC_CFGR_MCO_HSI                    = 0x05000000;        /*!< HSI clock selected as MCO source */
constexpr uint32_t  RCC_CFGR_MCO_HSE                    = 0x06000000;        /*!< HSE clock selected as MCO source  */
constexpr uint32_t  RCC_CFGR_MCO_PLL                    = 0x07000000;        /*!< PLL clock divided by 2 selected as MCO source */

constexpr uint32_t  RCC_CFGR_MCOF                       = 0x10000000;        /*!< Microcontroller Clock Output Flag */

constexpr uint32_t  RCC_CFGR_MCOPRE                     = 0x70000000;        /*!< MCOPRE[3:0] bits = Microcontroller Clock Output Prescaler; */
constexpr uint32_t  RCC_CFGR_MCOPRE_0                   = 0x10000000;        /*!< Bit 0 */
constexpr uint32_t  RCC_CFGR_MCOPRE_1                   = 0x20000000;        /*!< Bit 1 */
constexpr uint32_t  RCC_CFGR_MCOPRE_2                   = 0x40000000;        /*!< Bit 2 */

constexpr uint32_t  RCC_CFGR_PLLNODIV                   = 0x80000000;        /*!< PLL is not divided to MCO */

/*********************  Bit definition for RCC_CIR register  ********************/
constexpr uint32_t  RCC_CIR_LSIRDYF                     = 0x00000001;        /*!< LSI Ready Interrupt flag */
constexpr uint32_t  RCC_CIR_LSERDYF                     = 0x00000002;        /*!< LSE Ready Interrupt flag */
constexpr uint32_t  RCC_CIR_HSIRDYF                     = 0x00000004;        /*!< HSI Ready Interrupt flag */
constexpr uint32_t  RCC_CIR_HSERDYF                     = 0x00000008;        /*!< HSE Ready Interrupt flag */
constexpr uint32_t  RCC_CIR_PLLRDYF                     = 0x00000010;        /*!< PLL Ready Interrupt flag */
constexpr uint32_t  RCC_CIR_CSSF                        = 0x00000080;        /*!< Clock Security System Interrupt flag */
constexpr uint32_t  RCC_CIR_LSIRDYIE                    = 0x00000100;        /*!< LSI Ready Interrupt Enable */
constexpr uint32_t  RCC_CIR_LSERDYIE                    = 0x00000200;        /*!< LSE Ready Interrupt Enable */
constexpr uint32_t  RCC_CIR_HSIRDYIE                    = 0x00000400;        /*!< HSI Ready Interrupt Enable */
constexpr uint32_t  RCC_CIR_HSERDYIE                    = 0x00000800;        /*!< HSE Ready Interrupt Enable */
constexpr uint32_t  RCC_CIR_PLLRDYIE                    = 0x00001000;        /*!< PLL Ready Interrupt Enable */
constexpr uint32_t  RCC_CIR_LSIRDYC                     = 0x00010000;        /*!< LSI Ready Interrupt Clear */
constexpr uint32_t  RCC_CIR_LSERDYC                     = 0x00020000;        /*!< LSE Ready Interrupt Clear */
constexpr uint32_t  RCC_CIR_HSIRDYC                     = 0x00040000;        /*!< HSI Ready Interrupt Clear */
constexpr uint32_t  RCC_CIR_HSERDYC                     = 0x00080000;        /*!< HSE Ready Interrupt Clear */
constexpr uint32_t  RCC_CIR_PLLRDYC                     = 0x00100000;        /*!< PLL Ready Interrupt Clear */
constexpr uint32_t  RCC_CIR_CSSC                        = 0x00800000;        /*!< Clock Security System Interrupt Clear */

/******************  Bit definition for RCC_APB2RSTR register  *****************/
constexpr uint32_t  RCC_APB2RSTR_SYSCFGRST              = 0x00000001;        /*!< SYSCFG reset */
constexpr uint32_t  RCC_APB2RSTR_TIM1RST                = 0x00000800;        /*!< TIM1 reset */
constexpr uint32_t  RCC_APB2RSTR_SPI1RST                = 0x00001000;        /*!< SPI1 reset */
constexpr uint32_t  RCC_APB2RSTR_TIM8RST                = 0x00002000;        /*!< TIM8 reset */
constexpr uint32_t  RCC_APB2RSTR_USART1RST              = 0x00004000;        /*!< USART1 reset */
constexpr uint32_t  RCC_APB2RSTR_SPI4RST                = 0x00008000;        /*!< SPI4 reset */
constexpr uint32_t  RCC_APB2RSTR_TIM15RST               = 0x00010000;        /*!< TIM15 reset */
constexpr uint32_t  RCC_APB2RSTR_TIM16RST               = 0x00020000;        /*!< TIM16 reset */
constexpr uint32_t  RCC_APB2RSTR_TIM17RST               = 0x00040000;        /*!< TIM17 reset */
constexpr uint32_t  RCC_APB2RSTR_TIM20RST               = 0x00100000;        /*!< TIM20 reset */

/******************  Bit definition for RCC_APB1RSTR register  ******************/
constexpr uint32_t  RCC_APB1RSTR_TIM2RST                = 0x00000001;        /*!< Timer 2 reset */
constexpr uint32_t  RCC_APB1RSTR_TIM3RST                = 0x00000002;        /*!< Timer 3 reset */
constexpr uint32_t  RCC_APB1RSTR_TIM4RST                = 0x00000004;        /*!< Timer 4 reset */
constexpr uint32_t  RCC_APB1RSTR_TIM6RST                = 0x00000010;        /*!< Timer 6 reset */
constexpr uint32_t  RCC_APB1RSTR_TIM7RST                = 0x00000020;        /*!< Timer 7 reset */
constexpr uint32_t  RCC_APB1RSTR_WWDGRST                = 0x00000800;        /*!< Window Watchdog reset */
constexpr uint32_t  RCC_APB1RSTR_SPI2RST                = 0x00004000;        /*!< SPI2 reset */
constexpr uint32_t  RCC_APB1RSTR_SPI3RST                = 0x00008000;        /*!< SPI3 reset */
constexpr uint32_t  RCC_APB1RSTR_USART2RST              = 0x00020000;        /*!< USART 2 reset */
constexpr uint32_t  RCC_APB1RSTR_USART3RST              = 0x00040000;        /*!< USART 3 reset */
constexpr uint32_t  RCC_APB1RSTR_UART4RST               = 0x00080000;        /*!< UART 4 reset */
constexpr uint32_t  RCC_APB1RSTR_UART5RST               = 0x00100000;        /*!< UART 5 reset */
constexpr uint32_t  RCC_APB1RSTR_I2C1RST                = 0x00200000;        /*!< I2C 1 reset */
constexpr uint32_t  RCC_APB1RSTR_I2C2RST                = 0x00400000;        /*!< I2C 2 reset */
constexpr uint32_t  RCC_APB1RSTR_USBRST                 = 0x00800000;        /*!< USB reset */
constexpr uint32_t  RCC_APB1RSTR_CANRST                 = 0x02000000;        /*!< CAN reset */
constexpr uint32_t  RCC_APB1RSTR_PWRRST                 = 0x10000000;        /*!< PWR reset */
constexpr uint32_t  RCC_APB1RSTR_DAC1RST                = 0x20000000;        /*!< DAC 1 reset */
constexpr uint32_t  RCC_APB1RSTR_I2C3RST                = 0x40000000;        /*!< I2C 3 reset */

/******************  Bit definition for RCC_AHBENR register  ******************/
constexpr uint32_t  RCC_AHBENR_DMA1EN                   = 0x00000001;        /*!< DMA1 clock enable */
constexpr uint32_t  RCC_AHBENR_DMA2EN                   = 0x00000002;        /*!< DMA2 clock enable */
constexpr uint32_t  RCC_AHBENR_SRAMEN                   = 0x00000004;        /*!< SRAM interface clock enable */
constexpr uint32_t  RCC_AHBENR_FLITFEN                  = 0x00000010;        /*!< FLITF clock enable */
constexpr uint32_t  RCC_AHBENR_FMCEN                    = 0x00000020;        /*!< FMC clock enable */
constexpr uint32_t  RCC_AHBENR_CRCEN                    = 0x00000040;        /*!< CRC clock enable */
constexpr uint32_t  RCC_AHBENR_GPIOHEN                  = 0x00010000;        /*!< GPIOH clock enable */
constexpr uint32_t  RCC_AHBENR_GPIOAEN                  = 0x00020000;        /*!< GPIOA clock enable */
constexpr uint32_t  RCC_AHBENR_GPIOBEN                  = 0x00040000;        /*!< GPIOB clock enable */
constexpr uint32_t  RCC_AHBENR_GPIOCEN                  = 0x00080000;        /*!< GPIOC clock enable */
constexpr uint32_t  RCC_AHBENR_GPIODEN                  = 0x00100000;        /*!< GPIOD clock enable */
constexpr uint32_t  RCC_AHBENR_GPIOEEN                  = 0x00200000;        /*!< GPIOE clock enable */
constexpr uint32_t  RCC_AHBENR_GPIOFEN                  = 0x00400000;        /*!< GPIOF clock enable */
constexpr uint32_t  RCC_AHBENR_GPIOGEN                  = 0x00800000;        /*!< GPIOG clock enable */
constexpr uint32_t  RCC_AHBENR_TSCEN                    = 0x01000000;        /*!< TS clock enable */
constexpr uint32_t  RCC_AHBENR_ADC12EN                  = 0x10000000;        /*!< ADC1/ ADC2 clock enable */
constexpr uint32_t  RCC_AHBENR_ADC34EN                  = 0x20000000;        /*!< ADC3/ ADC4 clock enable */

/*****************  Bit definition for RCC_APB2ENR register  ******************/
constexpr uint32_t  RCC_APB2ENR_SYSCFGEN                = 0x00000001;        /*!< SYSCFG clock enable */
constexpr uint32_t  RCC_APB2ENR_TIM1EN                  = 0x00000800;        /*!< TIM1 clock enable */
constexpr uint32_t  RCC_APB2ENR_SPI1EN                  = 0x00001000;        /*!< SPI1 clock enable */
constexpr uint32_t  RCC_APB2ENR_TIM8EN                  = 0x00002000;        /*!< TIM8 clock enable */
constexpr uint32_t  RCC_APB2ENR_USART1EN                = 0x00004000;        /*!< USART1 clock enable */
constexpr uint32_t  RCC_APB2ENR_SPI4EN                  = 0x00008000;        /*!< SPI4 clock enable */
constexpr uint32_t  RCC_APB2ENR_TIM15EN                 = 0x00010000;        /*!< TIM15 clock enable */
constexpr uint32_t  RCC_APB2ENR_TIM16EN                 = 0x00020000;        /*!< TIM16 clock enable */
constexpr uint32_t  RCC_APB2ENR_TIM17EN                 = 0x00040000;        /*!< TIM17 clock enable */
constexpr uint32_t  RCC_APB2ENR_TIM20EN                 = 0x00100000;        /*!< TIM20 clock enable */

/******************  Bit definition for RCC_APB1ENR register  ******************/
constexpr uint32_t  RCC_APB1ENR_TIM2EN                  = 0x00000001;        /*!< Timer 2 clock enable */
constexpr uint32_t  RCC_APB1ENR_TIM3EN                  = 0x00000002;        /*!< Timer 3 clock enable */
constexpr uint32_t  RCC_APB1ENR_TIM4EN                  = 0x00000004;        /*!< Timer 4 clock enable */
constexpr uint32_t  RCC_APB1ENR_TIM6EN                  = 0x00000010;        /*!< Timer 6 clock enable */
constexpr uint32_t  RCC_APB1ENR_TIM7EN                  = 0x00000020;        /*!< Timer 7 clock enable */
constexpr uint32_t  RCC_APB1ENR_WWDGEN                  = 0x00000800;        /*!< Window Watchdog clock enable */
constexpr uint32_t  RCC_APB1ENR_SPI2EN                  = 0x00004000;        /*!< SPI2 clock enable */
constexpr uint32_t  RCC_APB1ENR_SPI3EN                  = 0x00008000;        /*!< SPI3 clock enable */
constexpr uint32_t  RCC_APB1ENR_USART2EN                = 0x00020000;        /*!< USART 2 clock enable */
constexpr uint32_t  RCC_APB1ENR_USART3EN                = 0x00040000;        /*!< USART 3 clock enable */
constexpr uint32_t  RCC_APB1ENR_UART4EN                 = 0x00080000;        /*!< UART 4 clock enable */
constexpr uint32_t  RCC_APB1ENR_UART5EN                 = 0x00100000;        /*!< UART 5 clock enable */
constexpr uint32_t  RCC_APB1ENR_I2C1EN                  = 0x00200000;        /*!< I2C 1 clock enable */
constexpr uint32_t  RCC_APB1ENR_I2C2EN                  = 0x00400000;        /*!< I2C 2 clock enable */
constexpr uint32_t  RCC_APB1ENR_USBEN                   = 0x00800000;        /*!< USB clock enable */
constexpr uint32_t  RCC_APB1ENR_CANEN                   = 0x02000000;        /*!< CAN clock enable */
constexpr uint32_t  RCC_APB1ENR_PWREN                   = 0x10000000;        /*!< PWR clock enable */
constexpr uint32_t  RCC_APB1ENR_DAC1EN                  = 0x20000000;        /*!< DAC 1 clock enable */
constexpr uint32_t  RCC_APB1ENR_I2C3EN                  = 0x40000000;        /*!< I2C 3 clock enable */

/********************  Bit definition for RCC_BDCR register  ******************/
constexpr uint32_t  RCC_BDCR_LSE                        = 0x00000007;        /*!< External Low Speed oscillator [2:0] bits */
constexpr uint32_t  RCC_BDCR_LSEON                      = 0x00000001;        /*!< External Low Speed oscillator enable */
constexpr uint32_t  RCC_BDCR_LSERDY                     = 0x00000002;        /*!< External Low Speed oscillator Ready */
constexpr uint32_t  RCC_BDCR_LSEBYP                     = 0x00000004;        /*!< External Low Speed oscillator Bypass */

constexpr uint32_t  RCC_BDCR_LSEDRV                     = 0x00000018;        /*!< LSEDRV[1:0] bits = LSE Osc. drive capability; */
constexpr uint32_t  RCC_BDCR_LSEDRV_0                   = 0x00000008;        /*!< Bit 0 */
constexpr uint32_t  RCC_BDCR_LSEDRV_1                   = 0x00000010;        /*!< Bit 1 */

constexpr uint32_t  RCC_BDCR_RTCSEL                     = 0x00000300;        /*!< RTCSEL[1:0] bits = RTC clock source selection; */
constexpr uint32_t  RCC_BDCR_RTCSEL_0                   = 0x00000100;        /*!< Bit 0 */
constexpr uint32_t  RCC_BDCR_RTCSEL_1                   = 0x00000200;        /*!< Bit 1 */

/*!< RTC configuration */
constexpr uint32_t  RCC_BDCR_RTCSEL_NOCLOCK             = 0x00000000;        /*!< No clock */
constexpr uint32_t  RCC_BDCR_RTCSEL_LSE                 = 0x00000100;        /*!< LSE oscillator clock used as RTC clock */
constexpr uint32_t  RCC_BDCR_RTCSEL_LSI                 = 0x00000200;        /*!< LSI oscillator clock used as RTC clock */
constexpr uint32_t  RCC_BDCR_RTCSEL_HSE                 = 0x00000300;        /*!< HSE oscillator clock divided by 32 used as RTC clock */

constexpr uint32_t  RCC_BDCR_RTCEN                      = 0x00008000;        /*!< RTC clock enable */
constexpr uint32_t  RCC_BDCR_BDRST                      = 0x00010000;        /*!< Backup domain software reset  */

/********************  Bit definition for RCC_CSR register  *******************/
constexpr uint32_t  RCC_CSR_LSION                       = 0x00000001;        /*!< Internal Low Speed oscillator enable */
constexpr uint32_t  RCC_CSR_LSIRDY                      = 0x00000002;        /*!< Internal Low Speed oscillator Ready */
constexpr uint32_t  RCC_CSR_RMVF                        = 0x01000000;        /*!< Remove reset flag */
constexpr uint32_t  RCC_CSR_OBLRSTF                     = 0x02000000;        /*!< OBL reset flag */
constexpr uint32_t  RCC_CSR_PINRSTF                     = 0x04000000;        /*!< PIN reset flag */
constexpr uint32_t  RCC_CSR_PORRSTF                     = 0x08000000;        /*!< POR/PDR reset flag */
constexpr uint32_t  RCC_CSR_SFTRSTF                     = 0x10000000;        /*!< Software Reset flag */
constexpr uint32_t  RCC_CSR_IWDGRSTF                    = 0x20000000;        /*!< Independent Watchdog reset flag */
constexpr uint32_t  RCC_CSR_WWDGRSTF                    = 0x40000000;        /*!< Window watchdog reset flag */
constexpr uint32_t  RCC_CSR_LPWRRSTF                    = 0x80000000;        /*!< Low-Power reset flag */

/*******************  Bit definition for RCC_AHBRSTR register  ****************/
constexpr uint32_t  RCC_AHBRSTR_FMCRST                  = 0x00000020;         /*!< FMC reset */
constexpr uint32_t  RCC_AHBRSTR_GPIOHRST                = 0x00010000;         /*!< GPIOH reset */
constexpr uint32_t  RCC_AHBRSTR_GPIOARST                = 0x00020000;         /*!< GPIOA reset */
constexpr uint32_t  RCC_AHBRSTR_GPIOBRST                = 0x00040000;         /*!< GPIOB reset */
constexpr uint32_t  RCC_AHBRSTR_GPIOCRST                = 0x00080000;         /*!< GPIOC reset */
constexpr uint32_t  RCC_AHBRSTR_GPIODRST                = 0x00100000;         /*!< GPIOD reset */
constexpr uint32_t  RCC_AHBRSTR_GPIOERST                = 0x00200000;         /*!< GPIOE reset */
constexpr uint32_t  RCC_AHBRSTR_GPIOFRST                = 0x00400000;         /*!< GPIOF reset */
constexpr uint32_t  RCC_AHBRSTR_GPIOGRST                = 0x00800000;         /*!< GPIOG reset */
constexpr uint32_t  RCC_AHBRSTR_TSCRST                  = 0x01000000;         /*!< TSC reset */
constexpr uint32_t  RCC_AHBRSTR_ADC12RST                = 0x10000000;         /*!< ADC1 & ADC2 reset */
constexpr uint32_t  RCC_AHBRSTR_ADC34RST                = 0x20000000;         /*!< ADC3 & ADC4 reset */

/*******************  Bit definition for RCC_CFGR2 register  ******************/
/*!< PREDIV configuration */
constexpr uint32_t  RCC_CFGR2_PREDIV                    = 0x0000000F;        /*!< PREDIV[3:0] bits */
constexpr uint32_t  RCC_CFGR2_PREDIV_0                  = 0x00000001;        /*!< Bit 0 */
constexpr uint32_t  RCC_CFGR2_PREDIV_1                  = 0x00000002;        /*!< Bit 1 */
constexpr uint32_t  RCC_CFGR2_PREDIV_2                  = 0x00000004;        /*!< Bit 2 */
constexpr uint32_t  RCC_CFGR2_PREDIV_3                  = 0x00000008;        /*!< Bit 3 */

constexpr uint32_t  RCC_CFGR2_PREDIV_DIV1               = 0x00000000;        /*!< PREDIV input clock not divided */
constexpr uint32_t  RCC_CFGR2_PREDIV_DIV2               = 0x00000001;        /*!< PREDIV input clock divided by 2 */
constexpr uint32_t  RCC_CFGR2_PREDIV_DIV3               = 0x00000002;        /*!< PREDIV input clock divided by 3 */
constexpr uint32_t  RCC_CFGR2_PREDIV_DIV4               = 0x00000003;        /*!< PREDIV input clock divided by 4 */
constexpr uint32_t  RCC_CFGR2_PREDIV_DIV5               = 0x00000004;        /*!< PREDIV input clock divided by 5 */
constexpr uint32_t  RCC_CFGR2_PREDIV_DIV6               = 0x00000005;        /*!< PREDIV input clock divided by 6 */
constexpr uint32_t  RCC_CFGR2_PREDIV_DIV7               = 0x00000006;        /*!< PREDIV input clock divided by 7 */
constexpr uint32_t  RCC_CFGR2_PREDIV_DIV8               = 0x00000007;        /*!< PREDIV input clock divided by 8 */
constexpr uint32_t  RCC_CFGR2_PREDIV_DIV9               = 0x00000008;        /*!< PREDIV input clock divided by 9 */
constexpr uint32_t  RCC_CFGR2_PREDIV_DIV10              = 0x00000009;        /*!< PREDIV input clock divided by 10 */
constexpr uint32_t  RCC_CFGR2_PREDIV_DIV11              = 0x0000000A;        /*!< PREDIV input clock divided by 11 */
constexpr uint32_t  RCC_CFGR2_PREDIV_DIV12              = 0x0000000B;        /*!< PREDIV input clock divided by 12 */
constexpr uint32_t  RCC_CFGR2_PREDIV_DIV13              = 0x0000000C;        /*!< PREDIV input clock divided by 13 */
constexpr uint32_t  RCC_CFGR2_PREDIV_DIV14              = 0x0000000D;        /*!< PREDIV input clock divided by 14 */
constexpr uint32_t  RCC_CFGR2_PREDIV_DIV15              = 0x0000000E;        /*!< PREDIV input clock divided by 15 */
constexpr uint32_t  RCC_CFGR2_PREDIV_DIV16              = 0x0000000F;        /*!< PREDIV input clock divided by 16 */

/*!< ADCPRE12 configuration */
constexpr uint32_t  RCC_CFGR2_ADCPRE12                  = 0x000001F0;        /*!< ADCPRE12[8:4] bits */
constexpr uint32_t  RCC_CFGR2_ADCPRE12_0                = 0x00000010;        /*!< Bit 0 */
constexpr uint32_t  RCC_CFGR2_ADCPRE12_1                = 0x00000020;        /*!< Bit 1 */
constexpr uint32_t  RCC_CFGR2_ADCPRE12_2                = 0x00000040;        /*!< Bit 2 */
constexpr uint32_t  RCC_CFGR2_ADCPRE12_3                = 0x00000080;        /*!< Bit 3 */
constexpr uint32_t  RCC_CFGR2_ADCPRE12_4                = 0x00000100;        /*!< Bit 4 */

constexpr uint32_t  RCC_CFGR2_ADCPRE12_NO               = 0x00000000;        /*!< ADC12 clock disabled, ADC12 can use AHB clock */
constexpr uint32_t  RCC_CFGR2_ADCPRE12_DIV1             = 0x00000100;        /*!< ADC12 PLL clock divided by 1 */
constexpr uint32_t  RCC_CFGR2_ADCPRE12_DIV2             = 0x00000110;        /*!< ADC12 PLL clock divided by 2 */
constexpr uint32_t  RCC_CFGR2_ADCPRE12_DIV4             = 0x00000120;        /*!< ADC12 PLL clock divided by 4 */
constexpr uint32_t  RCC_CFGR2_ADCPRE12_DIV6             = 0x00000130;        /*!< ADC12 PLL clock divided by 6 */
constexpr uint32_t  RCC_CFGR2_ADCPRE12_DIV8             = 0x00000140;        /*!< ADC12 PLL clock divided by 8 */
constexpr uint32_t  RCC_CFGR2_ADCPRE12_DIV10            = 0x00000150;        /*!< ADC12 PLL clock divided by 10 */
constexpr uint32_t  RCC_CFGR2_ADCPRE12_DIV12            = 0x00000160;        /*!< ADC12 PLL clock divided by 12 */
constexpr uint32_t  RCC_CFGR2_ADCPRE12_DIV16            = 0x00000170;        /*!< ADC12 PLL clock divided by 16 */
constexpr uint32_t  RCC_CFGR2_ADCPRE12_DIV32            = 0x00000180;        /*!< ADC12 PLL clock divided by 32 */
constexpr uint32_t  RCC_CFGR2_ADCPRE12_DIV64            = 0x00000190;        /*!< ADC12 PLL clock divided by 64 */
constexpr uint32_t  RCC_CFGR2_ADCPRE12_DIV128           = 0x000001A0;        /*!< ADC12 PLL clock divided by 128 */
constexpr uint32_t  RCC_CFGR2_ADCPRE12_DIV256           = 0x000001B0;        /*!< ADC12 PLL clock divided by 256 */

/*!< ADCPRE34 configuration */
constexpr uint32_t  RCC_CFGR2_ADCPRE34                  = 0x00003E00;        /*!< ADCPRE34[13:5] bits */
constexpr uint32_t  RCC_CFGR2_ADCPRE34_0                = 0x00000200;        /*!< Bit 0 */
constexpr uint32_t  RCC_CFGR2_ADCPRE34_1                = 0x00000400;        /*!< Bit 1 */
constexpr uint32_t  RCC_CFGR2_ADCPRE34_2                = 0x00000800;        /*!< Bit 2 */
constexpr uint32_t  RCC_CFGR2_ADCPRE34_3                = 0x00001000;        /*!< Bit 3 */
constexpr uint32_t  RCC_CFGR2_ADCPRE34_4                = 0x00002000;        /*!< Bit 4 */

constexpr uint32_t  RCC_CFGR2_ADCPRE34_NO               = 0x00000000;        /*!< ADC34 clock disabled, ADC34 can use AHB clock */
constexpr uint32_t  RCC_CFGR2_ADCPRE34_DIV1             = 0x00002000;        /*!< ADC34 PLL clock divided by 1 */
constexpr uint32_t  RCC_CFGR2_ADCPRE34_DIV2             = 0x00002200;        /*!< ADC34 PLL clock divided by 2 */
constexpr uint32_t  RCC_CFGR2_ADCPRE34_DIV4             = 0x00002400;        /*!< ADC34 PLL clock divided by 4 */
constexpr uint32_t  RCC_CFGR2_ADCPRE34_DIV6             = 0x00002600;        /*!< ADC34 PLL clock divided by 6 */
constexpr uint32_t  RCC_CFGR2_ADCPRE34_DIV8             = 0x00002800;        /*!< ADC34 PLL clock divided by 8 */
constexpr uint32_t  RCC_CFGR2_ADCPRE34_DIV10            = 0x00002A00;        /*!< ADC34 PLL clock divided by 10 */
constexpr uint32_t  RCC_CFGR2_ADCPRE34_DIV12            = 0x00002C00;        /*!< ADC34 PLL clock divided by 12 */
constexpr uint32_t  RCC_CFGR2_ADCPRE34_DIV16            = 0x00002E00;        /*!< ADC34 PLL clock divided by 16 */
constexpr uint32_t  RCC_CFGR2_ADCPRE34_DIV32            = 0x00003000;        /*!< ADC34 PLL clock divided by 32 */
constexpr uint32_t  RCC_CFGR2_ADCPRE34_DIV64            = 0x00003200;        /*!< ADC34 PLL clock divided by 64 */
constexpr uint32_t  RCC_CFGR2_ADCPRE34_DIV128           = 0x00003400;        /*!< ADC34 PLL clock divided by 128 */
constexpr uint32_t  RCC_CFGR2_ADCPRE34_DIV256           = 0x00003600;        /*!< ADC34 PLL clock divided by 256 */

/*******************  Bit definition for RCC_CFGR3 register  ******************/
constexpr uint32_t  RCC_CFGR3_USART1SW                  = 0x00000003;        /*!< USART1SW[1:0] bits */
constexpr uint32_t  RCC_CFGR3_USART1SW_0                = 0x00000001;        /*!< Bit 0 */
constexpr uint32_t  RCC_CFGR3_USART1SW_1                = 0x00000002;        /*!< Bit 1 */

constexpr uint32_t  RCC_CFGR3_USART1SW_PCLK             = 0x00000000;        /*!< PCLK1 clock used as USART1 clock source */
constexpr uint32_t  RCC_CFGR3_USART1SW_SYSCLK           = 0x00000001;        /*!< System clock selected as USART1 clock source */
constexpr uint32_t  RCC_CFGR3_USART1SW_LSE              = 0x00000002;        /*!< LSE oscillator clock used as USART1 clock source */
constexpr uint32_t  RCC_CFGR3_USART1SW_HSI              = 0x00000003;        /*!< HSI oscillator clock used as USART1 clock source */

constexpr uint32_t  RCC_CFGR3_I2CSW                     = 0x00000070;        /*!< I2CSW bits */
constexpr uint32_t  RCC_CFGR3_I2C1SW                    = 0x00000010;        /*!< I2C1SW bits */
constexpr uint32_t  RCC_CFGR3_I2C2SW                    = 0x00000020;        /*!< I2C2SW bits */
constexpr uint32_t  RCC_CFGR3_I2C3SW                    = 0x00000040;        /*!< I2C3SW bits */

constexpr uint32_t  RCC_CFGR3_I2C1SW_HSI                = 0x00000000;        /*!< HSI oscillator clock used as I2C1 clock source */
constexpr uint32_t  RCC_CFGR3_I2C1SW_SYSCLK             = 0x00000010;        /*!< System clock selected as I2C1 clock source */
constexpr uint32_t  RCC_CFGR3_I2C2SW_HSI                = 0x00000000;        /*!< HSI oscillator clock used as I2C2 clock source */
constexpr uint32_t  RCC_CFGR3_I2C2SW_SYSCLK             = 0x00000020;        /*!< System clock selected as I2C2 clock source */
constexpr uint32_t  RCC_CFGR3_I2C3SW_HSI                = 0x00000000;        /*!< HSI oscillator clock used as I2C3 clock source */
constexpr uint32_t  RCC_CFGR3_I2C3SW_SYSCLK             = 0x00000040;        /*!< System clock selected as I2C3 clock source */

constexpr uint32_t  RCC_CFGR3_TIMSW                     = 0x0000AF00;        /*!< TIMSW bits */
constexpr uint32_t  RCC_CFGR3_TIM1SW                    = 0x00000100;        /*!< TIM1SW bits */
constexpr uint32_t  RCC_CFGR3_TIM8SW                    = 0x00000200;        /*!< TIM8SW bits */
constexpr uint32_t  RCC_CFGR3_TIM15SW                   = 0x00000400;        /*!< TIM15SW bits */
constexpr uint32_t  RCC_CFGR3_TIM16SW                   = 0x00000800;        /*!< TIM16SW bits */
constexpr uint32_t  RCC_CFGR3_TIM17SW                   = 0x00002000;        /*!< TIM17SW bits */
constexpr uint32_t  RCC_CFGR3_TIM20SW                   = 0x00008000;        /*!< TIM20SW bits */
constexpr uint32_t  RCC_CFGR3_TIM2SW                    = 0x01000000;        /*!< TIM2SW bits */
constexpr uint32_t  RCC_CFGR3_TIM34SW                    = 0x02000000;       /*!< TIM34SW bits */

constexpr uint32_t  RCC_CFGR3_TIM1SW_HCLK               = 0x00000000;        /*!< HCLK used as TIM1 clock source */
constexpr uint32_t  RCC_CFGR3_TIM1SW_PLL                = 0x00000100;        /*!< PLL clock used as TIM1 clock source */

constexpr uint32_t  RCC_CFGR3_TIM8SW_HCLK               = 0x00000000;        /*!< HCLK used as TIM8 clock source */
constexpr uint32_t  RCC_CFGR3_TIM8SW_PLL                = 0x00000200;        /*!< PLL clock used as TIM8 clock source */

constexpr uint32_t  RCC_CFGR3_TIM15SW_HCLK               = 0x00000000;        /*!< HCLK used as TIM15 clock source */
constexpr uint32_t  RCC_CFGR3_TIM15SW_PLL                = 0x00000400;        /*!< PLL clock used as TIM15 clock source */

constexpr uint32_t  RCC_CFGR3_TIM16SW_HCLK               = 0x00000000;        /*!< HCLK used as TIM16 clock source */
constexpr uint32_t  RCC_CFGR3_TIM16SW_PLL                = 0x00000800;        /*!< PLL clock used as TIM16 clock source */

constexpr uint32_t  RCC_CFGR3_TIM17SW_HCLK               = 0x00000000;        /*!< HCLK used as TIM17 clock source */
constexpr uint32_t  RCC_CFGR3_TIM17SW_PLL                = 0x00002000;        /*!< PLL clock used as TIM17 clock source */

constexpr uint32_t  RCC_CFGR3_TIM20SW_HCLK               = 0x00000000;        /*!< HCLK used as TIM20 clock source */
constexpr uint32_t  RCC_CFGR3_TIM20SW_PLL                = 0x00008000;        /*!< PLL clock used as TIM20 clock source */

constexpr uint32_t  RCC_CFGR3_USART2SW                  = 0x00030000;        /*!< USART2SW[1:0] bits */
constexpr uint32_t  RCC_CFGR3_USART2SW_0                = 0x00010000;        /*!< Bit 0 */
constexpr uint32_t  RCC_CFGR3_USART2SW_1                = 0x00020000;        /*!< Bit 1 */

constexpr uint32_t  RCC_CFGR3_USART2SW_PCLK             = 0x00000000;        /*!< PCLK2 clock used as USART2 clock source */
constexpr uint32_t  RCC_CFGR3_USART2SW_SYSCLK           = 0x00010000;        /*!< System clock selected as USART2 clock source */
constexpr uint32_t  RCC_CFGR3_USART2SW_LSE              = 0x00020000;        /*!< LSE oscillator clock used as USART2 clock source */
constexpr uint32_t  RCC_CFGR3_USART2SW_HSI              = 0x00030000;        /*!< HSI oscillator clock used as USART2 clock source */

constexpr uint32_t  RCC_CFGR3_USART3SW                  = 0x000C0000;        /*!< USART3SW[1:0] bits */
constexpr uint32_t  RCC_CFGR3_USART3SW_0                = 0x00040000;        /*!< Bit 0 */
constexpr uint32_t  RCC_CFGR3_USART3SW_1                = 0x00080000;        /*!< Bit 1 */

constexpr uint32_t  RCC_CFGR3_USART3SW_PCLK             = 0x00000000;        /*!< PCLK2 clock used as USART3 clock source */
constexpr uint32_t  RCC_CFGR3_USART3SW_SYSCLK           = 0x00040000;        /*!< System clock selected as USART3 clock source */
constexpr uint32_t  RCC_CFGR3_USART3SW_LSE              = 0x00080000;        /*!< LSE oscillator clock used as USART3 clock source */
constexpr uint32_t  RCC_CFGR3_USART3SW_HSI              = 0x000C0000;        /*!< HSI oscillator clock used as USART3 clock source */

constexpr uint32_t  RCC_CFGR3_UART4SW                   = 0x00300000;        /*!< UART4SW[1:0] bits */
constexpr uint32_t  RCC_CFGR3_UART4SW_0                 = 0x00100000;        /*!< Bit 0 */
constexpr uint32_t  RCC_CFGR3_UART4SW_1                 = 0x00200000;        /*!< Bit 1 */

constexpr uint32_t  RCC_CFGR3_UART4SW_PCLK              = 0x00000000;        /*!< PCLK2 clock used as UART4 clock source */
constexpr uint32_t  RCC_CFGR3_UART4SW_SYSCLK            = 0x00100000;        /*!< System clock selected as UART4 clock source */
constexpr uint32_t  RCC_CFGR3_UART4SW_LSE               = 0x00200000;        /*!< LSE oscillator clock used as UART4 clock source */
constexpr uint32_t  RCC_CFGR3_UART4SW_HSI               = 0x00300000;        /*!< HSI oscillator clock used as UART4 clock source */

constexpr uint32_t  RCC_CFGR3_UART5SW                   = 0x00C00000;        /*!< UART5SW[1:0] bits */
constexpr uint32_t  RCC_CFGR3_UART5SW_0                 = 0x00400000;        /*!< Bit 0 */
constexpr uint32_t  RCC_CFGR3_UART5SW_1                 = 0x00800000;        /*!< Bit 1 */

constexpr uint32_t  RCC_CFGR3_UART5SW_PCLK              = 0x00000000;        /*!< PCLK2 clock used as UART5 clock source */
constexpr uint32_t  RCC_CFGR3_UART5SW_SYSCLK            = 0x00400000;        /*!< System clock selected as UART5 clock source */
constexpr uint32_t  RCC_CFGR3_UART5SW_LSE               = 0x00800000;        /*!< LSE oscillator clock used as UART5 clock source */
constexpr uint32_t  RCC_CFGR3_UART5SW_HSI               = 0x00C00000;        /*!< HSI oscillator clock used as UART5 clock source */

constexpr uint32_t  RCC_CFGR3_TIM2SW_HCLK               = 0x00000000;        /*!< HCLK used as TIM2 clock source */
constexpr uint32_t  RCC_CFGR3_TIM2SW_PLL                = 0x01000000;        /*!< PLL clock used as TIM2 clock source */

constexpr uint32_t  RCC_CFGR3_TIM34SW_HCLK               = 0x00000000;        /*!< HCLK used as TIM3/TIM4 clock source */
constexpr uint32_t  RCC_CFGR3_TIM34SW_PLL                = 0x02000000;        /*!< PLL clock used as TIM3/TIM4 clock source */

/******************************************************************************/
/*                                                                            */
/*                           Real-Time Clock = RTC;                            */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for RTC_TR register  *******************/
constexpr uint32_t RTC_TR_PM                            = 0x00400000;
constexpr uint32_t RTC_TR_HT                            = 0x00300000;
constexpr uint32_t RTC_TR_HT_0                          = 0x00100000;
constexpr uint32_t RTC_TR_HT_1                          = 0x00200000;
constexpr uint32_t RTC_TR_HU                            = 0x000F0000;
constexpr uint32_t RTC_TR_HU_0                          = 0x00010000;
constexpr uint32_t RTC_TR_HU_1                          = 0x00020000;
constexpr uint32_t RTC_TR_HU_2                          = 0x00040000;
constexpr uint32_t RTC_TR_HU_3                          = 0x00080000;
constexpr uint32_t RTC_TR_MNT                           = 0x00007000;
constexpr uint32_t RTC_TR_MNT_0                         = 0x00001000;
constexpr uint32_t RTC_TR_MNT_1                         = 0x00002000;
constexpr uint32_t RTC_TR_MNT_2                         = 0x00004000;
constexpr uint32_t RTC_TR_MNU                           = 0x00000F00;
constexpr uint32_t RTC_TR_MNU_0                         = 0x00000100;
constexpr uint32_t RTC_TR_MNU_1                         = 0x00000200;
constexpr uint32_t RTC_TR_MNU_2                         = 0x00000400;
constexpr uint32_t RTC_TR_MNU_3                         = 0x00000800;
constexpr uint32_t RTC_TR_ST                            = 0x00000070;
constexpr uint32_t RTC_TR_ST_0                          = 0x00000010;
constexpr uint32_t RTC_TR_ST_1                          = 0x00000020;
constexpr uint32_t RTC_TR_ST_2                          = 0x00000040;
constexpr uint32_t RTC_TR_SU                            = 0x0000000F;
constexpr uint32_t RTC_TR_SU_0                          = 0x00000001;
constexpr uint32_t RTC_TR_SU_1                          = 0x00000002;
constexpr uint32_t RTC_TR_SU_2                          = 0x00000004;
constexpr uint32_t RTC_TR_SU_3                          = 0x00000008;

/********************  Bits definition for RTC_DR register  *******************/
constexpr uint32_t RTC_DR_YT                            = 0x00F00000;
constexpr uint32_t RTC_DR_YT_0                          = 0x00100000;
constexpr uint32_t RTC_DR_YT_1                          = 0x00200000;
constexpr uint32_t RTC_DR_YT_2                          = 0x00400000;
constexpr uint32_t RTC_DR_YT_3                          = 0x00800000;
constexpr uint32_t RTC_DR_YU                            = 0x000F0000;
constexpr uint32_t RTC_DR_YU_0                          = 0x00010000;
constexpr uint32_t RTC_DR_YU_1                          = 0x00020000;
constexpr uint32_t RTC_DR_YU_2                          = 0x00040000;
constexpr uint32_t RTC_DR_YU_3                          = 0x00080000;
constexpr uint32_t RTC_DR_WDU                           = 0x0000E000;
constexpr uint32_t RTC_DR_WDU_0                         = 0x00002000;
constexpr uint32_t RTC_DR_WDU_1                         = 0x00004000;
constexpr uint32_t RTC_DR_WDU_2                         = 0x00008000;
constexpr uint32_t RTC_DR_MT                            = 0x00001000;
constexpr uint32_t RTC_DR_MU                            = 0x00000F00;
constexpr uint32_t RTC_DR_MU_0                          = 0x00000100;
constexpr uint32_t RTC_DR_MU_1                          = 0x00000200;
constexpr uint32_t RTC_DR_MU_2                          = 0x00000400;
constexpr uint32_t RTC_DR_MU_3                          = 0x00000800;
constexpr uint32_t RTC_DR_DT                            = 0x00000030;
constexpr uint32_t RTC_DR_DT_0                          = 0x00000010;
constexpr uint32_t RTC_DR_DT_1                          = 0x00000020;
constexpr uint32_t RTC_DR_DU                            = 0x0000000F;
constexpr uint32_t RTC_DR_DU_0                          = 0x00000001;
constexpr uint32_t RTC_DR_DU_1                          = 0x00000002;
constexpr uint32_t RTC_DR_DU_2                          = 0x00000004;
constexpr uint32_t RTC_DR_DU_3                          = 0x00000008;

/********************  Bits definition for RTC_CR register  *******************/
constexpr uint32_t RTC_CR_COE                           = 0x00800000;
constexpr uint32_t RTC_CR_OSEL                          = 0x00600000;
constexpr uint32_t RTC_CR_OSEL_0                        = 0x00200000;
constexpr uint32_t RTC_CR_OSEL_1                        = 0x00400000;
constexpr uint32_t RTC_CR_POL                           = 0x00100000;
constexpr uint32_t RTC_CR_COSEL                         = 0x00080000;
constexpr uint32_t RTC_CR_BCK                           = 0x00040000;
constexpr uint32_t RTC_CR_SUB1H                         = 0x00020000;
constexpr uint32_t RTC_CR_ADD1H                         = 0x00010000;
constexpr uint32_t RTC_CR_TSIE                          = 0x00008000;
constexpr uint32_t RTC_CR_WUTIE                         = 0x00004000;
constexpr uint32_t RTC_CR_ALRBIE                        = 0x00002000;
constexpr uint32_t RTC_CR_ALRAIE                        = 0x00001000;
constexpr uint32_t RTC_CR_TSE                           = 0x00000800;
constexpr uint32_t RTC_CR_WUTE                          = 0x00000400;
constexpr uint32_t RTC_CR_ALRBE                         = 0x00000200;
constexpr uint32_t RTC_CR_ALRAE                         = 0x00000100;
constexpr uint32_t RTC_CR_FMT                           = 0x00000040;
constexpr uint32_t RTC_CR_BYPSHAD                       = 0x00000020;
constexpr uint32_t RTC_CR_REFCKON                       = 0x00000010;
constexpr uint32_t RTC_CR_TSEDGE                        = 0x00000008;
constexpr uint32_t RTC_CR_WUCKSEL                       = 0x00000007;
constexpr uint32_t RTC_CR_WUCKSEL_0                     = 0x00000001;
constexpr uint32_t RTC_CR_WUCKSEL_1                     = 0x00000002;
constexpr uint32_t RTC_CR_WUCKSEL_2                     = 0x00000004;

/********************  Bits definition for RTC_ISR register  ******************/
constexpr uint32_t RTC_ISR_RECALPF                      = 0x00010000;
constexpr uint32_t RTC_ISR_TAMP3F                       = 0x00008000;
constexpr uint32_t RTC_ISR_TAMP2F                       = 0x00004000;
constexpr uint32_t RTC_ISR_TAMP1F                       = 0x00002000;
constexpr uint32_t RTC_ISR_TSOVF                        = 0x00001000;
constexpr uint32_t RTC_ISR_TSF                          = 0x00000800;
constexpr uint32_t RTC_ISR_WUTF                         = 0x00000400;
constexpr uint32_t RTC_ISR_ALRBF                        = 0x00000200;
constexpr uint32_t RTC_ISR_ALRAF                        = 0x00000100;
constexpr uint32_t RTC_ISR_INIT                         = 0x00000080;
constexpr uint32_t RTC_ISR_INITF                        = 0x00000040;
constexpr uint32_t RTC_ISR_RSF                          = 0x00000020;
constexpr uint32_t RTC_ISR_INITS                        = 0x00000010;
constexpr uint32_t RTC_ISR_SHPF                         = 0x00000008;
constexpr uint32_t RTC_ISR_WUTWF                        = 0x00000004;
constexpr uint32_t RTC_ISR_ALRBWF                       = 0x00000002;
constexpr uint32_t RTC_ISR_ALRAWF                       = 0x00000001;

/********************  Bits definition for RTC_PRER register  *****************/
constexpr uint32_t RTC_PRER_PREDIV_A                    = 0x007F0000;
constexpr uint32_t RTC_PRER_PREDIV_S                    = 0x00007FFF;

/********************  Bits definition for RTC_WUTR register  *****************/
constexpr uint32_t RTC_WUTR_WUT                         = 0x0000FFFF;

/********************  Bits definition for RTC_ALRMAR register  ***************/
constexpr uint32_t RTC_ALRMAR_MSK4                      = 0x80000000;
constexpr uint32_t RTC_ALRMAR_WDSEL                     = 0x40000000;
constexpr uint32_t RTC_ALRMAR_DT                        = 0x30000000;
constexpr uint32_t RTC_ALRMAR_DT_0                      = 0x10000000;
constexpr uint32_t RTC_ALRMAR_DT_1                      = 0x20000000;
constexpr uint32_t RTC_ALRMAR_DU                        = 0x0F000000;
constexpr uint32_t RTC_ALRMAR_DU_0                      = 0x01000000;
constexpr uint32_t RTC_ALRMAR_DU_1                      = 0x02000000;
constexpr uint32_t RTC_ALRMAR_DU_2                      = 0x04000000;
constexpr uint32_t RTC_ALRMAR_DU_3                      = 0x08000000;
constexpr uint32_t RTC_ALRMAR_MSK3                      = 0x00800000;
constexpr uint32_t RTC_ALRMAR_PM                        = 0x00400000;
constexpr uint32_t RTC_ALRMAR_HT                        = 0x00300000;
constexpr uint32_t RTC_ALRMAR_HT_0                      = 0x00100000;
constexpr uint32_t RTC_ALRMAR_HT_1                      = 0x00200000;
constexpr uint32_t RTC_ALRMAR_HU                        = 0x000F0000;
constexpr uint32_t RTC_ALRMAR_HU_0                      = 0x00010000;
constexpr uint32_t RTC_ALRMAR_HU_1                      = 0x00020000;
constexpr uint32_t RTC_ALRMAR_HU_2                      = 0x00040000;
constexpr uint32_t RTC_ALRMAR_HU_3                      = 0x00080000;
constexpr uint32_t RTC_ALRMAR_MSK2                      = 0x00008000;
constexpr uint32_t RTC_ALRMAR_MNT                       = 0x00007000;
constexpr uint32_t RTC_ALRMAR_MNT_0                     = 0x00001000;
constexpr uint32_t RTC_ALRMAR_MNT_1                     = 0x00002000;
constexpr uint32_t RTC_ALRMAR_MNT_2                     = 0x00004000;
constexpr uint32_t RTC_ALRMAR_MNU                       = 0x00000F00;
constexpr uint32_t RTC_ALRMAR_MNU_0                     = 0x00000100;
constexpr uint32_t RTC_ALRMAR_MNU_1                     = 0x00000200;
constexpr uint32_t RTC_ALRMAR_MNU_2                     = 0x00000400;
constexpr uint32_t RTC_ALRMAR_MNU_3                     = 0x00000800;
constexpr uint32_t RTC_ALRMAR_MSK1                      = 0x00000080;
constexpr uint32_t RTC_ALRMAR_ST                        = 0x00000070;
constexpr uint32_t RTC_ALRMAR_ST_0                      = 0x00000010;
constexpr uint32_t RTC_ALRMAR_ST_1                      = 0x00000020;
constexpr uint32_t RTC_ALRMAR_ST_2                      = 0x00000040;
constexpr uint32_t RTC_ALRMAR_SU                        = 0x0000000F;
constexpr uint32_t RTC_ALRMAR_SU_0                      = 0x00000001;
constexpr uint32_t RTC_ALRMAR_SU_1                      = 0x00000002;
constexpr uint32_t RTC_ALRMAR_SU_2                      = 0x00000004;
constexpr uint32_t RTC_ALRMAR_SU_3                      = 0x00000008;

/********************  Bits definition for RTC_ALRMBR register  ***************/
constexpr uint32_t RTC_ALRMBR_MSK4                      = 0x80000000;
constexpr uint32_t RTC_ALRMBR_WDSEL                     = 0x40000000;
constexpr uint32_t RTC_ALRMBR_DT                        = 0x30000000;
constexpr uint32_t RTC_ALRMBR_DT_0                      = 0x10000000;
constexpr uint32_t RTC_ALRMBR_DT_1                      = 0x20000000;
constexpr uint32_t RTC_ALRMBR_DU                        = 0x0F000000;
constexpr uint32_t RTC_ALRMBR_DU_0                      = 0x01000000;
constexpr uint32_t RTC_ALRMBR_DU_1                      = 0x02000000;
constexpr uint32_t RTC_ALRMBR_DU_2                      = 0x04000000;
constexpr uint32_t RTC_ALRMBR_DU_3                      = 0x08000000;
constexpr uint32_t RTC_ALRMBR_MSK3                      = 0x00800000;
constexpr uint32_t RTC_ALRMBR_PM                        = 0x00400000;
constexpr uint32_t RTC_ALRMBR_HT                        = 0x00300000;
constexpr uint32_t RTC_ALRMBR_HT_0                      = 0x00100000;
constexpr uint32_t RTC_ALRMBR_HT_1                      = 0x00200000;
constexpr uint32_t RTC_ALRMBR_HU                        = 0x000F0000;
constexpr uint32_t RTC_ALRMBR_HU_0                      = 0x00010000;
constexpr uint32_t RTC_ALRMBR_HU_1                      = 0x00020000;
constexpr uint32_t RTC_ALRMBR_HU_2                      = 0x00040000;
constexpr uint32_t RTC_ALRMBR_HU_3                      = 0x00080000;
constexpr uint32_t RTC_ALRMBR_MSK2                      = 0x00008000;
constexpr uint32_t RTC_ALRMBR_MNT                       = 0x00007000;
constexpr uint32_t RTC_ALRMBR_MNT_0                     = 0x00001000;
constexpr uint32_t RTC_ALRMBR_MNT_1                     = 0x00002000;
constexpr uint32_t RTC_ALRMBR_MNT_2                     = 0x00004000;
constexpr uint32_t RTC_ALRMBR_MNU                       = 0x00000F00;
constexpr uint32_t RTC_ALRMBR_MNU_0                     = 0x00000100;
constexpr uint32_t RTC_ALRMBR_MNU_1                     = 0x00000200;
constexpr uint32_t RTC_ALRMBR_MNU_2                     = 0x00000400;
constexpr uint32_t RTC_ALRMBR_MNU_3                     = 0x00000800;
constexpr uint32_t RTC_ALRMBR_MSK1                      = 0x00000080;
constexpr uint32_t RTC_ALRMBR_ST                        = 0x00000070;
constexpr uint32_t RTC_ALRMBR_ST_0                      = 0x00000010;
constexpr uint32_t RTC_ALRMBR_ST_1                      = 0x00000020;
constexpr uint32_t RTC_ALRMBR_ST_2                      = 0x00000040;
constexpr uint32_t RTC_ALRMBR_SU                        = 0x0000000F;
constexpr uint32_t RTC_ALRMBR_SU_0                      = 0x00000001;
constexpr uint32_t RTC_ALRMBR_SU_1                      = 0x00000002;
constexpr uint32_t RTC_ALRMBR_SU_2                      = 0x00000004;
constexpr uint32_t RTC_ALRMBR_SU_3                      = 0x00000008;

/********************  Bits definition for RTC_WPR register  ******************/
constexpr uint32_t RTC_WPR_KEY                          = 0x000000FF;

/********************  Bits definition for RTC_SSR register  ******************/
constexpr uint32_t RTC_SSR_SS                           = 0x0000FFFF;

/********************  Bits definition for RTC_SHIFTR register  ***************/
constexpr uint32_t RTC_SHIFTR_SUBFS                     = 0x00007FFF;
constexpr uint32_t RTC_SHIFTR_ADD1S                     = 0x80000000;

/********************  Bits definition for RTC_TSTR register  *****************/
constexpr uint32_t RTC_TSTR_PM                          = 0x00400000;
constexpr uint32_t RTC_TSTR_HT                          = 0x00300000;
constexpr uint32_t RTC_TSTR_HT_0                        = 0x00100000;
constexpr uint32_t RTC_TSTR_HT_1                        = 0x00200000;
constexpr uint32_t RTC_TSTR_HU                          = 0x000F0000;
constexpr uint32_t RTC_TSTR_HU_0                        = 0x00010000;
constexpr uint32_t RTC_TSTR_HU_1                        = 0x00020000;
constexpr uint32_t RTC_TSTR_HU_2                        = 0x00040000;
constexpr uint32_t RTC_TSTR_HU_3                        = 0x00080000;
constexpr uint32_t RTC_TSTR_MNT                         = 0x00007000;
constexpr uint32_t RTC_TSTR_MNT_0                       = 0x00001000;
constexpr uint32_t RTC_TSTR_MNT_1                       = 0x00002000;
constexpr uint32_t RTC_TSTR_MNT_2                       = 0x00004000;
constexpr uint32_t RTC_TSTR_MNU                         = 0x00000F00;
constexpr uint32_t RTC_TSTR_MNU_0                       = 0x00000100;
constexpr uint32_t RTC_TSTR_MNU_1                       = 0x00000200;
constexpr uint32_t RTC_TSTR_MNU_2                       = 0x00000400;
constexpr uint32_t RTC_TSTR_MNU_3                       = 0x00000800;
constexpr uint32_t RTC_TSTR_ST                          = 0x00000070;
constexpr uint32_t RTC_TSTR_ST_0                        = 0x00000010;
constexpr uint32_t RTC_TSTR_ST_1                        = 0x00000020;
constexpr uint32_t RTC_TSTR_ST_2                        = 0x00000040;
constexpr uint32_t RTC_TSTR_SU                          = 0x0000000F;
constexpr uint32_t RTC_TSTR_SU_0                        = 0x00000001;
constexpr uint32_t RTC_TSTR_SU_1                        = 0x00000002;
constexpr uint32_t RTC_TSTR_SU_2                        = 0x00000004;
constexpr uint32_t RTC_TSTR_SU_3                        = 0x00000008;

/********************  Bits definition for RTC_TSDR register  *****************/
constexpr uint32_t RTC_TSDR_WDU                         = 0x0000E000;
constexpr uint32_t RTC_TSDR_WDU_0                       = 0x00002000;
constexpr uint32_t RTC_TSDR_WDU_1                       = 0x00004000;
constexpr uint32_t RTC_TSDR_WDU_2                       = 0x00008000;
constexpr uint32_t RTC_TSDR_MT                          = 0x00001000;
constexpr uint32_t RTC_TSDR_MU                          = 0x00000F00;
constexpr uint32_t RTC_TSDR_MU_0                        = 0x00000100;
constexpr uint32_t RTC_TSDR_MU_1                        = 0x00000200;
constexpr uint32_t RTC_TSDR_MU_2                        = 0x00000400;
constexpr uint32_t RTC_TSDR_MU_3                        = 0x00000800;
constexpr uint32_t RTC_TSDR_DT                          = 0x00000030;
constexpr uint32_t RTC_TSDR_DT_0                        = 0x00000010;
constexpr uint32_t RTC_TSDR_DT_1                        = 0x00000020;
constexpr uint32_t RTC_TSDR_DU                          = 0x0000000F;
constexpr uint32_t RTC_TSDR_DU_0                        = 0x00000001;
constexpr uint32_t RTC_TSDR_DU_1                        = 0x00000002;
constexpr uint32_t RTC_TSDR_DU_2                        = 0x00000004;
constexpr uint32_t RTC_TSDR_DU_3                        = 0x00000008;

/********************  Bits definition for RTC_TSSSR register  ****************/
constexpr uint32_t RTC_TSSSR_SS                         = 0x0000FFFF;

/********************  Bits definition for RTC_CAL register  *****************/
constexpr uint32_t RTC_CALR_CALP                        = 0x00008000;
constexpr uint32_t RTC_CALR_CALW8                       = 0x00004000;
constexpr uint32_t RTC_CALR_CALW16                      = 0x00002000;
constexpr uint32_t RTC_CALR_CALM                        = 0x000001FF;
constexpr uint32_t RTC_CALR_CALM_0                      = 0x00000001;
constexpr uint32_t RTC_CALR_CALM_1                      = 0x00000002;
constexpr uint32_t RTC_CALR_CALM_2                      = 0x00000004;
constexpr uint32_t RTC_CALR_CALM_3                      = 0x00000008;
constexpr uint32_t RTC_CALR_CALM_4                      = 0x00000010;
constexpr uint32_t RTC_CALR_CALM_5                      = 0x00000020;
constexpr uint32_t RTC_CALR_CALM_6                      = 0x00000040;
constexpr uint32_t RTC_CALR_CALM_7                      = 0x00000080;
constexpr uint32_t RTC_CALR_CALM_8                      = 0x00000100;

/********************  Bits definition for RTC_TAFCR register  ****************/
constexpr uint32_t RTC_TAFCR_ALARMOUTTYPE               = 0x00040000;
constexpr uint32_t RTC_TAFCR_TAMPPUDIS                  = 0x00008000;
constexpr uint32_t RTC_TAFCR_TAMPPRCH                   = 0x00006000;
constexpr uint32_t RTC_TAFCR_TAMPPRCH_0                 = 0x00002000;
constexpr uint32_t RTC_TAFCR_TAMPPRCH_1                 = 0x00004000;
constexpr uint32_t RTC_TAFCR_TAMPFLT                    = 0x00001800;
constexpr uint32_t RTC_TAFCR_TAMPFLT_0                  = 0x00000800;
constexpr uint32_t RTC_TAFCR_TAMPFLT_1                  = 0x00001000;
constexpr uint32_t RTC_TAFCR_TAMPFREQ                   = 0x00000700;
constexpr uint32_t RTC_TAFCR_TAMPFREQ_0                 = 0x00000100;
constexpr uint32_t RTC_TAFCR_TAMPFREQ_1                 = 0x00000200;
constexpr uint32_t RTC_TAFCR_TAMPFREQ_2                 = 0x00000400;
constexpr uint32_t RTC_TAFCR_TAMPTS                     = 0x00000080;
constexpr uint32_t RTC_TAFCR_TAMP3TRG                   = 0x00000040;
constexpr uint32_t RTC_TAFCR_TAMP3E                     = 0x00000020;
constexpr uint32_t RTC_TAFCR_TAMP2TRG                   = 0x00000010;
constexpr uint32_t RTC_TAFCR_TAMP2E                     = 0x00000008;
constexpr uint32_t RTC_TAFCR_TAMPIE                     = 0x00000004;
constexpr uint32_t RTC_TAFCR_TAMP1TRG                   = 0x00000002;
constexpr uint32_t RTC_TAFCR_TAMP1E                     = 0x00000001;

/********************  Bits definition for RTC_ALRMASSR register  *************/
constexpr uint32_t RTC_ALRMASSR_MASKSS                  = 0x0F000000;
constexpr uint32_t RTC_ALRMASSR_MASKSS_0                = 0x01000000;
constexpr uint32_t RTC_ALRMASSR_MASKSS_1                = 0x02000000;
constexpr uint32_t RTC_ALRMASSR_MASKSS_2                = 0x04000000;
constexpr uint32_t RTC_ALRMASSR_MASKSS_3                = 0x08000000;
constexpr uint32_t RTC_ALRMASSR_SS                      = 0x00007FFF;

/********************  Bits definition for RTC_ALRMBSSR register  *************/
constexpr uint32_t RTC_ALRMBSSR_MASKSS                  = 0x0F000000;
constexpr uint32_t RTC_ALRMBSSR_MASKSS_0                = 0x01000000;
constexpr uint32_t RTC_ALRMBSSR_MASKSS_1                = 0x02000000;
constexpr uint32_t RTC_ALRMBSSR_MASKSS_2                = 0x04000000;
constexpr uint32_t RTC_ALRMBSSR_MASKSS_3                = 0x08000000;
constexpr uint32_t RTC_ALRMBSSR_SS                      = 0x00007FFF;

/********************  Bits definition for RTC_BKP0R register  ****************/
constexpr uint32_t RTC_BKP0R                            = 0xFFFFFFFF;

/********************  Bits definition for RTC_BKP1R register  ****************/
constexpr uint32_t RTC_BKP1R                            = 0xFFFFFFFF;

/********************  Bits definition for RTC_BKP2R register  ****************/
constexpr uint32_t RTC_BKP2R                            = 0xFFFFFFFF;

/********************  Bits definition for RTC_BKP3R register  ****************/
constexpr uint32_t RTC_BKP3R                            = 0xFFFFFFFF;

/********************  Bits definition for RTC_BKP4R register  ****************/
constexpr uint32_t RTC_BKP4R                            = 0xFFFFFFFF;

/********************  Bits definition for RTC_BKP5R register  ****************/
constexpr uint32_t RTC_BKP5R                            = 0xFFFFFFFF;

/********************  Bits definition for RTC_BKP6R register  ****************/
constexpr uint32_t RTC_BKP6R                            = 0xFFFFFFFF;

/********************  Bits definition for RTC_BKP7R register  ****************/
constexpr uint32_t RTC_BKP7R                            = 0xFFFFFFFF;

/********************  Bits definition for RTC_BKP8R register  ****************/
constexpr uint32_t RTC_BKP8R                            = 0xFFFFFFFF;

/********************  Bits definition for RTC_BKP9R register  ****************/
constexpr uint32_t RTC_BKP9R                            = 0xFFFFFFFF;

/********************  Bits definition for RTC_BKP10R register  ***************/
constexpr uint32_t RTC_BKP10R                           = 0xFFFFFFFF;

/********************  Bits definition for RTC_BKP11R register  ***************/
constexpr uint32_t RTC_BKP11R                           = 0xFFFFFFFF;

/********************  Bits definition for RTC_BKP12R register  ***************/
constexpr uint32_t RTC_BKP12R                           = 0xFFFFFFFF;

/********************  Bits definition for RTC_BKP13R register  ***************/
constexpr uint32_t RTC_BKP13R                           = 0xFFFFFFFF;

/********************  Bits definition for RTC_BKP14R register  ***************/
constexpr uint32_t RTC_BKP14R                           = 0xFFFFFFFF;

/********************  Bits definition for RTC_BKP15R register  ***************/
constexpr uint32_t RTC_BKP15R                           = 0xFFFFFFFF;

/******************** Number of backup registers ******************************/
constexpr uint32_t RTC_BKP_NUMBER                       = 0x00000010;

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface = SPI;                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for SPI_CR1 register  ********************/
constexpr uint32_t  SPI_CR1_CPHA                        = 0x00000001;        /*!< Clock Phase */
constexpr uint32_t  SPI_CR1_CPOL                        = 0x00000002;        /*!< Clock Polarity */
constexpr uint32_t  SPI_CR1_MSTR                        = 0x00000004;        /*!< Master Selection */
constexpr uint32_t  SPI_CR1_BR                          = 0x00000038;        /*!< BR[2:0] bits = Baud Rate Control; */
constexpr uint32_t  SPI_CR1_BR_0                        = 0x00000008;        /*!< Bit 0 */
constexpr uint32_t  SPI_CR1_BR_1                        = 0x00000010;        /*!< Bit 1 */
constexpr uint32_t  SPI_CR1_BR_2                        = 0x00000020;        /*!< Bit 2 */
constexpr uint32_t  SPI_CR1_SPE                         = 0x00000040;        /*!< SPI Enable */
constexpr uint32_t  SPI_CR1_LSBFIRST                    = 0x00000080;        /*!< Frame Format */
constexpr uint32_t  SPI_CR1_SSI                         = 0x00000100;        /*!< Internal slave select */
constexpr uint32_t  SPI_CR1_SSM                         = 0x00000200;        /*!< Software slave management */
constexpr uint32_t  SPI_CR1_RXONLY                      = 0x00000400;        /*!< Receive only */
constexpr uint32_t  SPI_CR1_CRCL                        = 0x00000800;        /*!< CRC Length */
constexpr uint32_t  SPI_CR1_CRCNEXT                     = 0x00001000;        /*!< Transmit CRC next */
constexpr uint32_t  SPI_CR1_CRCEN                       = 0x00002000;        /*!< Hardware CRC calculation enable */
constexpr uint32_t  SPI_CR1_BIDIOE                      = 0x00004000;        /*!< Output enable in bidirectional mode */
constexpr uint32_t  SPI_CR1_BIDIMODE                    = 0x00008000;        /*!< Bidirectional data mode enable */

/*******************  Bit definition for SPI_CR2 register  ********************/
constexpr uint32_t  SPI_CR2_RXDMAEN                     = 0x00000001;        /*!< Rx Buffer DMA Enable */
constexpr uint32_t  SPI_CR2_TXDMAEN                     = 0x00000002;        /*!< Tx Buffer DMA Enable */
constexpr uint32_t  SPI_CR2_SSOE                        = 0x00000004;        /*!< SS Output Enable */
constexpr uint32_t  SPI_CR2_NSSP                        = 0x00000008;        /*!< NSS pulse management Enable */
constexpr uint32_t  SPI_CR2_FRF                         = 0x00000010;        /*!< Frame Format Enable */
constexpr uint32_t  SPI_CR2_ERRIE                       = 0x00000020;        /*!< Error Interrupt Enable */
constexpr uint32_t  SPI_CR2_RXNEIE                      = 0x00000040;        /*!< RX buffer Not Empty Interrupt Enable */
constexpr uint32_t  SPI_CR2_TXEIE                       = 0x00000080;        /*!< Tx buffer Empty Interrupt Enable */
constexpr uint32_t  SPI_CR2_DS                          = 0x00000F00;        /*!< DS[3:0] Data Size */
constexpr uint32_t  SPI_CR2_DS_0                        = 0x00000100;        /*!< Bit 0 */
constexpr uint32_t  SPI_CR2_DS_1                        = 0x00000200;        /*!< Bit 1 */
constexpr uint32_t  SPI_CR2_DS_2                        = 0x00000400;        /*!< Bit 2 */
constexpr uint32_t  SPI_CR2_DS_3                        = 0x00000800;        /*!< Bit 3 */
constexpr uint32_t  SPI_CR2_FRXTH                       = 0x00001000;        /*!< FIFO reception Threshold */
constexpr uint32_t  SPI_CR2_LDMARX                      = 0x00002000;        /*!< Last DMA transfer for reception */
constexpr uint32_t  SPI_CR2_LDMATX                      = 0x00004000;        /*!< Last DMA transfer for transmission */

/********************  Bit definition for SPI_SR register  ********************/
constexpr uint32_t  SPI_SR_RXNE                         = 0x00000001;        /*!< Receive buffer Not Empty */
constexpr uint32_t  SPI_SR_TXE                          = 0x00000002;        /*!< Transmit buffer Empty */
constexpr uint32_t  SPI_SR_CHSIDE                       = 0x00000004;        /*!< Channel side */
constexpr uint32_t  SPI_SR_UDR                          = 0x00000008;        /*!< Underrun flag */
constexpr uint32_t  SPI_SR_CRCERR                       = 0x00000010;        /*!< CRC Error flag */
constexpr uint32_t  SPI_SR_MODF                         = 0x00000020;        /*!< Mode fault */
constexpr uint32_t  SPI_SR_OVR                          = 0x00000040;        /*!< Overrun flag */
constexpr uint32_t  SPI_SR_BSY                          = 0x00000080;        /*!< Busy flag */
constexpr uint32_t  SPI_SR_FRE                          = 0x00000100;        /*!< TI frame format error */
constexpr uint32_t  SPI_SR_FRLVL                        = 0x00000600;        /*!< FIFO Reception Level */
constexpr uint32_t  SPI_SR_FRLVL_0                      = 0x00000200;        /*!< Bit 0 */
constexpr uint32_t  SPI_SR_FRLVL_1                      = 0x00000400;        /*!< Bit 1 */
constexpr uint32_t  SPI_SR_FTLVL                        = 0x00001800;        /*!< FIFO Transmission Level */
constexpr uint32_t  SPI_SR_FTLVL_0                      = 0x00000800;        /*!< Bit 0 */
constexpr uint32_t  SPI_SR_FTLVL_1                      = 0x00001000;        /*!< Bit 1 */

/********************  Bit definition for SPI_DR register  ********************/
constexpr uint32_t  SPI_DR_DR                           = 0x0000FFFF;        /*!< Data Register */

/*******************  Bit definition for SPI_CRCPR register  ******************/
constexpr uint32_t  SPI_CRCPR_CRCPOLY                   = 0x0000FFFF;        /*!< CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
constexpr uint32_t  SPI_RXCRCR_RXCRC                    = 0x0000FFFF;        /*!< Rx CRC Register */

/******************  Bit definition for SPI_TXCRCR register  ******************/
constexpr uint32_t  SPI_TXCRCR_TXCRC                    = 0x0000FFFF;        /*!< Tx CRC Register */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
constexpr uint32_t  SPI_I2SCFGR_CHLEN                   = 0x00000001;        /*!<Channel length = number of bits per audio channel; */
constexpr uint32_t  SPI_I2SCFGR_DATLEN                  = 0x00000006;        /*!<DATLEN[1:0] bits = Data length to be transferred; */
constexpr uint32_t  SPI_I2SCFGR_DATLEN_0                = 0x00000002;        /*!<Bit 0 */
constexpr uint32_t  SPI_I2SCFGR_DATLEN_1                = 0x00000004;        /*!<Bit 1 */
constexpr uint32_t  SPI_I2SCFGR_CKPOL                   = 0x00000008;        /*!<steady state clock polarity */
constexpr uint32_t  SPI_I2SCFGR_I2SSTD                  = 0x00000030;        /*!<I2SSTD[1:0] bits = I2S standard selection; */
constexpr uint32_t  SPI_I2SCFGR_I2SSTD_0                = 0x00000010;        /*!<Bit 0 */
constexpr uint32_t  SPI_I2SCFGR_I2SSTD_1                = 0x00000020;        /*!<Bit 1 */
constexpr uint32_t  SPI_I2SCFGR_PCMSYNC                 = 0x00000080;        /*!<PCM frame synchronization */
constexpr uint32_t  SPI_I2SCFGR_I2SCFG                  = 0x00000300;        /*!<I2SCFG[1:0] bits = I2S configuration mode; */
constexpr uint32_t  SPI_I2SCFGR_I2SCFG_0                = 0x00000100;        /*!<Bit 0 */
constexpr uint32_t  SPI_I2SCFGR_I2SCFG_1                = 0x00000200;        /*!<Bit 1 */
constexpr uint32_t  SPI_I2SCFGR_I2SE                    = 0x00000400;        /*!<I2S Enable */
constexpr uint32_t  SPI_I2SCFGR_I2SMOD                  = 0x00000800;        /*!<I2S mode selection */

/******************  Bit definition for SPI_I2SPR register  *******************/
constexpr uint32_t  SPI_I2SPR_I2SDIV                    = 0x000000FF;        /*!<I2S Linear prescaler */
constexpr uint32_t  SPI_I2SPR_ODD                       = 0x00000100;        /*!<Odd factor for the prescaler */
constexpr uint32_t  SPI_I2SPR_MCKOE                     = 0x00000200;        /*!<Master Clock Output Enable */

/******************************************************************************/
/*                                                                            */
/*                        System Configuration= SYSCFG;                        */
/*                                                                            */
/******************************************************************************/
/*****************  Bit definition for SYSCFG_CFGR1 register  *****************/
constexpr uint32_t SYSCFG_CFGR1_MEM_MODE               = 0x00000007; /*!< SYSCFG_Memory Remap Config */
constexpr uint32_t SYSCFG_CFGR1_MEM_MODE_0             = 0x00000001; /*!< Bit 0 */
constexpr uint32_t SYSCFG_CFGR1_MEM_MODE_1             = 0x00000002; /*!< Bit 1 */
constexpr uint32_t SYSCFG_CFGR1_MEM_MODE_2             = 0x00000004; /*!< Bit 1 */
constexpr uint32_t SYSCFG_CFGR1_USB_IT_RMP             = 0x00000020; /*!< USB interrupt remap */
constexpr uint32_t SYSCFG_CFGR1_TIM1_ITR3_RMP          = 0x00000040; /*!< Timer 1 ITR3 selection */
constexpr uint32_t SYSCFG_CFGR1_DAC1_TRIG1_RMP         = 0x00000080; /*!< DAC1 Trigger1 remap */
constexpr uint32_t SYSCFG_CFGR1_DMA_RMP                = 0x00007900; /*!< DMA remap mask */
constexpr uint32_t SYSCFG_CFGR1_ADC24_DMA_RMP          = 0x00000100; /*!< ADC2 and ADC4 DMA remap */
constexpr uint32_t SYSCFG_CFGR1_TIM16_DMA_RMP          = 0x00000800; /*!< Timer 16 DMA remap */
constexpr uint32_t SYSCFG_CFGR1_TIM17_DMA_RMP          = 0x00001000; /*!< Timer 17 DMA remap */
constexpr uint32_t SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP    = 0x00002000; /*!< Timer 6 / DAC1 CH1 DMA remap */
constexpr uint32_t SYSCFG_CFGR1_TIM7DAC1Ch2_DMA_RMP    = 0x00004000; /*!< Timer 7 / DAC1 CH2 DMA remap */
constexpr uint32_t SYSCFG_CFGR1_I2C_PB6_FMP            = 0x00010000; /*!< I2C PB6 Fast mode plus */
constexpr uint32_t SYSCFG_CFGR1_I2C_PB7_FMP            = 0x00020000; /*!< I2C PB7 Fast mode plus */
constexpr uint32_t SYSCFG_CFGR1_I2C_PB8_FMP            = 0x00040000; /*!< I2C PB8 Fast mode plus */
constexpr uint32_t SYSCFG_CFGR1_I2C_PB9_FMP            = 0x00080000; /*!< I2C PB9 Fast mode plus */
constexpr uint32_t SYSCFG_CFGR1_I2C1_FMP               = 0x00100000; /*!< I2C1 Fast mode plus */
constexpr uint32_t SYSCFG_CFGR1_I2C2_FMP               = 0x00200000; /*!< I2C2 Fast mode plus */
constexpr uint32_t SYSCFG_CFGR1_ENCODER_MODE           = 0x00C00000; /*!< Encoder Mode */
constexpr uint32_t SYSCFG_CFGR1_ENCODER_MODE_0         = 0x00400000; /*!< Encoder Mode 0 */
constexpr uint32_t SYSCFG_CFGR1_ENCODER_MODE_1         = 0x00800000; /*!< Encoder Mode 1 */
constexpr uint32_t SYSCFG_CFGR1_I2C3_FMP               = 0x01000000; /*!< I2C3 Fast mode plus */
constexpr uint32_t SYSCFG_CFGR1_FPU_IE                 = 0xFC000000; /*!< Floating Point Unit Interrupt Enable */
constexpr uint32_t SYSCFG_CFGR1_FPU_IE_0               = 0x04000000; /*!< Floating Point Unit Interrupt Enable 0 */
constexpr uint32_t SYSCFG_CFGR1_FPU_IE_1               = 0x08000000; /*!< Floating Point Unit Interrupt Enable 1 */
constexpr uint32_t SYSCFG_CFGR1_FPU_IE_2               = 0x10000000; /*!< Floating Point Unit Interrupt Enable 2 */
constexpr uint32_t SYSCFG_CFGR1_FPU_IE_3               = 0x20000000; /*!< Floating Point Unit Interrupt Enable 3 */
constexpr uint32_t SYSCFG_CFGR1_FPU_IE_4               = 0x40000000; /*!< Floating Point Unit Interrupt Enable 4 */
constexpr uint32_t SYSCFG_CFGR1_FPU_IE_5               = 0x80000000; /*!< Floating Point Unit Interrupt Enable 5 */

/*****************  Bit definition for SYSCFG_RCR register  *******************/
constexpr uint32_t SYSCFG_RCR_PAGE0          = 0x00000001; /*!< ICODE SRAM Write protection page 0 */
constexpr uint32_t SYSCFG_RCR_PAGE1          = 0x00000002; /*!< ICODE SRAM Write protection page 1 */
constexpr uint32_t SYSCFG_RCR_PAGE2          = 0x00000004; /*!< ICODE SRAM Write protection page 2 */
constexpr uint32_t SYSCFG_RCR_PAGE3          = 0x00000008; /*!< ICODE SRAM Write protection page 3 */
constexpr uint32_t SYSCFG_RCR_PAGE4          = 0x00000010; /*!< ICODE SRAM Write protection page 4 */
constexpr uint32_t SYSCFG_RCR_PAGE5          = 0x00000020; /*!< ICODE SRAM Write protection page 5 */
constexpr uint32_t SYSCFG_RCR_PAGE6          = 0x00000040; /*!< ICODE SRAM Write protection page 6 */
constexpr uint32_t SYSCFG_RCR_PAGE7          = 0x00000080; /*!< ICODE SRAM Write protection page 7 */
constexpr uint32_t SYSCFG_RCR_PAGE8          = 0x00000100; /*!< ICODE SRAM Write protection page 8 */
constexpr uint32_t SYSCFG_RCR_PAGE9          = 0x00000200; /*!< ICODE SRAM Write protection page 9 */
constexpr uint32_t SYSCFG_RCR_PAGE10         = 0x00000400; /*!< ICODE SRAM Write protection page 10 */
constexpr uint32_t SYSCFG_RCR_PAGE11         = 0x00000800; /*!< ICODE SRAM Write protection page 11 */
constexpr uint32_t SYSCFG_RCR_PAGE12         = 0x00001000; /*!< ICODE SRAM Write protection page 12 */
constexpr uint32_t SYSCFG_RCR_PAGE13         = 0x00002000; /*!< ICODE SRAM Write protection page 13 */
constexpr uint32_t SYSCFG_RCR_PAGE14         = 0x00004000; /*!< ICODE SRAM Write protection page 14 */
constexpr uint32_t SYSCFG_RCR_PAGE15         = 0x00008000; /*!< ICODE SRAM Write protection page 15 */

/*****************  Bit definition for SYSCFG_EXTICR1 register  ***************/
constexpr uint32_t SYSCFG_EXTICR1_EXTI0            = 0x0000000F; /*!< EXTI 0 configuration */
constexpr uint32_t SYSCFG_EXTICR1_EXTI1            = 0x000000F0; /*!< EXTI 1 configuration */
constexpr uint32_t SYSCFG_EXTICR1_EXTI2            = 0x00000F00; /*!< EXTI 2 configuration */
constexpr uint32_t SYSCFG_EXTICR1_EXTI3            = 0x0000F000; /*!< EXTI 3 configuration */

/*!<*
  * @brief  EXTI0 configuration
  */
constexpr uint32_t SYSCFG_EXTICR1_EXTI0_PA         = 0x00000000; /*!< PA[0] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI0_PB         = 0x00000001; /*!< PB[0] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI0_PC         = 0x00000002; /*!< PC[0] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI0_PD         = 0x00000003; /*!< PD[0] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI0_PE         = 0x00000004; /*!< PE[0] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI0_PF         = 0x00000005; /*!< PF[0] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI0_PG         = 0x00000006; /*!< PG[0] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI0_PH         = 0x00000007; /*!< PH[0] pin */

/*!<*
  * @brief  EXTI1 configuration
  */
constexpr uint32_t SYSCFG_EXTICR1_EXTI1_PA         = 0x00000000; /*!< PA[1] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI1_PB         = 0x00000010; /*!< PB[1] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI1_PC         = 0x00000020; /*!< PC[1] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI1_PD         = 0x00000030; /*!< PD[1] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI1_PE         = 0x00000040; /*!< PE[1] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI1_PF         = 0x00000050; /*!< PF[1] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI1_PG         = 0x00000060; /*!< PG[1] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI1_PH         = 0x00000070; /*!< PH[1] pin */

/*!<*
  * @brief  EXTI2 configuration
  */
constexpr uint32_t SYSCFG_EXTICR1_EXTI2_PA         = 0x00000000; /*!< PA[2] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI2_PB         = 0x00000100; /*!< PB[2] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI2_PC         = 0x00000200; /*!< PC[2] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI2_PD         = 0x00000300; /*!< PD[2] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI2_PE         = 0x00000400; /*!< PE[2] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI2_PF         = 0x00000500; /*!< PF[2] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI2_PG         = 0x00000600; /*!< PG[2] pin */

/*!<*
  * @brief  EXTI3 configuration
  */
constexpr uint32_t SYSCFG_EXTICR1_EXTI3_PA         = 0x00000000; /*!< PA[3] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI3_PB         = 0x00001000; /*!< PB[3] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI3_PC         = 0x00002000; /*!< PC[3] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI3_PD         = 0x00003000; /*!< PD[3] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI3_PE         = 0x00004000; /*!< PE[3] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI3_PF         = 0x00005000; /*!< PE[3] pin */
constexpr uint32_t SYSCFG_EXTICR1_EXTI3_PG         = 0x00006000; /*!< PG[3] pin */

/*****************  Bit definition for SYSCFG_EXTICR2 register  ***************/
constexpr uint32_t SYSCFG_EXTICR2_EXTI4            = 0x0000000F; /*!< EXTI 4 configuration */
constexpr uint32_t SYSCFG_EXTICR2_EXTI5            = 0x000000F0; /*!< EXTI 5 configuration */
constexpr uint32_t SYSCFG_EXTICR2_EXTI6            = 0x00000F00; /*!< EXTI 6 configuration */
constexpr uint32_t SYSCFG_EXTICR2_EXTI7            = 0x0000F000; /*!< EXTI 7 configuration */

/*!<*
  * @brief  EXTI4 configuration
  */
constexpr uint32_t SYSCFG_EXTICR2_EXTI4_PA         = 0x00000000; /*!< PA[4] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI4_PB         = 0x00000001; /*!< PB[4] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI4_PC         = 0x00000002; /*!< PC[4] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI4_PD         = 0x00000003; /*!< PD[4] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI4_PE         = 0x00000004; /*!< PE[4] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI4_PF         = 0x00000005; /*!< PF[4] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI4_PG         = 0x00000006; /*!< PG[4] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI4_PH         = 0x00000007; /*!< PH[4] pin */

/*!<*
  * @brief  EXTI5 configuration
  */
constexpr uint32_t SYSCFG_EXTICR2_EXTI5_PA         = 0x00000000; /*!< PA[5] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI5_PB         = 0x00000010; /*!< PB[5] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI5_PC         = 0x00000020; /*!< PC[5] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI5_PD         = 0x00000030; /*!< PD[5] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI5_PE         = 0x00000040; /*!< PE[5] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI5_PF         = 0x00000050; /*!< PF[5] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI5_PG         = 0x00000060; /*!< PG[5] pin */

/*!<*
  * @brief  EXTI6 configuration
  */
constexpr uint32_t SYSCFG_EXTICR2_EXTI6_PA         = 0x00000000; /*!< PA[6] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI6_PB         = 0x00000100; /*!< PB[6] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI6_PC         = 0x00000200; /*!< PC[6] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI6_PD         = 0x00000300; /*!< PD[6] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI6_PE         = 0x00000400; /*!< PE[6] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI6_PF         = 0x00000500; /*!< PF[6] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI6_PG         = 0x00000600; /*!< PG[6] pin */

/*!<*
  * @brief  EXTI7 configuration
  */
constexpr uint32_t SYSCFG_EXTICR2_EXTI7_PA         = 0x00000000; /*!< PA[7] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI7_PB         = 0x00001000; /*!< PB[7] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI7_PC         = 0x00002000; /*!< PC[7] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI7_PD         = 0x00003000; /*!< PD[7] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI7_PE         = 0x00004000; /*!< PE[7] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI7_PF         = 0x00005000; /*!< PF[7] pin */
constexpr uint32_t SYSCFG_EXTICR2_EXTI7_PG         = 0x00006000; /*!< PG[7] pin */

/*****************  Bit definition for SYSCFG_EXTICR3 register  ***************/
constexpr uint32_t SYSCFG_EXTICR3_EXTI8            = 0x0000000F; /*!< EXTI 8 configuration */
constexpr uint32_t SYSCFG_EXTICR3_EXTI9            = 0x000000F0; /*!< EXTI 9 configuration */
constexpr uint32_t SYSCFG_EXTICR3_EXTI10           = 0x00000F00; /*!< EXTI 10 configuration */
constexpr uint32_t SYSCFG_EXTICR3_EXTI11           = 0x0000F000; /*!< EXTI 11 configuration */

/*!<*
  * @brief  EXTI8 configuration
  */
constexpr uint32_t SYSCFG_EXTICR3_EXTI8_PA         = 0x00000000; /*!< PA[8] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI8_PB         = 0x00000001; /*!< PB[8] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI8_PC         = 0x00000002; /*!< PC[8] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI8_PD         = 0x00000003; /*!< PD[8] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI8_PE         = 0x00000004; /*!< PE[8] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI8_PF         = 0x00000005; /*!< PF[8] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI8_PG         = 0x00000006; /*!< PG[8] pin */

/*!<*
  * @brief  EXTI9 configuration
  */
constexpr uint32_t SYSCFG_EXTICR3_EXTI9_PA         = 0x00000000; /*!< PA[9] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI9_PB         = 0x00000010; /*!< PB[9] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI9_PC         = 0x00000020; /*!< PC[9] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI9_PD         = 0x00000030; /*!< PD[9] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI9_PE         = 0x00000040; /*!< PE[9] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI9_PF         = 0x00000050; /*!< PF[9] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI9_PG         = 0x00000060; /*!< PG[9] pin */

/*!<*
  * @brief  EXTI10 configuration
  */
constexpr uint32_t SYSCFG_EXTICR3_EXTI10_PA        = 0x00000000; /*!< PA[10] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI10_PB        = 0x00000100; /*!< PB[10] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI10_PC        = 0x00000200; /*!< PC[10] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI10_PD        = 0x00000300; /*!< PD[10] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI10_PE        = 0x00000400; /*!< PE[10] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI10_PF        = 0x00000500; /*!< PF[10] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI10_PG        = 0x00000600; /*!< PG[10] pin */

/*!<*
  * @brief  EXTI11 configuration
  */
constexpr uint32_t SYSCFG_EXTICR3_EXTI11_PA        = 0x00000000; /*!< PA[11] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI11_PB        = 0x00001000; /*!< PB[11] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI11_PC        = 0x00002000; /*!< PC[11] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI11_PD        = 0x00003000; /*!< PD[11] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI11_PE        = 0x00004000; /*!< PE[11] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI11_PF        = 0x00005000; /*!< PF[11] pin */
constexpr uint32_t SYSCFG_EXTICR3_EXTI11_PG        = 0x00006000; /*!< PG[11] pin */

/*****************  Bit definition for SYSCFG_EXTICR4 register  *****************/
constexpr uint32_t SYSCFG_EXTICR4_EXTI12           = 0x0000000F; /*!< EXTI 12 configuration */
constexpr uint32_t SYSCFG_EXTICR4_EXTI13           = 0x000000F0; /*!< EXTI 13 configuration */
constexpr uint32_t SYSCFG_EXTICR4_EXTI14           = 0x00000F00; /*!< EXTI 14 configuration */
constexpr uint32_t SYSCFG_EXTICR4_EXTI15           = 0x0000F000; /*!< EXTI 15 configuration */

/*!<*
  * @brief  EXTI12 configuration
  */
constexpr uint32_t SYSCFG_EXTICR4_EXTI12_PA        = 0x00000000; /*!< PA[12] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI12_PB        = 0x00000001; /*!< PB[12] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI12_PC        = 0x00000002; /*!< PC[12] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI12_PD        = 0x00000003; /*!< PD[12] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI12_PE        = 0x00000004; /*!< PE[12] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI12_PF        = 0x00000005; /*!< PF[12] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI12_PG        = 0x00000006; /*!< PG[12] pin */

/*!<*
  * @brief  EXTI13 configuration
  */
constexpr uint32_t SYSCFG_EXTICR4_EXTI13_PA        = 0x00000000; /*!< PA[13] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI13_PB        = 0x00000010; /*!< PB[13] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI13_PC        = 0x00000020; /*!< PC[13] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI13_PD        = 0x00000030; /*!< PD[13] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI13_PE        = 0x00000040; /*!< PE[13] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI13_PF        = 0x00000050; /*!< PF[13] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI13_PG        = 0x00000060; /*!< PG[13] pin */

/*!<*
  * @brief  EXTI14 configuration
  */
constexpr uint32_t SYSCFG_EXTICR4_EXTI14_PA        = 0x00000000; /*!< PA[14] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI14_PB        = 0x00000100; /*!< PB[14] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI14_PC        = 0x00000200; /*!< PC[14] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI14_PD        = 0x00000300; /*!< PD[14] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI14_PE        = 0x00000400; /*!< PE[14] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI14_PF        = 0x00000500; /*!< PF[14] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI14_PG        = 0x00000600; /*!< PG[14] pin */

/*!<*
  * @brief  EXTI15 configuration
  */
constexpr uint32_t SYSCFG_EXTICR4_EXTI15_PA        = 0x00000000; /*!< PA[15] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI15_PB        = 0x00001000; /*!< PB[15] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI15_PC        = 0x00002000; /*!< PC[15] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI15_PD        = 0x00003000; /*!< PD[15] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI15_PE        = 0x00004000; /*!< PE[15] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI15_PF        = 0x00005000; /*!< PF[15] pin */
constexpr uint32_t SYSCFG_EXTICR4_EXTI15_PG        = 0x00006000; /*!< PG[15] pin */

/*****************  Bit definition for SYSCFG_CFGR2 register  *****************/
constexpr uint32_t SYSCFG_CFGR2_LOCKUP_LOCK               = 0x00000001; /*!< Enables and locks the LOCKUP = Hardfault; output of CortexM4 with Break Input of TIM1/8/15/16/17/20 */
constexpr uint32_t SYSCFG_CFGR2_SRAM_PARITY_LOCK          = 0x00000002; /*!< Enables and locks the SRAM_PARITY error signal with Break Input of TIM1/8/15/16/17/20 */
constexpr uint32_t SYSCFG_CFGR2_PVD_LOCK                  = 0x00000004; /*!< Enables and locks the PVD connection with TIM1/8/15/16/17/20 Break Input, as well as the PVDE and PLS[2:0] in the PWR_CR register */
constexpr uint32_t SYSCFG_CFGR2_BYP_ADDR_PAR              = 0x00000010; /*!< Disables the adddress parity check on RAM */
constexpr uint32_t SYSCFG_CFGR2_SRAM_PE                   = 0x00000100; /*!< SRAM Parity error flag */

/*****************  Bit definition for SYSCFG_CFGR4 register  *****************/
constexpr uint32_t SYSCFG_CFGR4_ADC12_EXT2_RMP            = 0x00000001; /*!< ADC12 regular channel EXT2 remap */
constexpr uint32_t SYSCFG_CFGR4_ADC12_EXT3_RMP            = 0x00000002; /*!< ADC12 regular channel EXT3 remap */
constexpr uint32_t SYSCFG_CFGR4_ADC12_EXT5_RMP            = 0x00000004; /*!< ADC12 regular channel EXT5 remap */
constexpr uint32_t SYSCFG_CFGR4_ADC12_EXT13_RMP           = 0x00000008; /*!< ADC12 regular channel EXT13 remap */
constexpr uint32_t SYSCFG_CFGR4_ADC12_EXT15_RMP           = 0x00000010; /*!< ADC12 regular channel EXT15 remap */
constexpr uint32_t SYSCFG_CFGR4_ADC12_JEXT3_RMP           = 0x00000020; /*!< ADC12 injected channel JEXT3 remap */
constexpr uint32_t SYSCFG_CFGR4_ADC12_JEXT6_RMP           = 0x00000040; /*!< ADC12 injected channel JEXT6 remap */
constexpr uint32_t SYSCFG_CFGR4_ADC12_JEXT13_RMP          = 0x00000080; /*!< ADC12 injected channel JEXT13 remap */
constexpr uint32_t SYSCFG_CFGR4_ADC34_EXT5_RMP            = 0x00000100; /*!< ADC34 regular channel EXT5 remap */
constexpr uint32_t SYSCFG_CFGR4_ADC34_EXT6_RMP            = 0x00000200; /*!< ADC34 regular channel EXT6 remap */
constexpr uint32_t SYSCFG_CFGR4_ADC34_EXT15_RMP           = 0x00000400; /*!< ADC34 regular channel EXT15 remap */
constexpr uint32_t SYSCFG_CFGR4_ADC34_JEXT5_RMP           = 0x00000800; /*!< ADC34 injected channel JEXT5 remap */
constexpr uint32_t SYSCFG_CFGR4_ADC34_JEXT11_RMP          = 0x00001000; /*!< ADC34 injected channel JEXT11 remap */
constexpr uint32_t SYSCFG_CFGR4_ADC34_JEXT14_RMP          = 0x00002000; /*!< ADC34 injected channel JEXT14 remap */

/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TIM_CR1 register  ********************/
constexpr uint32_t  TIM_CR1_CEN                         = 0x00000001;            /*!<Counter enable */
constexpr uint32_t  TIM_CR1_UDIS                        = 0x00000002;            /*!<Update disable */
constexpr uint32_t  TIM_CR1_URS                         = 0x00000004;            /*!<Update request source */
constexpr uint32_t  TIM_CR1_OPM                         = 0x00000008;            /*!<One pulse mode */
constexpr uint32_t  TIM_CR1_DIR                         = 0x00000010;            /*!<Direction */

constexpr uint32_t  TIM_CR1_CMS                         = 0x00000060;            /*!<CMS[1:0] bits = Center-aligned mode selection; */
constexpr uint32_t  TIM_CR1_CMS_0                       = 0x00000020;            /*!<Bit 0 */
constexpr uint32_t  TIM_CR1_CMS_1                       = 0x00000040;            /*!<Bit 1 */

constexpr uint32_t  TIM_CR1_ARPE                        = 0x00000080;            /*!<Auto-reload preload enable */

constexpr uint32_t  TIM_CR1_CKD                         = 0x00000300;            /*!<CKD[1:0] bits = clock division; */
constexpr uint32_t  TIM_CR1_CKD_0                       = 0x00000100;            /*!<Bit 0 */
constexpr uint32_t  TIM_CR1_CKD_1                       = 0x00000200;            /*!<Bit 1 */

constexpr uint32_t  TIM_CR1_UIFREMAP                    = 0x00000800;            /*!<Update interrupt flag remap */

/*******************  Bit definition for TIM_CR2 register  ********************/
constexpr uint32_t  TIM_CR2_CCPC                        = 0x00000001;            /*!<Capture/Compare Preloaded Control */
constexpr uint32_t  TIM_CR2_CCUS                        = 0x00000004;            /*!<Capture/Compare Control Update Selection */
constexpr uint32_t  TIM_CR2_CCDS                        = 0x00000008;            /*!<Capture/Compare DMA Selection */

constexpr uint32_t  TIM_CR2_MMS                         = 0x00000070;            /*!<MMS[2:0] bits = Master Mode Selection; */
constexpr uint32_t  TIM_CR2_MMS_0                       = 0x00000010;            /*!<Bit 0 */
constexpr uint32_t  TIM_CR2_MMS_1                       = 0x00000020;            /*!<Bit 1 */
constexpr uint32_t  TIM_CR2_MMS_2                       = 0x00000040;            /*!<Bit 2 */

constexpr uint32_t  TIM_CR2_TI1S                        = 0x00000080;            /*!<TI1 Selection */
constexpr uint32_t  TIM_CR2_OIS1                        = 0x00000100;            /*!<Output Idle state 1 = OC1 output; */
constexpr uint32_t  TIM_CR2_OIS1N                       = 0x00000200;            /*!<Output Idle state 1 = OC1N output; */
constexpr uint32_t  TIM_CR2_OIS2                        = 0x00000400;            /*!<Output Idle state 2 = OC2 output; */
constexpr uint32_t  TIM_CR2_OIS2N                       = 0x00000800;            /*!<Output Idle state 2 = OC2N output; */
constexpr uint32_t  TIM_CR2_OIS3                        = 0x00001000;            /*!<Output Idle state 3 = OC3 output; */
constexpr uint32_t  TIM_CR2_OIS3N                       = 0x00002000;            /*!<Output Idle state 3 = OC3N output; */
constexpr uint32_t  TIM_CR2_OIS4                        = 0x00004000;            /*!<Output Idle state 4 = OC4 output; */
constexpr uint32_t  TIM_CR2_OIS5                        = 0x00010000;            /*!<Output Idle state 4 = OC4 output; */
constexpr uint32_t  TIM_CR2_OIS6                        = 0x00040000;            /*!<Output Idle state 4 = OC4 output; */

constexpr uint32_t  TIM_CR2_MMS2                        = 0x00F00000;            /*!<MMS[2:0] bits = Master Mode Selection; */
constexpr uint32_t  TIM_CR2_MMS2_0                      = 0x00100000;            /*!<Bit 0 */
constexpr uint32_t  TIM_CR2_MMS2_1                      = 0x00200000;            /*!<Bit 1 */
constexpr uint32_t  TIM_CR2_MMS2_2                      = 0x00400000;            /*!<Bit 2 */
constexpr uint32_t  TIM_CR2_MMS2_3                      = 0x00800000;            /*!<Bit 2 */

/*******************  Bit definition for TIM_SMCR register  *******************/
constexpr uint32_t  TIM_SMCR_SMS                        = 0x00010007;            /*!<SMS[2:0] bits = Slave mode selection; */
constexpr uint32_t  TIM_SMCR_SMS_0                      = 0x00000001;            /*!<Bit 0 */
constexpr uint32_t  TIM_SMCR_SMS_1                      = 0x00000002;            /*!<Bit 1 */
constexpr uint32_t  TIM_SMCR_SMS_2                      = 0x00000004;            /*!<Bit 2 */
constexpr uint32_t  TIM_SMCR_SMS_3                      = 0x00010000;            /*!<Bit 3 */

constexpr uint32_t  TIM_SMCR_OCCS                       = 0x00000008;            /*!< OCREF clear selection */

constexpr uint32_t  TIM_SMCR_TS                         = 0x00000070;            /*!<TS[2:0] bits = Trigger selection; */
constexpr uint32_t  TIM_SMCR_TS_0                       = 0x00000010;            /*!<Bit 0 */
constexpr uint32_t  TIM_SMCR_TS_1                       = 0x00000020;            /*!<Bit 1 */
constexpr uint32_t  TIM_SMCR_TS_2                       = 0x00000040;            /*!<Bit 2 */

constexpr uint32_t  TIM_SMCR_MSM                        = 0x00000080;            /*!<Master/slave mode */

constexpr uint32_t  TIM_SMCR_ETF                        = 0x00000F00;            /*!<ETF[3:0] bits = External trigger filter; */
constexpr uint32_t  TIM_SMCR_ETF_0                      = 0x00000100;            /*!<Bit 0 */
constexpr uint32_t  TIM_SMCR_ETF_1                      = 0x00000200;            /*!<Bit 1 */
constexpr uint32_t  TIM_SMCR_ETF_2                      = 0x00000400;            /*!<Bit 2 */
constexpr uint32_t  TIM_SMCR_ETF_3                      = 0x00000800;            /*!<Bit 3 */

constexpr uint32_t  TIM_SMCR_ETPS                       = 0x00003000;            /*!<ETPS[1:0] bits = External trigger prescaler; */
constexpr uint32_t  TIM_SMCR_ETPS_0                     = 0x00001000;            /*!<Bit 0 */
constexpr uint32_t  TIM_SMCR_ETPS_1                     = 0x00002000;            /*!<Bit 1 */

constexpr uint32_t  TIM_SMCR_ECE                        = 0x00004000;            /*!<External clock enable */
constexpr uint32_t  TIM_SMCR_ETP                        = 0x00008000;            /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
constexpr uint32_t  TIM_DIER_UIE                        = 0x00000001;            /*!<Update interrupt enable */
constexpr uint32_t  TIM_DIER_CC1IE                      = 0x00000002;            /*!<Capture/Compare 1 interrupt enable */
constexpr uint32_t  TIM_DIER_CC2IE                      = 0x00000004;            /*!<Capture/Compare 2 interrupt enable */
constexpr uint32_t  TIM_DIER_CC3IE                      = 0x00000008;            /*!<Capture/Compare 3 interrupt enable */
constexpr uint32_t  TIM_DIER_CC4IE                      = 0x00000010;            /*!<Capture/Compare 4 interrupt enable */
constexpr uint32_t  TIM_DIER_COMIE                      = 0x00000020;            /*!<COM interrupt enable */
constexpr uint32_t  TIM_DIER_TIE                        = 0x00000040;            /*!<Trigger interrupt enable */
constexpr uint32_t  TIM_DIER_BIE                        = 0x00000080;            /*!<Break interrupt enable */
constexpr uint32_t  TIM_DIER_UDE                        = 0x00000100;            /*!<Update DMA request enable */
constexpr uint32_t  TIM_DIER_CC1DE                      = 0x00000200;            /*!<Capture/Compare 1 DMA request enable */
constexpr uint32_t  TIM_DIER_CC2DE                      = 0x00000400;            /*!<Capture/Compare 2 DMA request enable */
constexpr uint32_t  TIM_DIER_CC3DE                      = 0x00000800;            /*!<Capture/Compare 3 DMA request enable */
constexpr uint32_t  TIM_DIER_CC4DE                      = 0x00001000;            /*!<Capture/Compare 4 DMA request enable */
constexpr uint32_t  TIM_DIER_COMDE                      = 0x00002000;            /*!<COM DMA request enable */
constexpr uint32_t  TIM_DIER_TDE                        = 0x00004000;            /*!<Trigger DMA request enable */

/********************  Bit definition for TIM_SR register  ********************/
constexpr uint32_t  TIM_SR_UIF                          = 0x00000001;            /*!<Update interrupt Flag */
constexpr uint32_t  TIM_SR_CC1IF                        = 0x00000002;            /*!<Capture/Compare 1 interrupt Flag */
constexpr uint32_t  TIM_SR_CC2IF                        = 0x00000004;            /*!<Capture/Compare 2 interrupt Flag */
constexpr uint32_t  TIM_SR_CC3IF                        = 0x00000008;            /*!<Capture/Compare 3 interrupt Flag */
constexpr uint32_t  TIM_SR_CC4IF                        = 0x00000010;            /*!<Capture/Compare 4 interrupt Flag */
constexpr uint32_t  TIM_SR_COMIF                        = 0x00000020;            /*!<COM interrupt Flag */
constexpr uint32_t  TIM_SR_TIF                          = 0x00000040;            /*!<Trigger interrupt Flag */
constexpr uint32_t  TIM_SR_BIF                          = 0x00000080;            /*!<Break interrupt Flag */
constexpr uint32_t  TIM_SR_B2IF                         = 0x00000100;            /*!<Break2 interrupt Flag */
constexpr uint32_t  TIM_SR_CC1OF                        = 0x00000200;            /*!<Capture/Compare 1 Overcapture Flag */
constexpr uint32_t  TIM_SR_CC2OF                        = 0x00000400;            /*!<Capture/Compare 2 Overcapture Flag */
constexpr uint32_t  TIM_SR_CC3OF                        = 0x00000800;            /*!<Capture/Compare 3 Overcapture Flag */
constexpr uint32_t  TIM_SR_CC4OF                        = 0x00001000;            /*!<Capture/Compare 4 Overcapture Flag */
constexpr uint32_t  TIM_SR_CC5IF                        = 0x00010000;            /*!<Capture/Compare 5 interrupt Flag */
constexpr uint32_t  TIM_SR_CC6IF                        = 0x00020000;            /*!<Capture/Compare 6 interrupt Flag */

/*******************  Bit definition for TIM_EGR register  ********************/
constexpr uint32_t  TIM_EGR_UG                          = 0x00000001;               /*!<Update Generation */
constexpr uint32_t  TIM_EGR_CC1G                        = 0x00000002;               /*!<Capture/Compare 1 Generation */
constexpr uint32_t  TIM_EGR_CC2G                        = 0x00000004;               /*!<Capture/Compare 2 Generation */
constexpr uint32_t  TIM_EGR_CC3G                        = 0x00000008;               /*!<Capture/Compare 3 Generation */
constexpr uint32_t  TIM_EGR_CC4G                        = 0x00000010;               /*!<Capture/Compare 4 Generation */
constexpr uint32_t  TIM_EGR_COMG                        = 0x00000020;               /*!<Capture/Compare Control Update Generation */
constexpr uint32_t  TIM_EGR_TG                          = 0x00000040;               /*!<Trigger Generation */
constexpr uint32_t  TIM_EGR_BG                          = 0x00000080;               /*!<Break Generation */
constexpr uint32_t  TIM_EGR_B2G                         = 0x00000100;               /*!<Break Generation */

/******************  Bit definition for TIM_CCMR1 register  *******************/
constexpr uint32_t  TIM_CCMR1_CC1S                      = 0x00000003;            /*!<CC1S[1:0] bits = Capture/Compare 1 Selection; */
constexpr uint32_t  TIM_CCMR1_CC1S_0                    = 0x00000001;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR1_CC1S_1                    = 0x00000002;            /*!<Bit 1 */

constexpr uint32_t  TIM_CCMR1_OC1FE                     = 0x00000004;            /*!<Output Compare 1 Fast enable */
constexpr uint32_t  TIM_CCMR1_OC1PE                     = 0x00000008;            /*!<Output Compare 1 Preload enable */

constexpr uint32_t  TIM_CCMR1_OC1M                      = 0x00010070;            /*!<OC1M[2:0] bits = Output Compare 1 Mode; */
constexpr uint32_t  TIM_CCMR1_OC1M_0                    = 0x00000010;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR1_OC1M_1                    = 0x00000020;            /*!<Bit 1 */
constexpr uint32_t  TIM_CCMR1_OC1M_2                    = 0x00000040;            /*!<Bit 2 */
constexpr uint32_t  TIM_CCMR1_OC1M_3                    = 0x00010000;            /*!<Bit 3 */

constexpr uint32_t  TIM_CCMR1_OC1CE                     = 0x00000080;            /*!<Output Compare 1Clear Enable */

constexpr uint32_t  TIM_CCMR1_CC2S                      = 0x00000300;            /*!<CC2S[1:0] bits = Capture/Compare 2 Selection; */
constexpr uint32_t  TIM_CCMR1_CC2S_0                    = 0x00000100;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR1_CC2S_1                    = 0x00000200;            /*!<Bit 1 */

constexpr uint32_t  TIM_CCMR1_OC2FE                     = 0x00000400;            /*!<Output Compare 2 Fast enable */
constexpr uint32_t  TIM_CCMR1_OC2PE                     = 0x00000800;            /*!<Output Compare 2 Preload enable */

constexpr uint32_t  TIM_CCMR1_OC2M                      = 0x01007000;            /*!<OC2M[2:0] bits = Output Compare 2 Mode; */
constexpr uint32_t  TIM_CCMR1_OC2M_0                    = 0x00001000;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR1_OC2M_1                    = 0x00002000;            /*!<Bit 1 */
constexpr uint32_t  TIM_CCMR1_OC2M_2                    = 0x00004000;            /*!<Bit 2 */
constexpr uint32_t  TIM_CCMR1_OC2M_3                    = 0x01000000;            /*!<Bit 3 */

constexpr uint32_t  TIM_CCMR1_OC2CE                     = 0x00008000;            /*!<Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

constexpr uint32_t  TIM_CCMR1_IC1PSC                    = 0x0000000C;            /*!<IC1PSC[1:0] bits = Input Capture 1 Prescaler; */
constexpr uint32_t  TIM_CCMR1_IC1PSC_0                  = 0x00000004;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR1_IC1PSC_1                  = 0x00000008;            /*!<Bit 1 */

constexpr uint32_t  TIM_CCMR1_IC1F                      = 0x000000F0;            /*!<IC1F[3:0] bits = Input Capture 1 Filter; */
constexpr uint32_t  TIM_CCMR1_IC1F_0                    = 0x00000010;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR1_IC1F_1                    = 0x00000020;            /*!<Bit 1 */
constexpr uint32_t  TIM_CCMR1_IC1F_2                    = 0x00000040;            /*!<Bit 2 */
constexpr uint32_t  TIM_CCMR1_IC1F_3                    = 0x00000080;            /*!<Bit 3 */

constexpr uint32_t  TIM_CCMR1_IC2PSC                    = 0x00000C00;            /*!<IC2PSC[1:0] bits = Input Capture 2 Prescaler; */
constexpr uint32_t  TIM_CCMR1_IC2PSC_0                  = 0x00000400;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR1_IC2PSC_1                  = 0x00000800;            /*!<Bit 1 */

constexpr uint32_t  TIM_CCMR1_IC2F                      = 0x0000F000;            /*!<IC2F[3:0] bits = Input Capture 2 Filter; */
constexpr uint32_t  TIM_CCMR1_IC2F_0                    = 0x00001000;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR1_IC2F_1                    = 0x00002000;            /*!<Bit 1 */
constexpr uint32_t  TIM_CCMR1_IC2F_2                    = 0x00004000;            /*!<Bit 2 */
constexpr uint32_t  TIM_CCMR1_IC2F_3                    = 0x00008000;            /*!<Bit 3 */

/******************  Bit definition for TIM_CCMR2 register  *******************/
constexpr uint32_t  TIM_CCMR2_CC3S                      = 0x00000003;            /*!<CC3S[1:0] bits = Capture/Compare 3 Selection; */
constexpr uint32_t  TIM_CCMR2_CC3S_0                    = 0x00000001;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR2_CC3S_1                    = 0x00000002;            /*!<Bit 1 */

constexpr uint32_t  TIM_CCMR2_OC3FE                     = 0x00000004;            /*!<Output Compare 3 Fast enable */
constexpr uint32_t  TIM_CCMR2_OC3PE                     = 0x00000008;            /*!<Output Compare 3 Preload enable */

constexpr uint32_t  TIM_CCMR2_OC3M                      = 0x00010070;            /*!<OC3M[2:0] bits = Output Compare 3 Mode; */
constexpr uint32_t  TIM_CCMR2_OC3M_0                    = 0x00000010;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR2_OC3M_1                    = 0x00000020;            /*!<Bit 1 */
constexpr uint32_t  TIM_CCMR2_OC3M_2                    = 0x00000040;            /*!<Bit 2 */
constexpr uint32_t  TIM_CCMR2_OC3M_3                    = 0x00010000;            /*!<Bit 3 */

constexpr uint32_t  TIM_CCMR2_OC3CE                     = 0x00000080;            /*!<Output Compare 3 Clear Enable */

constexpr uint32_t  TIM_CCMR2_CC4S                      = 0x00000300;            /*!<CC4S[1:0] bits = Capture/Compare 4 Selection; */
constexpr uint32_t  TIM_CCMR2_CC4S_0                    = 0x00000100;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR2_CC4S_1                    = 0x00000200;            /*!<Bit 1 */

constexpr uint32_t  TIM_CCMR2_OC4FE                     = 0x00000400;            /*!<Output Compare 4 Fast enable */
constexpr uint32_t  TIM_CCMR2_OC4PE                     = 0x00000800;            /*!<Output Compare 4 Preload enable */

constexpr uint32_t  TIM_CCMR2_OC4M                      = 0x01007000;            /*!<OC4M[2:0] bits = Output Compare 4 Mode; */
constexpr uint32_t  TIM_CCMR2_OC4M_0                    = 0x00001000;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR2_OC4M_1                    = 0x00002000;            /*!<Bit 1 */
constexpr uint32_t  TIM_CCMR2_OC4M_2                    = 0x00004000;            /*!<Bit 2 */
constexpr uint32_t  TIM_CCMR2_OC4M_3                    = 0x01000000;            /*!<Bit 3 */

constexpr uint32_t  TIM_CCMR2_OC4CE                     = 0x00008000;            /*!<Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

constexpr uint32_t  TIM_CCMR2_IC3PSC                    = 0x00000000000C;            /*!<IC3PSC[1:0] bits = Input Capture 3 Prescaler; */
constexpr uint32_t  TIM_CCMR2_IC3PSC_0                  = 0x000000000004;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR2_IC3PSC_1                  = 0x000000000008;            /*!<Bit 1 */

constexpr uint32_t  TIM_CCMR2_IC3F                      = 0x0000000000F0;            /*!<IC3F[3:0] bits = Input Capture 3 Filter; */
constexpr uint32_t  TIM_CCMR2_IC3F_0                    = 0x000000000010;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR2_IC3F_1                    = 0x000000000020;            /*!<Bit 1 */
constexpr uint32_t  TIM_CCMR2_IC3F_2                    = 0x000000000040;            /*!<Bit 2 */
constexpr uint32_t  TIM_CCMR2_IC3F_3                    = 0x000000000080;            /*!<Bit 3 */

constexpr uint32_t  TIM_CCMR2_IC4PSC                    = 0x000000000C00;            /*!<IC4PSC[1:0] bits = Input Capture 4 Prescaler; */
constexpr uint32_t  TIM_CCMR2_IC4PSC_0                  = 0x000000000400;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR2_IC4PSC_1                  = 0x000000000800;            /*!<Bit 1 */

constexpr uint32_t  TIM_CCMR2_IC4F                      = 0x00000000F000;            /*!<IC4F[3:0] bits = Input Capture 4 Filter; */
constexpr uint32_t  TIM_CCMR2_IC4F_0                    = 0x000000001000;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR2_IC4F_1                    = 0x000000002000;            /*!<Bit 1 */
constexpr uint32_t  TIM_CCMR2_IC4F_2                    = 0x000000004000;            /*!<Bit 2 */
constexpr uint32_t  TIM_CCMR2_IC4F_3                    = 0x000000008000;            /*!<Bit 3 */

/*******************  Bit definition for TIM_CCER register  *******************/
constexpr uint32_t  TIM_CCER_CC1E                       = 0x00000001;            /*!<Capture/Compare 1 output enable */
constexpr uint32_t  TIM_CCER_CC1P                       = 0x00000002;            /*!<Capture/Compare 1 output Polarity */
constexpr uint32_t  TIM_CCER_CC1NE                      = 0x00000004;            /*!<Capture/Compare 1 Complementary output enable */
constexpr uint32_t  TIM_CCER_CC1NP                      = 0x00000008;            /*!<Capture/Compare 1 Complementary output Polarity */
constexpr uint32_t  TIM_CCER_CC2E                       = 0x00000010;            /*!<Capture/Compare 2 output enable */
constexpr uint32_t  TIM_CCER_CC2P                       = 0x00000020;            /*!<Capture/Compare 2 output Polarity */
constexpr uint32_t  TIM_CCER_CC2NE                      = 0x00000040;            /*!<Capture/Compare 2 Complementary output enable */
constexpr uint32_t  TIM_CCER_CC2NP                      = 0x00000080;            /*!<Capture/Compare 2 Complementary output Polarity */
constexpr uint32_t  TIM_CCER_CC3E                       = 0x00000100;            /*!<Capture/Compare 3 output enable */
constexpr uint32_t  TIM_CCER_CC3P                       = 0x00000200;            /*!<Capture/Compare 3 output Polarity */
constexpr uint32_t  TIM_CCER_CC3NE                      = 0x00000400;            /*!<Capture/Compare 3 Complementary output enable */
constexpr uint32_t  TIM_CCER_CC3NP                      = 0x00000800;            /*!<Capture/Compare 3 Complementary output Polarity */
constexpr uint32_t  TIM_CCER_CC4E                       = 0x00001000;            /*!<Capture/Compare 4 output enable */
constexpr uint32_t  TIM_CCER_CC4P                       = 0x00002000;            /*!<Capture/Compare 4 output Polarity */
constexpr uint32_t  TIM_CCER_CC4NP                      = 0x00008000;            /*!<Capture/Compare 4 Complementary output Polarity */
constexpr uint32_t  TIM_CCER_CC5E                       = 0x00010000;            /*!<Capture/Compare 5 output enable */
constexpr uint32_t  TIM_CCER_CC5P                       = 0x00020000;            /*!<Capture/Compare 5 output Polarity */
constexpr uint32_t  TIM_CCER_CC6E                       = 0x00100000;            /*!<Capture/Compare 6 output enable */
constexpr uint32_t  TIM_CCER_CC6P                       = 0x00200000;            /*!<Capture/Compare 6 output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
constexpr uint32_t  TIM_CNT_CNT                         = 0xFFFFFFFF;            /*!<Counter Value */
constexpr uint32_t  TIM_CNT_UIFCPY                      = 0x80000000;            /*!<Update interrupt flag copy */

/*******************  Bit definition for TIM_PSC register  ********************/
constexpr uint32_t  TIM_PSC_PSC                         = 0x0000FFFF;            /*!<Prescaler Value */

/*******************  Bit definition for TIM_ARR register  ********************/
constexpr uint32_t  TIM_ARR_ARR                         = 0xFFFFFFFF;            /*!<actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  ********************/
constexpr uint32_t  TIM_RCR_REP                         = 0x000000FF;            /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  *******************/
constexpr uint32_t  TIM_CCR1_CCR1                       = 0x0000FFFF;            /*!<Capture/Compare 1 Value */

/*******************  Bit definition for TIM_CCR2 register  *******************/
constexpr uint32_t  TIM_CCR2_CCR2                       = 0x0000FFFF;            /*!<Capture/Compare 2 Value */

/*******************  Bit definition for TIM_CCR3 register  *******************/
constexpr uint32_t  TIM_CCR3_CCR3                       = 0x0000FFFF;            /*!<Capture/Compare 3 Value */

/*******************  Bit definition for TIM_CCR4 register  *******************/
constexpr uint32_t  TIM_CCR4_CCR4                       = 0x0000FFFF;            /*!<Capture/Compare 4 Value */

/*******************  Bit definition for TIM_CCR5 register  *******************/
constexpr uint32_t  TIM_CCR5_CCR5                       = 0xFFFFFFFF;        /*!<Capture/Compare 5 Value */
constexpr uint32_t  TIM_CCR5_GC5C1                      = 0x20000000;        /*!<Group Channel 5 and Channel 1 */
constexpr uint32_t  TIM_CCR5_GC5C2                      = 0x40000000;        /*!<Group Channel 5 and Channel 2 */
constexpr uint32_t  TIM_CCR5_GC5C3                      = 0x80000000;        /*!<Group Channel 5 and Channel 3 */

/*******************  Bit definition for TIM_CCR6 register  *******************/
constexpr uint32_t  TIM_CCR6_CCR6                       = 0x0000FFFF;            /*!<Capture/Compare 6 Value */

/*******************  Bit definition for TIM_BDTR register  *******************/
constexpr uint32_t  TIM_BDTR_DTG                        = 0x000000FF;            /*!<DTG[0:7] bits = Dead-Time Generator set-up; */
constexpr uint32_t  TIM_BDTR_DTG_0                      = 0x00000001;            /*!<Bit 0 */
constexpr uint32_t  TIM_BDTR_DTG_1                      = 0x00000002;            /*!<Bit 1 */
constexpr uint32_t  TIM_BDTR_DTG_2                      = 0x00000004;            /*!<Bit 2 */
constexpr uint32_t  TIM_BDTR_DTG_3                      = 0x00000008;            /*!<Bit 3 */
constexpr uint32_t  TIM_BDTR_DTG_4                      = 0x00000010;            /*!<Bit 4 */
constexpr uint32_t  TIM_BDTR_DTG_5                      = 0x00000020;            /*!<Bit 5 */
constexpr uint32_t  TIM_BDTR_DTG_6                      = 0x00000040;            /*!<Bit 6 */
constexpr uint32_t  TIM_BDTR_DTG_7                      = 0x00000080;            /*!<Bit 7 */

constexpr uint32_t  TIM_BDTR_LOCK                       = 0x00000300;            /*!<LOCK[1:0] bits = Lock Configuration; */
constexpr uint32_t  TIM_BDTR_LOCK_0                     = 0x00000100;            /*!<Bit 0 */
constexpr uint32_t  TIM_BDTR_LOCK_1                     = 0x00000200;            /*!<Bit 1 */

constexpr uint32_t  TIM_BDTR_OSSI                       = 0x00000400;            /*!<Off-State Selection for Idle mode */
constexpr uint32_t  TIM_BDTR_OSSR                       = 0x00000800;            /*!<Off-State Selection for Run mode */
constexpr uint32_t  TIM_BDTR_BKE                        = 0x00001000;            /*!<Break enable for Break1 */
constexpr uint32_t  TIM_BDTR_BKP                        = 0x00002000;            /*!<Break Polarity for Break1 */
constexpr uint32_t  TIM_BDTR_AOE                        = 0x00004000;            /*!<Automatic Output enable */
constexpr uint32_t  TIM_BDTR_MOE                        = 0x00008000;            /*!<Main Output enable */

constexpr uint32_t  TIM_BDTR_BKF                        = 0x000F0000;            /*!<Break Filter for Break1 */
constexpr uint32_t  TIM_BDTR_BK2F                       = 0x00F00000;            /*!<Break Filter for Break2 */

constexpr uint32_t  TIM_BDTR_BK2E                       = 0x01000000;            /*!<Break enable for Break2 */
constexpr uint32_t  TIM_BDTR_BK2P                       = 0x02000000;            /*!<Break Polarity for Break2 */

/*******************  Bit definition for TIM_DCR register  ********************/
constexpr uint32_t  TIM_DCR_DBA                         = 0x0000001F;            /*!<DBA[4:0] bits = DMA Base Address; */
constexpr uint32_t  TIM_DCR_DBA_0                       = 0x00000001;            /*!<Bit 0 */
constexpr uint32_t  TIM_DCR_DBA_1                       = 0x00000002;            /*!<Bit 1 */
constexpr uint32_t  TIM_DCR_DBA_2                       = 0x00000004;            /*!<Bit 2 */
constexpr uint32_t  TIM_DCR_DBA_3                       = 0x00000008;            /*!<Bit 3 */
constexpr uint32_t  TIM_DCR_DBA_4                       = 0x00000010;            /*!<Bit 4 */

constexpr uint32_t  TIM_DCR_DBL                         = 0x00001F00;            /*!<DBL[4:0] bits = DMA Burst Length; */
constexpr uint32_t  TIM_DCR_DBL_0                       = 0x00000100;            /*!<Bit 0 */
constexpr uint32_t  TIM_DCR_DBL_1                       = 0x00000200;            /*!<Bit 1 */
constexpr uint32_t  TIM_DCR_DBL_2                       = 0x00000400;            /*!<Bit 2 */
constexpr uint32_t  TIM_DCR_DBL_3                       = 0x00000800;            /*!<Bit 3 */
constexpr uint32_t  TIM_DCR_DBL_4                       = 0x00001000;            /*!<Bit 4 */

/*******************  Bit definition for TIM_DMAR register  *******************/
constexpr uint32_t  TIM_DMAR_DMAB                       = 0x0000FFFF;            /*!<DMA register for burst accesses */

/*******************  Bit definition for TIM16_OR register  ********************/
constexpr uint32_t TIM16_OR_TI1_RMP                     = 0x00000003;            /*!<TI1_RMP[1:0] bits = TIM16 Input 1 remap; */
constexpr uint32_t TIM16_OR_TI1_RMP_0                   = 0x00000001;            /*!<Bit 0 */
constexpr uint32_t TIM16_OR_TI1_RMP_1                   = 0x00000002;            /*!<Bit 1 */

/*******************  Bit definition for TIM1_OR register  ********************/
constexpr uint32_t TIM1_OR_ETR_RMP                      = 0x0000000F;            /*!<ETR_RMP[3:0] bits = TIM1 ETR remap; */
constexpr uint32_t TIM1_OR_ETR_RMP_0                    = 0x00000001;            /*!<Bit 0 */
constexpr uint32_t TIM1_OR_ETR_RMP_1                    = 0x00000002;            /*!<Bit 1 */
constexpr uint32_t TIM1_OR_ETR_RMP_2                    = 0x00000004;            /*!<Bit 2 */
constexpr uint32_t TIM1_OR_ETR_RMP_3                    = 0x00000008;            /*!<Bit 3 */

/*******************  Bit definition for TIM8_OR register  ********************/
constexpr uint32_t TIM8_OR_ETR_RMP                      = 0x0000000F;            /*!<ETR_RMP[3:0] bits = TIM8 ETR remap; */
constexpr uint32_t TIM8_OR_ETR_RMP_0                    = 0x00000001;            /*!<Bit 0 */
constexpr uint32_t TIM8_OR_ETR_RMP_1                    = 0x00000002;            /*!<Bit 1 */
constexpr uint32_t TIM8_OR_ETR_RMP_2                    = 0x00000004;            /*!<Bit 2 */
constexpr uint32_t TIM8_OR_ETR_RMP_3                    = 0x00000008;            /*!<Bit 3 */

/*******************  Bit definition for TIM20_OR register  *******************/
constexpr uint32_t TIM20_OR_ETR_RMP                     = 0x0000000F;            /*!<ETR_RMP[3:0] bits = TIM20 ETR remap; */
constexpr uint32_t TIM20_OR_ETR_RMP_0                   = 0x00000001;            /*!<Bit 0 */
constexpr uint32_t TIM20_OR_ETR_RMP_1                   = 0x00000002;            /*!<Bit 1 */
constexpr uint32_t TIM20_OR_ETR_RMP_2                   = 0x00000004;            /*!<Bit 2 */
constexpr uint32_t TIM20_OR_ETR_RMP_3                   = 0x00000008;            /*!<Bit 3 */

/******************  Bit definition for TIM_CCMR3 register  *******************/
constexpr uint32_t  TIM_CCMR3_OC5FE                     = 0x00000004;            /*!<Output Compare 5 Fast enable */
constexpr uint32_t  TIM_CCMR3_OC5PE                     = 0x00000008;            /*!<Output Compare 5 Preload enable */

constexpr uint32_t  TIM_CCMR3_OC5M                      = 0x00010070;            /*!<OC5M[2:0] bits = Output Compare 5 Mode; */
constexpr uint32_t  TIM_CCMR3_OC5M_0                    = 0x00000010;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR3_OC5M_1                    = 0x00000020;            /*!<Bit 1 */
constexpr uint32_t  TIM_CCMR3_OC5M_2                    = 0x00000040;            /*!<Bit 2 */
constexpr uint32_t  TIM_CCMR3_OC5M_3                    = 0x00010000;            /*!<Bit 3 */

constexpr uint32_t  TIM_CCMR3_OC5CE                     = 0x00000080;            /*!<Output Compare 5 Clear Enable */

constexpr uint32_t  TIM_CCMR3_OC6FE                     = 0x00000400;            /*!<Output Compare 6 Fast enable */
constexpr uint32_t  TIM_CCMR3_OC6PE                     = 0x00000800;            /*!<Output Compare 6 Preload enable */

constexpr uint32_t  TIM_CCMR3_OC6M                      = 0x01007000;            /*!<OC6M[2:0] bits = Output Compare 6 Mode; */
constexpr uint32_t  TIM_CCMR3_OC6M_0                    = 0x00001000;            /*!<Bit 0 */
constexpr uint32_t  TIM_CCMR3_OC6M_1                    = 0x00002000;            /*!<Bit 1 */
constexpr uint32_t  TIM_CCMR3_OC6M_2                    = 0x00004000;            /*!<Bit 2 */
constexpr uint32_t  TIM_CCMR3_OC6M_3                    = 0x01000000;            /*!<Bit 3 */

constexpr uint32_t  TIM_CCMR3_OC6CE                     = 0x00008000;            /*!<Output Compare 6 Clear Enable */

/******************************************************************************/
/*                                                                            */
/*                          Touch Sensing Controller = TSC;                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TSC_CR register  *********************/
constexpr uint32_t  TSC_CR_TSCE                         = 0x00000001;            /*!<Touch sensing controller enable */
constexpr uint32_t  TSC_CR_START                        = 0x00000002;            /*!<Start acquisition */
constexpr uint32_t  TSC_CR_AM                           = 0x00000004;            /*!<Acquisition mode */
constexpr uint32_t  TSC_CR_SYNCPOL                      = 0x00000008;            /*!<Synchronization pin polarity */
constexpr uint32_t  TSC_CR_IODEF                        = 0x00000010;            /*!<IO default mode */

constexpr uint32_t  TSC_CR_MCV                          = 0x000000E0;            /*!<MCV[2:0] bits = Max Count Value; */
constexpr uint32_t  TSC_CR_MCV_0                        = 0x00000020;            /*!<Bit 0 */
constexpr uint32_t  TSC_CR_MCV_1                        = 0x00000040;            /*!<Bit 1 */
constexpr uint32_t  TSC_CR_MCV_2                        = 0x00000080;            /*!<Bit 2 */

constexpr uint32_t  TSC_CR_PGPSC                        = 0x00007000;            /*!<PGPSC[2:0] bits = Pulse Generator Prescaler; */
constexpr uint32_t  TSC_CR_PGPSC_0                      = 0x00001000;            /*!<Bit 0 */
constexpr uint32_t  TSC_CR_PGPSC_1                      = 0x00002000;            /*!<Bit 1 */
constexpr uint32_t  TSC_CR_PGPSC_2                      = 0x00004000;            /*!<Bit 2 */

constexpr uint32_t  TSC_CR_SSPSC                        = 0x00008000;            /*!<Spread Spectrum Prescaler */
constexpr uint32_t  TSC_CR_SSE                          = 0x00010000;            /*!<Spread Spectrum Enable */

constexpr uint32_t  TSC_CR_SSD                          = 0x00FE0000;            /*!<SSD[6:0] bits = Spread Spectrum Deviation; */
constexpr uint32_t  TSC_CR_SSD_0                        = 0x00020000;            /*!<Bit 0 */
constexpr uint32_t  TSC_CR_SSD_1                        = 0x00040000;            /*!<Bit 1 */
constexpr uint32_t  TSC_CR_SSD_2                        = 0x00080000;            /*!<Bit 2 */
constexpr uint32_t  TSC_CR_SSD_3                        = 0x00100000;            /*!<Bit 3 */
constexpr uint32_t  TSC_CR_SSD_4                        = 0x00200000;            /*!<Bit 4 */
constexpr uint32_t  TSC_CR_SSD_5                        = 0x00400000;            /*!<Bit 5 */
constexpr uint32_t  TSC_CR_SSD_6                        = 0x00800000;            /*!<Bit 6 */

constexpr uint32_t  TSC_CR_CTPL                         = 0x0F000000;            /*!<CTPL[3:0] bits = Charge Transfer pulse low; */
constexpr uint32_t  TSC_CR_CTPL_0                       = 0x01000000;            /*!<Bit 0 */
constexpr uint32_t  TSC_CR_CTPL_1                       = 0x02000000;            /*!<Bit 1 */
constexpr uint32_t  TSC_CR_CTPL_2                       = 0x04000000;            /*!<Bit 2 */
constexpr uint32_t  TSC_CR_CTPL_3                       = 0x08000000;            /*!<Bit 3 */

constexpr uint32_t  TSC_CR_CTPH                         = 0xF0000000;            /*!<CTPH[3:0] bits = Charge Transfer pulse high; */
constexpr uint32_t  TSC_CR_CTPH_0                       = 0x10000000;            /*!<Bit 0 */
constexpr uint32_t  TSC_CR_CTPH_1                       = 0x20000000;            /*!<Bit 1 */
constexpr uint32_t  TSC_CR_CTPH_2                       = 0x40000000;            /*!<Bit 2 */
constexpr uint32_t  TSC_CR_CTPH_3                       = 0x80000000;            /*!<Bit 3 */

/*******************  Bit definition for TSC_IER register  ********************/
constexpr uint32_t  TSC_IER_EOAIE                       = 0x00000001;            /*!<End of acquisition interrupt enable */
constexpr uint32_t  TSC_IER_MCEIE                       = 0x00000002;            /*!<Max count error interrupt enable */

/*******************  Bit definition for TSC_ICR register  ********************/
constexpr uint32_t  TSC_ICR_EOAIC                       = 0x00000001;            /*!<End of acquisition interrupt clear */
constexpr uint32_t  TSC_ICR_MCEIC                       = 0x00000002;            /*!<Max count error interrupt clear */

/*******************  Bit definition for TSC_ISR register  ********************/
constexpr uint32_t  TSC_ISR_EOAF                        = 0x00000001;            /*!<End of acquisition flag */
constexpr uint32_t  TSC_ISR_MCEF                        = 0x00000002;            /*!<Max count error flag */

/*******************  Bit definition for TSC_IOHCR register  ******************/
constexpr uint32_t  TSC_IOHCR_G1_IO1                    = 0x00000001;            /*!<GROUP1_IO1 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G1_IO2                    = 0x00000002;            /*!<GROUP1_IO2 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G1_IO3                    = 0x00000004;            /*!<GROUP1_IO3 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G1_IO4                    = 0x00000008;            /*!<GROUP1_IO4 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G2_IO1                    = 0x00000010;            /*!<GROUP2_IO1 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G2_IO2                    = 0x00000020;            /*!<GROUP2_IO2 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G2_IO3                    = 0x00000040;            /*!<GROUP2_IO3 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G2_IO4                    = 0x00000080;            /*!<GROUP2_IO4 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G3_IO1                    = 0x00000100;            /*!<GROUP3_IO1 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G3_IO2                    = 0x00000200;            /*!<GROUP3_IO2 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G3_IO3                    = 0x00000400;            /*!<GROUP3_IO3 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G3_IO4                    = 0x00000800;            /*!<GROUP3_IO4 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G4_IO1                    = 0x00001000;            /*!<GROUP4_IO1 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G4_IO2                    = 0x00002000;            /*!<GROUP4_IO2 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G4_IO3                    = 0x00004000;            /*!<GROUP4_IO3 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G4_IO4                    = 0x00008000;            /*!<GROUP4_IO4 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G5_IO1                    = 0x00010000;            /*!<GROUP5_IO1 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G5_IO2                    = 0x00020000;            /*!<GROUP5_IO2 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G5_IO3                    = 0x00040000;            /*!<GROUP5_IO3 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G5_IO4                    = 0x00080000;            /*!<GROUP5_IO4 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G6_IO1                    = 0x00100000;            /*!<GROUP6_IO1 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G6_IO2                    = 0x00200000;            /*!<GROUP6_IO2 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G6_IO3                    = 0x00400000;            /*!<GROUP6_IO3 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G6_IO4                    = 0x00800000;            /*!<GROUP6_IO4 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G7_IO1                    = 0x01000000;            /*!<GROUP7_IO1 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G7_IO2                    = 0x02000000;            /*!<GROUP7_IO2 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G7_IO3                    = 0x04000000;            /*!<GROUP7_IO3 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G7_IO4                    = 0x08000000;            /*!<GROUP7_IO4 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G8_IO1                    = 0x10000000;            /*!<GROUP8_IO1 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G8_IO2                    = 0x20000000;            /*!<GROUP8_IO2 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G8_IO3                    = 0x40000000;            /*!<GROUP8_IO3 schmitt trigger hysteresis mode */
constexpr uint32_t  TSC_IOHCR_G8_IO4                    = 0x80000000;            /*!<GROUP8_IO4 schmitt trigger hysteresis mode */

/*******************  Bit definition for TSC_IOASCR register  *****************/
constexpr uint32_t  TSC_IOASCR_G1_IO1                   = 0x00000001;            /*!<GROUP1_IO1 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G1_IO2                   = 0x00000002;            /*!<GROUP1_IO2 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G1_IO3                   = 0x00000004;            /*!<GROUP1_IO3 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G1_IO4                   = 0x00000008;            /*!<GROUP1_IO4 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G2_IO1                   = 0x00000010;            /*!<GROUP2_IO1 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G2_IO2                   = 0x00000020;            /*!<GROUP2_IO2 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G2_IO3                   = 0x00000040;            /*!<GROUP2_IO3 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G2_IO4                   = 0x00000080;            /*!<GROUP2_IO4 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G3_IO1                   = 0x00000100;            /*!<GROUP3_IO1 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G3_IO2                   = 0x00000200;            /*!<GROUP3_IO2 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G3_IO3                   = 0x00000400;            /*!<GROUP3_IO3 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G3_IO4                   = 0x00000800;            /*!<GROUP3_IO4 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G4_IO1                   = 0x00001000;            /*!<GROUP4_IO1 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G4_IO2                   = 0x00002000;            /*!<GROUP4_IO2 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G4_IO3                   = 0x00004000;            /*!<GROUP4_IO3 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G4_IO4                   = 0x00008000;            /*!<GROUP4_IO4 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G5_IO1                   = 0x00010000;            /*!<GROUP5_IO1 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G5_IO2                   = 0x00020000;            /*!<GROUP5_IO2 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G5_IO3                   = 0x00040000;            /*!<GROUP5_IO3 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G5_IO4                   = 0x00080000;            /*!<GROUP5_IO4 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G6_IO1                   = 0x00100000;            /*!<GROUP6_IO1 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G6_IO2                   = 0x00200000;            /*!<GROUP6_IO2 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G6_IO3                   = 0x00400000;            /*!<GROUP6_IO3 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G6_IO4                   = 0x00800000;            /*!<GROUP6_IO4 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G7_IO1                   = 0x01000000;            /*!<GROUP7_IO1 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G7_IO2                   = 0x02000000;            /*!<GROUP7_IO2 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G7_IO3                   = 0x04000000;            /*!<GROUP7_IO3 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G7_IO4                   = 0x08000000;            /*!<GROUP7_IO4 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G8_IO1                   = 0x10000000;            /*!<GROUP8_IO1 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G8_IO2                   = 0x20000000;            /*!<GROUP8_IO2 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G8_IO3                   = 0x40000000;            /*!<GROUP8_IO3 analog switch enable */
constexpr uint32_t  TSC_IOASCR_G8_IO4                   = 0x80000000;            /*!<GROUP8_IO4 analog switch enable */

/*******************  Bit definition for TSC_IOSCR register  ******************/
constexpr uint32_t  TSC_IOSCR_G1_IO1                    = 0x00000001;            /*!<GROUP1_IO1 sampling mode */
constexpr uint32_t  TSC_IOSCR_G1_IO2                    = 0x00000002;            /*!<GROUP1_IO2 sampling mode */
constexpr uint32_t  TSC_IOSCR_G1_IO3                    = 0x00000004;            /*!<GROUP1_IO3 sampling mode */
constexpr uint32_t  TSC_IOSCR_G1_IO4                    = 0x00000008;            /*!<GROUP1_IO4 sampling mode */
constexpr uint32_t  TSC_IOSCR_G2_IO1                    = 0x00000010;            /*!<GROUP2_IO1 sampling mode */
constexpr uint32_t  TSC_IOSCR_G2_IO2                    = 0x00000020;            /*!<GROUP2_IO2 sampling mode */
constexpr uint32_t  TSC_IOSCR_G2_IO3                    = 0x00000040;            /*!<GROUP2_IO3 sampling mode */
constexpr uint32_t  TSC_IOSCR_G2_IO4                    = 0x00000080;            /*!<GROUP2_IO4 sampling mode */
constexpr uint32_t  TSC_IOSCR_G3_IO1                    = 0x00000100;            /*!<GROUP3_IO1 sampling mode */
constexpr uint32_t  TSC_IOSCR_G3_IO2                    = 0x00000200;            /*!<GROUP3_IO2 sampling mode */
constexpr uint32_t  TSC_IOSCR_G3_IO3                    = 0x00000400;            /*!<GROUP3_IO3 sampling mode */
constexpr uint32_t  TSC_IOSCR_G3_IO4                    = 0x00000800;            /*!<GROUP3_IO4 sampling mode */
constexpr uint32_t  TSC_IOSCR_G4_IO1                    = 0x00001000;            /*!<GROUP4_IO1 sampling mode */
constexpr uint32_t  TSC_IOSCR_G4_IO2                    = 0x00002000;            /*!<GROUP4_IO2 sampling mode */
constexpr uint32_t  TSC_IOSCR_G4_IO3                    = 0x00004000;            /*!<GROUP4_IO3 sampling mode */
constexpr uint32_t  TSC_IOSCR_G4_IO4                    = 0x00008000;            /*!<GROUP4_IO4 sampling mode */
constexpr uint32_t  TSC_IOSCR_G5_IO1                    = 0x00010000;            /*!<GROUP5_IO1 sampling mode */
constexpr uint32_t  TSC_IOSCR_G5_IO2                    = 0x00020000;            /*!<GROUP5_IO2 sampling mode */
constexpr uint32_t  TSC_IOSCR_G5_IO3                    = 0x00040000;            /*!<GROUP5_IO3 sampling mode */
constexpr uint32_t  TSC_IOSCR_G5_IO4                    = 0x00080000;            /*!<GROUP5_IO4 sampling mode */
constexpr uint32_t  TSC_IOSCR_G6_IO1                    = 0x00100000;            /*!<GROUP6_IO1 sampling mode */
constexpr uint32_t  TSC_IOSCR_G6_IO2                    = 0x00200000;            /*!<GROUP6_IO2 sampling mode */
constexpr uint32_t  TSC_IOSCR_G6_IO3                    = 0x00400000;            /*!<GROUP6_IO3 sampling mode */
constexpr uint32_t  TSC_IOSCR_G6_IO4                    = 0x00800000;            /*!<GROUP6_IO4 sampling mode */
constexpr uint32_t  TSC_IOSCR_G7_IO1                    = 0x01000000;            /*!<GROUP7_IO1 sampling mode */
constexpr uint32_t  TSC_IOSCR_G7_IO2                    = 0x02000000;            /*!<GROUP7_IO2 sampling mode */
constexpr uint32_t  TSC_IOSCR_G7_IO3                    = 0x04000000;            /*!<GROUP7_IO3 sampling mode */
constexpr uint32_t  TSC_IOSCR_G7_IO4                    = 0x08000000;            /*!<GROUP7_IO4 sampling mode */
constexpr uint32_t  TSC_IOSCR_G8_IO1                    = 0x10000000;            /*!<GROUP8_IO1 sampling mode */
constexpr uint32_t  TSC_IOSCR_G8_IO2                    = 0x20000000;            /*!<GROUP8_IO2 sampling mode */
constexpr uint32_t  TSC_IOSCR_G8_IO3                    = 0x40000000;            /*!<GROUP8_IO3 sampling mode */
constexpr uint32_t  TSC_IOSCR_G8_IO4                    = 0x80000000;            /*!<GROUP8_IO4 sampling mode */

/*******************  Bit definition for TSC_IOCCR register  ******************/
constexpr uint32_t  TSC_IOCCR_G1_IO1                    = 0x00000001;            /*!<GROUP1_IO1 channel mode */
constexpr uint32_t  TSC_IOCCR_G1_IO2                    = 0x00000002;            /*!<GROUP1_IO2 channel mode */
constexpr uint32_t  TSC_IOCCR_G1_IO3                    = 0x00000004;            /*!<GROUP1_IO3 channel mode */
constexpr uint32_t  TSC_IOCCR_G1_IO4                    = 0x00000008;            /*!<GROUP1_IO4 channel mode */
constexpr uint32_t  TSC_IOCCR_G2_IO1                    = 0x00000010;            /*!<GROUP2_IO1 channel mode */
constexpr uint32_t  TSC_IOCCR_G2_IO2                    = 0x00000020;            /*!<GROUP2_IO2 channel mode */
constexpr uint32_t  TSC_IOCCR_G2_IO3                    = 0x00000040;            /*!<GROUP2_IO3 channel mode */
constexpr uint32_t  TSC_IOCCR_G2_IO4                    = 0x00000080;            /*!<GROUP2_IO4 channel mode */
constexpr uint32_t  TSC_IOCCR_G3_IO1                    = 0x00000100;            /*!<GROUP3_IO1 channel mode */
constexpr uint32_t  TSC_IOCCR_G3_IO2                    = 0x00000200;            /*!<GROUP3_IO2 channel mode */
constexpr uint32_t  TSC_IOCCR_G3_IO3                    = 0x00000400;            /*!<GROUP3_IO3 channel mode */
constexpr uint32_t  TSC_IOCCR_G3_IO4                    = 0x00000800;            /*!<GROUP3_IO4 channel mode */
constexpr uint32_t  TSC_IOCCR_G4_IO1                    = 0x00001000;            /*!<GROUP4_IO1 channel mode */
constexpr uint32_t  TSC_IOCCR_G4_IO2                    = 0x00002000;            /*!<GROUP4_IO2 channel mode */
constexpr uint32_t  TSC_IOCCR_G4_IO3                    = 0x00004000;            /*!<GROUP4_IO3 channel mode */
constexpr uint32_t  TSC_IOCCR_G4_IO4                    = 0x00008000;            /*!<GROUP4_IO4 channel mode */
constexpr uint32_t  TSC_IOCCR_G5_IO1                    = 0x00010000;            /*!<GROUP5_IO1 channel mode */
constexpr uint32_t  TSC_IOCCR_G5_IO2                    = 0x00020000;            /*!<GROUP5_IO2 channel mode */
constexpr uint32_t  TSC_IOCCR_G5_IO3                    = 0x00040000;            /*!<GROUP5_IO3 channel mode */
constexpr uint32_t  TSC_IOCCR_G5_IO4                    = 0x00080000;            /*!<GROUP5_IO4 channel mode */
constexpr uint32_t  TSC_IOCCR_G6_IO1                    = 0x00100000;            /*!<GROUP6_IO1 channel mode */
constexpr uint32_t  TSC_IOCCR_G6_IO2                    = 0x00200000;            /*!<GROUP6_IO2 channel mode */
constexpr uint32_t  TSC_IOCCR_G6_IO3                    = 0x00400000;            /*!<GROUP6_IO3 channel mode */
constexpr uint32_t  TSC_IOCCR_G6_IO4                    = 0x00800000;            /*!<GROUP6_IO4 channel mode */
constexpr uint32_t  TSC_IOCCR_G7_IO1                    = 0x01000000;            /*!<GROUP7_IO1 channel mode */
constexpr uint32_t  TSC_IOCCR_G7_IO2                    = 0x02000000;            /*!<GROUP7_IO2 channel mode */
constexpr uint32_t  TSC_IOCCR_G7_IO3                    = 0x04000000;            /*!<GROUP7_IO3 channel mode */
constexpr uint32_t  TSC_IOCCR_G7_IO4                    = 0x08000000;            /*!<GROUP7_IO4 channel mode */
constexpr uint32_t  TSC_IOCCR_G8_IO1                    = 0x10000000;            /*!<GROUP8_IO1 channel mode */
constexpr uint32_t  TSC_IOCCR_G8_IO2                    = 0x20000000;            /*!<GROUP8_IO2 channel mode */
constexpr uint32_t  TSC_IOCCR_G8_IO3                    = 0x40000000;            /*!<GROUP8_IO3 channel mode */
constexpr uint32_t  TSC_IOCCR_G8_IO4                    = 0x80000000;            /*!<GROUP8_IO4 channel mode */

/*******************  Bit definition for TSC_IOGCSR register  *****************/
constexpr uint32_t  TSC_IOGCSR_G1E                      = 0x00000001;            /*!<Analog IO GROUP1 enable */
constexpr uint32_t  TSC_IOGCSR_G2E                      = 0x00000002;            /*!<Analog IO GROUP2 enable */
constexpr uint32_t  TSC_IOGCSR_G3E                      = 0x00000004;            /*!<Analog IO GROUP3 enable */
constexpr uint32_t  TSC_IOGCSR_G4E                      = 0x00000008;            /*!<Analog IO GROUP4 enable */
constexpr uint32_t  TSC_IOGCSR_G5E                      = 0x00000010;            /*!<Analog IO GROUP5 enable */
constexpr uint32_t  TSC_IOGCSR_G6E                      = 0x00000020;            /*!<Analog IO GROUP6 enable */
constexpr uint32_t  TSC_IOGCSR_G7E                      = 0x00000040;            /*!<Analog IO GROUP7 enable */
constexpr uint32_t  TSC_IOGCSR_G8E                      = 0x00000080;            /*!<Analog IO GROUP8 enable */
constexpr uint32_t  TSC_IOGCSR_G1S                      = 0x00010000;            /*!<Analog IO GROUP1 status */
constexpr uint32_t  TSC_IOGCSR_G2S                      = 0x00020000;            /*!<Analog IO GROUP2 status */
constexpr uint32_t  TSC_IOGCSR_G3S                      = 0x00040000;            /*!<Analog IO GROUP3 status */
constexpr uint32_t  TSC_IOGCSR_G4S                      = 0x00080000;            /*!<Analog IO GROUP4 status */
constexpr uint32_t  TSC_IOGCSR_G5S                      = 0x00100000;            /*!<Analog IO GROUP5 status */
constexpr uint32_t  TSC_IOGCSR_G6S                      = 0x00200000;            /*!<Analog IO GROUP6 status */
constexpr uint32_t  TSC_IOGCSR_G7S                      = 0x00400000;            /*!<Analog IO GROUP7 status */
constexpr uint32_t  TSC_IOGCSR_G8S                      = 0x00800000;            /*!<Analog IO GROUP8 status */

/*******************  Bit definition for TSC_IOGXCR register  *****************/
constexpr uint32_t  TSC_IOGXCR_CNT                      = 0x00003FFF;            /*!<CNT[13:0] bits = Counter value; */

/******************************************************************************/
/*                                                                            */
/*      Universal Synchronous Asynchronous Receiver Transmitter = USART;       */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for USART_CR1 register  *******************/
constexpr uint32_t  USART_CR1_UE                        = 0x00000001;            /*!< USART Enable */
constexpr uint32_t  USART_CR1_UESM                      = 0x00000002;            /*!< USART Enable in STOP Mode */
constexpr uint32_t  USART_CR1_RE                        = 0x00000004;            /*!< Receiver Enable */
constexpr uint32_t  USART_CR1_TE                        = 0x00000008;            /*!< Transmitter Enable */
constexpr uint32_t  USART_CR1_IDLEIE                    = 0x00000010;            /*!< IDLE Interrupt Enable */
constexpr uint32_t  USART_CR1_RXNEIE                    = 0x00000020;            /*!< RXNE Interrupt Enable */
constexpr uint32_t  USART_CR1_TCIE                      = 0x00000040;            /*!< Transmission Complete Interrupt Enable */
constexpr uint32_t  USART_CR1_TXEIE                     = 0x00000080;            /*!< TXE Interrupt Enable */
constexpr uint32_t  USART_CR1_PEIE                      = 0x00000100;            /*!< PE Interrupt Enable */
constexpr uint32_t  USART_CR1_PS                        = 0x00000200;            /*!< Parity Selection */
constexpr uint32_t  USART_CR1_PCE                       = 0x00000400;            /*!< Parity Control Enable */
constexpr uint32_t  USART_CR1_WAKE                      = 0x00000800;            /*!< Receiver Wakeup method */
constexpr uint32_t  USART_CR1_M0                        = 0x00001000;            /*!< Word length bit 0 */
constexpr uint32_t  USART_CR1_MME                       = 0x00002000;            /*!< Mute Mode Enable */
constexpr uint32_t  USART_CR1_CMIE                      = 0x00004000;            /*!< Character match interrupt enable */
constexpr uint32_t  USART_CR1_OVER8                     = 0x00008000;            /*!< Oversampling by 8-bit or 16-bit mode */
constexpr uint32_t  USART_CR1_DEDT                      = 0x001F0000;            /*!< DEDT[4:0] bits = Driver Enable Deassertion Time; */
constexpr uint32_t  USART_CR1_DEDT_0                    = 0x00010000;            /*!< Bit 0 */
constexpr uint32_t  USART_CR1_DEDT_1                    = 0x00020000;            /*!< Bit 1 */
constexpr uint32_t  USART_CR1_DEDT_2                    = 0x00040000;            /*!< Bit 2 */
constexpr uint32_t  USART_CR1_DEDT_3                    = 0x00080000;            /*!< Bit 3 */
constexpr uint32_t  USART_CR1_DEDT_4                    = 0x00100000;            /*!< Bit 4 */
constexpr uint32_t  USART_CR1_DEAT                      = 0x03E00000;            /*!< DEAT[4:0] bits = Driver Enable Assertion Time; */
constexpr uint32_t  USART_CR1_DEAT_0                    = 0x00200000;            /*!< Bit 0 */
constexpr uint32_t  USART_CR1_DEAT_1                    = 0x00400000;            /*!< Bit 1 */
constexpr uint32_t  USART_CR1_DEAT_2                    = 0x00800000;            /*!< Bit 2 */
constexpr uint32_t  USART_CR1_DEAT_3                    = 0x01000000;            /*!< Bit 3 */
constexpr uint32_t  USART_CR1_DEAT_4                    = 0x02000000;            /*!< Bit 4 */
constexpr uint32_t  USART_CR1_RTOIE                     = 0x04000000;            /*!< Receive Time Out interrupt enable */
constexpr uint32_t  USART_CR1_EOBIE                     = 0x08000000;            /*!< End of Block interrupt enable */
constexpr uint32_t  USART_CR1_M1                        = 0x10000000;            /*!< Word length bit 1 */
constexpr uint32_t  USART_CR1_M                         = 0x10001000;            /*!< [M1:M0] Word length */

/******************  Bit definition for USART_CR2 register  *******************/
constexpr uint32_t  USART_CR2_ADDM7                     = 0x00000010;            /*!< 7-bit or 4-bit Address Detection */
constexpr uint32_t  USART_CR2_LBDL                      = 0x00000020;            /*!< LIN Break Detection Length */
constexpr uint32_t  USART_CR2_LBDIE                     = 0x00000040;            /*!< LIN Break Detection Interrupt Enable */
constexpr uint32_t  USART_CR2_LBCL                      = 0x00000100;            /*!< Last Bit Clock pulse */
constexpr uint32_t  USART_CR2_CPHA                      = 0x00000200;            /*!< Clock Phase */
constexpr uint32_t  USART_CR2_CPOL                      = 0x00000400;            /*!< Clock Polarity */
constexpr uint32_t  USART_CR2_CLKEN                     = 0x00000800;            /*!< Clock Enable */
constexpr uint32_t  USART_CR2_STOP                      = 0x00003000;            /*!< STOP[1:0] bits = STOP bits; */
constexpr uint32_t  USART_CR2_STOP_0                    = 0x00001000;            /*!< Bit 0 */
constexpr uint32_t  USART_CR2_STOP_1                    = 0x00002000;            /*!< Bit 1 */
constexpr uint32_t  USART_CR2_LINEN                     = 0x00004000;            /*!< LIN mode enable */
constexpr uint32_t  USART_CR2_SWAP                      = 0x00008000;            /*!< SWAP TX/RX pins */
constexpr uint32_t  USART_CR2_RXINV                     = 0x00010000;            /*!< RX pin active level inversion */
constexpr uint32_t  USART_CR2_TXINV                     = 0x00020000;            /*!< TX pin active level inversion */
constexpr uint32_t  USART_CR2_DATAINV                   = 0x00040000;            /*!< Binary data inversion */
constexpr uint32_t  USART_CR2_MSBFIRST                  = 0x00080000;            /*!< Most Significant Bit First */
constexpr uint32_t  USART_CR2_ABREN                     = 0x00100000;            /*!< Auto Baud-Rate Enable*/
constexpr uint32_t  USART_CR2_ABRMODE                   = 0x00600000;            /*!< ABRMOD[1:0] bits = Auto Baud-Rate Mode; */
constexpr uint32_t  USART_CR2_ABRMODE_0                 = 0x00200000;            /*!< Bit 0 */
constexpr uint32_t  USART_CR2_ABRMODE_1                 = 0x00400000;            /*!< Bit 1 */
constexpr uint32_t  USART_CR2_RTOEN                     = 0x00800000;            /*!< Receiver Time-Out enable */
constexpr uint32_t  USART_CR2_ADD                       = 0xFF000000;            /*!< Address of the USART node */

/******************  Bit definition for USART_CR3 register  *******************/
constexpr uint32_t  USART_CR3_EIE                       = 0x00000001;            /*!< Error Interrupt Enable */
constexpr uint32_t  USART_CR3_IREN                      = 0x00000002;            /*!< IrDA mode Enable */
constexpr uint32_t  USART_CR3_IRLP                      = 0x00000004;            /*!< IrDA Low-Power */
constexpr uint32_t  USART_CR3_HDSEL                     = 0x00000008;            /*!< Half-Duplex Selection */
constexpr uint32_t  USART_CR3_NACK                      = 0x00000010;            /*!< SmartCard NACK enable */
constexpr uint32_t  USART_CR3_SCEN                      = 0x00000020;            /*!< SmartCard mode enable */
constexpr uint32_t  USART_CR3_DMAR                      = 0x00000040;            /*!< DMA Enable Receiver */
constexpr uint32_t  USART_CR3_DMAT                      = 0x00000080;            /*!< DMA Enable Transmitter */
constexpr uint32_t  USART_CR3_RTSE                      = 0x00000100;            /*!< RTS Enable */
constexpr uint32_t  USART_CR3_CTSE                      = 0x00000200;            /*!< CTS Enable */
constexpr uint32_t  USART_CR3_CTSIE                     = 0x00000400;            /*!< CTS Interrupt Enable */
constexpr uint32_t  USART_CR3_ONEBIT                    = 0x00000800;            /*!< One sample bit method enable */
constexpr uint32_t  USART_CR3_OVRDIS                    = 0x00001000;            /*!< Overrun Disable */
constexpr uint32_t  USART_CR3_DDRE                      = 0x00002000;            /*!< DMA Disable on Reception Error */
constexpr uint32_t  USART_CR3_DEM                       = 0x00004000;            /*!< Driver Enable Mode */
constexpr uint32_t  USART_CR3_DEP                       = 0x00008000;            /*!< Driver Enable Polarity Selection */
constexpr uint32_t  USART_CR3_SCARCNT                   = 0x000E0000;            /*!< SCARCNT[2:0] bits = SmartCard Auto-Retry Count; */
constexpr uint32_t  USART_CR3_SCARCNT_0                 = 0x00020000;            /*!< Bit 0 */
constexpr uint32_t  USART_CR3_SCARCNT_1                 = 0x00040000;            /*!< Bit 1 */
constexpr uint32_t  USART_CR3_SCARCNT_2                 = 0x00080000;            /*!< Bit 2 */
constexpr uint32_t  USART_CR3_WUS                       = 0x00300000;            /*!< WUS[1:0] bits = Wake UP Interrupt Flag Selection; */
constexpr uint32_t  USART_CR3_WUS_0                     = 0x00100000;            /*!< Bit 0 */
constexpr uint32_t  USART_CR3_WUS_1                     = 0x00200000;            /*!< Bit 1 */
constexpr uint32_t  USART_CR3_WUFIE                     = 0x00400000;            /*!< Wake Up Interrupt Enable */

/******************  Bit definition for USART_BRR register  *******************/
constexpr uint32_t  USART_BRR_DIV_FRACTION              = 0x0000000F;            /*!< Fraction of USARTDIV */
constexpr uint32_t  USART_BRR_DIV_MANTISSA              = 0x0000FFF0;            /*!< Mantissa of USARTDIV */

/******************  Bit definition for USART_GTPR register  ******************/
constexpr uint32_t  USART_GTPR_PSC                      = 0x000000FF;            /*!< PSC[7:0] bits = Prescaler value; */
constexpr uint32_t  USART_GTPR_GT                       = 0x0000FF00;            /*!< GT[7:0] bits = Guard time value; */


/*******************  Bit definition for USART_RTOR register  *****************/
constexpr uint32_t  USART_RTOR_RTO                      = 0x00FFFFFF;            /*!< Receiver Time Out Value */
constexpr uint32_t  USART_RTOR_BLEN                     = 0xFF000000;            /*!< Block Length */

/*******************  Bit definition for USART_RQR register  ******************/
constexpr uint32_t  USART_RQR_ABRRQ                     = 0x00000001;            /*!< Auto-Baud Rate Request */
constexpr uint32_t  USART_RQR_SBKRQ                     = 0x00000002;            /*!< Send Break Request */
constexpr uint32_t  USART_RQR_MMRQ                      = 0x00000004;            /*!< Mute Mode Request */
constexpr uint32_t  USART_RQR_RXFRQ                     = 0x00000008;            /*!< Receive Data flush Request */
constexpr uint32_t  USART_RQR_TXFRQ                     = 0x00000010;            /*!< Transmit data flush Request */

/*******************  Bit definition for USART_ISR register  ******************/
constexpr uint32_t  USART_ISR_PE                        = 0x00000001;            /*!< Parity Error */
constexpr uint32_t  USART_ISR_FE                        = 0x00000002;            /*!< Framing Error */
constexpr uint32_t  USART_ISR_NE                        = 0x00000004;            /*!< Noise detected Flag */
constexpr uint32_t  USART_ISR_ORE                       = 0x00000008;            /*!< OverRun Error */
constexpr uint32_t  USART_ISR_IDLE                      = 0x00000010;            /*!< IDLE line detected */
constexpr uint32_t  USART_ISR_RXNE                      = 0x00000020;            /*!< Read Data Register Not Empty */
constexpr uint32_t  USART_ISR_TC                        = 0x00000040;            /*!< Transmission Complete */
constexpr uint32_t  USART_ISR_TXE                       = 0x00000080;            /*!< Transmit Data Register Empty */
constexpr uint32_t  USART_ISR_LBDF                      = 0x00000100;            /*!< LIN Break Detection Flag */
constexpr uint32_t  USART_ISR_CTSIF                     = 0x00000200;            /*!< CTS interrupt flag */
constexpr uint32_t  USART_ISR_CTS                       = 0x00000400;            /*!< CTS flag */
constexpr uint32_t  USART_ISR_RTOF                      = 0x00000800;            /*!< Receiver Time Out */
constexpr uint32_t  USART_ISR_EOBF                      = 0x00001000;            /*!< End Of Block Flag */
constexpr uint32_t  USART_ISR_ABRE                      = 0x00004000;            /*!< Auto-Baud Rate Error */
constexpr uint32_t  USART_ISR_ABRF                      = 0x00008000;            /*!< Auto-Baud Rate Flag */
constexpr uint32_t  USART_ISR_BUSY                      = 0x00010000;            /*!< Busy Flag */
constexpr uint32_t  USART_ISR_CMF                       = 0x00020000;            /*!< Character Match Flag */
constexpr uint32_t  USART_ISR_SBKF                      = 0x00040000;            /*!< Send Break Flag */
constexpr uint32_t  USART_ISR_RWU                       = 0x00080000;            /*!< Receive Wake Up from mute mode Flag */
constexpr uint32_t  USART_ISR_WUF                       = 0x00100000;            /*!< Wake Up from stop mode Flag */
constexpr uint32_t  USART_ISR_TEACK                     = 0x00200000;            /*!< Transmit Enable Acknowledge Flag */
constexpr uint32_t  USART_ISR_REACK                     = 0x00400000;            /*!< Receive Enable Acknowledge Flag */

/*******************  Bit definition for USART_ICR register  ******************/
constexpr uint32_t  USART_ICR_PECF                      = 0x00000001;            /*!< Parity Error Clear Flag */
constexpr uint32_t  USART_ICR_FECF                      = 0x00000002;            /*!< Framing Error Clear Flag */
constexpr uint32_t  USART_ICR_NCF                       = 0x00000004;            /*!< Noise detected Clear Flag */
constexpr uint32_t  USART_ICR_ORECF                     = 0x00000008;            /*!< OverRun Error Clear Flag */
constexpr uint32_t  USART_ICR_IDLECF                    = 0x00000010;            /*!< IDLE line detected Clear Flag */
constexpr uint32_t  USART_ICR_TCCF                      = 0x00000040;            /*!< Transmission Complete Clear Flag */
constexpr uint32_t  USART_ICR_LBDCF                     = 0x00000100;            /*!< LIN Break Detection Clear Flag */
constexpr uint32_t  USART_ICR_CTSCF                     = 0x00000200;            /*!< CTS Interrupt Clear Flag */
constexpr uint32_t  USART_ICR_RTOCF                     = 0x00000800;            /*!< Receiver Time Out Clear Flag */
constexpr uint32_t  USART_ICR_EOBCF                     = 0x00001000;            /*!< End Of Block Clear Flag */
constexpr uint32_t  USART_ICR_CMCF                      = 0x00020000;            /*!< Character Match Clear Flag */
constexpr uint32_t  USART_ICR_WUCF                      = 0x00100000;            /*!< Wake Up from stop mode Clear Flag */

/*******************  Bit definition for USART_RDR register  ******************/
constexpr uint32_t  USART_RDR_RDR                       = 0x000001FF;            /*!< RDR[8:0] bits = Receive Data value; */

/*******************  Bit definition for USART_TDR register  ******************/
constexpr uint32_t  USART_TDR_TDR                       = 0x000001FF;            /*!< TDR[8:0] bits = Transmit Data value; */

/******************************************************************************/
/*                                                                            */
/*                         USB Device General registers                       */
/*                                                                            */
/******************************************************************************/
constexpr uint32_t USB_CNTR                           = USB_BASE + 0x40;             /*!< Control register */
constexpr uint32_t USB_ISTR                           = USB_BASE + 0x44;             /*!< Interrupt status register */
constexpr uint32_t USB_FNR                            = USB_BASE + 0x48;             /*!< Frame number register */
constexpr uint32_t USB_DADDR                          = USB_BASE + 0x4C;             /*!< Device address register */
constexpr uint32_t USB_BTABLE                         = USB_BASE + 0x50;             /*!< Buffer Table address register */
constexpr uint32_t USB_LPMCSR                         = USB_BASE + 0x54;             /*!< LPM Control and Status register */

/****************************  ISTR interrupt events  *************************/
constexpr uint16_t USB_ISTR_CTR                         = 0x8000;             /*!< Correct TRansfer = clear-only bit; */
constexpr uint16_t USB_ISTR_PMAOVRM                     = 0x4000;             /*!< DMA OVeR/underrun = clear-only bit; */
constexpr uint16_t USB_ISTR_ERR                         = 0x2000;             /*!< ERRor = clear-only bit; */
constexpr uint16_t USB_ISTR_WKUP                        = 0x1000;             /*!< WaKe UP = clear-only bit; */
constexpr uint16_t USB_ISTR_SUSP                        = 0x0800;             /*!< SUSPend = clear-only bit; */
constexpr uint16_t USB_ISTR_RESET                       = 0x0400;             /*!< RESET = clear-only bit; */
constexpr uint16_t USB_ISTR_SOF                         = 0x0200;             /*!< Start Of Frame = clear-only bit; */
constexpr uint16_t USB_ISTR_ESOF                        = 0x0100;             /*!< Expected Start Of Frame = clear-only bit; */
constexpr uint16_t USB_ISTR_L1REQ                       = 0x0080;             /*!< LPM L1 state request  */
constexpr uint16_t USB_ISTR_DIR                         = 0x0010;             /*!< DIRection of transaction = read-only bit;  */
constexpr uint16_t USB_ISTR_EP_ID                       = 0x000F;             /*!< EndPoint IDentifier = read-only bit;  */

constexpr uint32_t USB_CLR_CTR                          = ~USB_ISTR_CTR;             /*!< clear Correct TRansfer bit */
constexpr uint32_t USB_CLR_PMAOVRM                      = ~USB_ISTR_PMAOVRM;          /*!< clear DMA OVeR/underrun bit*/
constexpr uint32_t USB_CLR_ERR                          = ~USB_ISTR_ERR;             /*!< clear ERRor bit */
constexpr uint32_t USB_CLR_WKUP                         = ~USB_ISTR_WKUP;            /*!< clear WaKe UP bit */
constexpr uint32_t USB_CLR_SUSP                         = ~USB_ISTR_SUSP;            /*!< clear SUSPend bit */
constexpr uint32_t USB_CLR_RESET                        = ~USB_ISTR_RESET;           /*!< clear RESET bit */
constexpr uint32_t USB_CLR_SOF                          = ~USB_ISTR_SOF;             /*!< clear Start Of Frame bit */
constexpr uint32_t USB_CLR_ESOF                         = ~USB_ISTR_ESOF;            /*!< clear Expected Start Of Frame bit */
constexpr uint32_t USB_CLR_L1REQ                        = ~USB_ISTR_L1REQ;           /*!< clear LPM L1  bit */

/*************************  CNTR control register bits definitions  ***********/
constexpr uint16_t USB_CNTR_CTRM                        = 0x8000;             /*!< Correct TRansfer Mask */
constexpr uint16_t USB_CNTR_PMAOVRM                     = 0x4000;             /*!< DMA OVeR/underrun Mask */
constexpr uint16_t USB_CNTR_ERRM                        = 0x2000;             /*!< ERRor Mask */
constexpr uint16_t USB_CNTR_WKUPM                       = 0x1000;             /*!< WaKe UP Mask */
constexpr uint16_t USB_CNTR_SUSPM                       = 0x0800;             /*!< SUSPend Mask */
constexpr uint16_t USB_CNTR_RESETM                      = 0x0400;             /*!< RESET Mask   */
constexpr uint16_t USB_CNTR_SOFM                        = 0x0200;             /*!< Start Of Frame Mask */
constexpr uint16_t USB_CNTR_ESOFM                       = 0x0100;             /*!< Expected Start Of Frame Mask */
constexpr uint16_t USB_CNTR_L1REQM                      = 0x0080;             /*!< LPM L1 state request interrupt mask */
constexpr uint16_t USB_CNTR_L1RESUME                    = 0x0020;             /*!< LPM L1 Resume request */
constexpr uint16_t USB_CNTR_RESUME                      = 0x0010;             /*!< RESUME request */
constexpr uint16_t USB_CNTR_FSUSP                       = 0x0008;             /*!< Force SUSPend */
constexpr uint16_t USB_CNTR_LP_MODE                     = 0x0004;             /*!< Low-power MODE */
constexpr uint16_t USB_CNTR_PDWN                        = 0x0002;             /*!< Power DoWN */
constexpr uint16_t USB_CNTR_FRES                        = 0x0001;             /*!< Force USB RESet */

/***************************  LPM register bits definitions  ******************/
constexpr uint16_t  USB_LPMCSR_BESL                     = 0x00F0;             /*!< BESL value received with last ACKed LPM Token  */
constexpr uint16_t  USB_LPMCSR_REMWAKE                  = 0x0008;             /*!< bRemoteWake value received with last ACKed LPM Token */
constexpr uint16_t  USB_LPMCSR_LPMACK                   = 0x0002;             /*!< LPM Token acknowledge enable*/
constexpr uint16_t  USB_LPMCSR_LMPEN                    = 0x0001;             /*!< LPM support enable  */

/********************  FNR Frame Number Register bit definitions   ************/
constexpr uint16_t USB_FNR_RXDP                         = 0x8000;             /*!< status of D+ data line */
constexpr uint16_t USB_FNR_RXDM                         = 0x4000;             /*!< status of D- data line */
constexpr uint16_t USB_FNR_LCK                          = 0x2000;             /*!< LoCKed */
constexpr uint16_t USB_FNR_LSOF                         = 0x1800;             /*!< Lost SOF */
constexpr uint16_t USB_FNR_FN                           = 0x07FF;             /*!< Frame Number */

/********************  DADDR Device ADDRess bit definitions    ****************/
constexpr uint8_t USB_DADDR_EF                         = 0x80;                /*!< USB device address Enable Function */
constexpr uint8_t USB_DADDR_ADD                        = 0x7F;                /*!< USB device address */

/******************************  Endpoint register    *************************/
constexpr uint32_t USB_EP0R                             = USB_BASE;                  /*!< endpoint 0 register address */
constexpr uint32_t USB_EP1R                             = USB_BASE + 0x04;           /*!< endpoint 1 register address */
constexpr uint32_t USB_EP2R                             = USB_BASE + 0x08;           /*!< endpoint 2 register address */
constexpr uint32_t USB_EP3R                             = USB_BASE + 0x0C;           /*!< endpoint 3 register address */
constexpr uint32_t USB_EP4R                             = USB_BASE + 0x10;           /*!< endpoint 4 register address */
constexpr uint32_t USB_EP5R                             = USB_BASE + 0x14;           /*!< endpoint 5 register address */
constexpr uint32_t USB_EP6R                             = USB_BASE + 0x18;           /*!< endpoint 6 register address */
constexpr uint32_t USB_EP7R                             = USB_BASE + 0x1C;           /*!< endpoint 7 register address */
/* bit positions */
constexpr uint16_t USB_EP_CTR_RX                        = 0x8000;             /*!<  EndPoint Correct TRansfer RX */
constexpr uint16_t USB_EP_DTOG_RX                       = 0x4000;             /*!<  EndPoint Data TOGGLE RX */
constexpr uint16_t USB_EPRX_STAT                        = 0x3000;             /*!<  EndPoint RX STATus bit field */
constexpr uint16_t USB_EP_SETUP                         = 0x0800;             /*!<  EndPoint SETUP */
constexpr uint16_t USB_EP_T_FIELD                       = 0x0600;             /*!<  EndPoint TYPE */
constexpr uint16_t USB_EP_KIND                          = 0x0100;             /*!<  EndPoint KIND */
constexpr uint16_t USB_EP_CTR_TX                        = 0x0080;             /*!<  EndPoint Correct TRansfer TX */
constexpr uint16_t USB_EP_DTOG_TX                       = 0x0040;             /*!<  EndPoint Data TOGGLE TX */
constexpr uint16_t USB_EPTX_STAT                        = 0x0030;             /*!<  EndPoint TX STATus bit field */
constexpr uint16_t USB_EPADDR_FIELD                     = 0x000F;             /*!<  EndPoint ADDRess FIELD */

/* EndPoint REGister MASK = no toggle fields; */
constexpr uint32_t USB_EPREG_MASK     = USB_EP_CTR_RX|USB_EP_SETUP|USB_EP_T_FIELD|USB_EP_KIND|USB_EP_CTR_TX|USB_EPADDR_FIELD;
                                                                               /*!< EP_TYPE[1:0] EndPoint TYPE */
constexpr uint16_t USB_EP_TYPE_MASK                     = 0x0600;             /*!< EndPoint TYPE Mask */
constexpr uint16_t USB_EP_BULK                          = 0x0000;             /*!< EndPoint BULK */
constexpr uint16_t USB_EP_CONTROL                       = 0x0200;             /*!< EndPoint CONTROL */
constexpr uint16_t USB_EP_ISOCHRONOUS                   = 0x0400;             /*!< EndPoint ISOCHRONOUS */
constexpr uint16_t USB_EP_INTERRUPT                     = 0x0600;             /*!< EndPoint INTERRUPT */
constexpr uint32_t USB_EP_T_MASK                        = ~USB_EP_T_FIELD & USB_EPREG_MASK;

constexpr uint32_t USB_EPKIND_MASK    = ~USB_EP_KIND & USB_EPREG_MASK;            /*!< EP_KIND EndPoint KIND */
                                                                               /*!< STAT_TX[1:0] STATus for TX transfer */
constexpr uint16_t USB_EP_TX_DIS                        = 0x0000;             /*!< EndPoint TX DISabled */
constexpr uint16_t USB_EP_TX_STALL                      = 0x0010;             /*!< EndPoint TX STALLed */
constexpr uint16_t USB_EP_TX_NAK                        = 0x0020;             /*!< EndPoint TX NAKed */
constexpr uint16_t USB_EP_TX_VALID                      = 0x0030;             /*!< EndPoint TX VALID */
constexpr uint16_t USB_EPTX_DTOG1                       = 0x0010;             /*!< EndPoint TX Data TOGgle bit1 */
constexpr uint16_t USB_EPTX_DTOG2                       = 0x0020;             /*!< EndPoint TX Data TOGgle bit2 */
constexpr uint32_t USB_EPTX_DTOGMASK  = USB_EPTX_STAT|USB_EPREG_MASK;
                                                                               /*!< STAT_RX[1:0] STATus for RX transfer */
constexpr uint16_t USB_EP_RX_DIS                        = 0x0000;             /*!< EndPoint RX DISabled */
constexpr uint16_t USB_EP_RX_STALL                      = 0x1000;             /*!< EndPoint RX STALLed */
constexpr uint16_t USB_EP_RX_NAK                        = 0x2000;             /*!< EndPoint RX NAKed */
constexpr uint16_t USB_EP_RX_VALID                      = 0x3000;             /*!< EndPoint RX VALID */
constexpr uint16_t USB_EPRX_DTOG1                       = 0x1000;             /*!< EndPoint RX Data TOGgle bit1 */
constexpr uint16_t USB_EPRX_DTOG2                       = 0x2000;             /*!< EndPoint RX Data TOGgle bit1 */
constexpr uint32_t USB_EPRX_DTOGMASK  = USB_EPRX_STAT|USB_EPREG_MASK;

/******************************************************************************/
/*                                                                            */
/*                            Window WATCHDOG                                 */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for WWDG_CR register  ********************/
constexpr uint32_t  WWDG_CR_T                           = 0x0000007F;        /*!<T[6:0] bits = 7-Bit counter = MSB to LSB;; */
constexpr uint32_t  WWDG_CR_T0                          = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  WWDG_CR_T1                          = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  WWDG_CR_T2                          = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  WWDG_CR_T3                          = 0x00000008;        /*!<Bit 3 */
constexpr uint32_t  WWDG_CR_T4                          = 0x00000010;        /*!<Bit 4 */
constexpr uint32_t  WWDG_CR_T5                          = 0x00000020;        /*!<Bit 5 */
constexpr uint32_t  WWDG_CR_T6                          = 0x00000040;        /*!<Bit 6 */

constexpr uint32_t  WWDG_CR_WDGA                        = 0x00000080;        /*!<Activation bit */

/*******************  Bit definition for WWDG_CFR register  *******************/
constexpr uint32_t  WWDG_CFR_W                          = 0x0000007F;        /*!<W[6:0] bits = 7-bit window value; */
constexpr uint32_t  WWDG_CFR_W0                         = 0x00000001;        /*!<Bit 0 */
constexpr uint32_t  WWDG_CFR_W1                         = 0x00000002;        /*!<Bit 1 */
constexpr uint32_t  WWDG_CFR_W2                         = 0x00000004;        /*!<Bit 2 */
constexpr uint32_t  WWDG_CFR_W3                         = 0x00000008;        /*!<Bit 3 */
constexpr uint32_t  WWDG_CFR_W4                         = 0x00000010;        /*!<Bit 4 */
constexpr uint32_t  WWDG_CFR_W5                         = 0x00000020;        /*!<Bit 5 */
constexpr uint32_t  WWDG_CFR_W6                         = 0x00000040;        /*!<Bit 6 */

constexpr uint32_t  WWDG_CFR_WDGTB                      = 0x00000180;        /*!<WDGTB[1:0] bits = Timer Base; */
constexpr uint32_t  WWDG_CFR_WDGTB0                     = 0x00000080;        /*!<Bit 0 */
constexpr uint32_t  WWDG_CFR_WDGTB1                     = 0x00000100;        /*!<Bit 1 */

constexpr uint32_t  WWDG_CFR_EWI                        = 0x00000200;        /*!<Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_SR register  ********************/
constexpr uint32_t  WWDG_SR_EWIF                        = 0x00000001;        /*!<Early Wakeup Interrupt Flag */


#endif /* __STM32F303xE_H */
/************************ = C; COPYRIGHT STMicroelectronics *****END OF FILE*****/
