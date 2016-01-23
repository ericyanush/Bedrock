//
//  Interrupts.hpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-15.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#ifndef Interrupts_h
#define Interrupts_h

enum class InterruptVector : int32_t
{
    /** Cortex-M4 Processor Exceptions */
    NonMaskableInt         = -14,
    HardFault              = -13,
    MemoryManagement       = -12,                            
    BusFault               = -11,                            
    UsageFault             = -10,                            
    SupervisorCall         = -5,
    DebugMonitor           = -4,                             
    PendSV                 = -2,                             
    SysTick                = -1,
    /**  STM32 specific Interrupt Numbers */
    WindowWatchDog               = 0,     
    ProgrammableVoltageDetector  = 1,     
    TamperStamp                  = 2,     
    RTCWakeup                    = 3,     
    Flash                        = 4,     
    RCC                          = 5,     
    ExternalInt0                 = 6,     
    ExternalInt1                 = 7,     
    ExternalInt2_TouchSense      = 8,     
    ExternalInt3                 = 9,
    ExternalInt4                 = 10,     
    DMA1_Channel1                = 11,     
    DMA1_Channel2                = 12,     
    DMA1_Channel3                = 13,     
    DMA1_Channel4                = 14,     
    DMA1_Channel5                = 15,     
    DMA1_Channel6                = 16,     
    DMA1_Channel7                = 17,     
    ADC1_2                       = 18,     
    USB_HP_CAN_TX                = 19,     
    USB_LP_CAN_RXFIFO0           = 20,     
    CAN_RXFIFO1                  = 21,     
    CANStatusChange              = 22,     
    ExternalInt9_5               = 23,     
    Timer1Break_Timer15          = 24,     
    Timer1Update_Timer16         = 25,     
    Timer1_TrigCom_Timer17       = 26,     
    Timer1_CapCom                = 27,     
    Timer2                       = 28,     
    Timer3                       = 29,     
    Timer4                       = 30,     
    I2C1Event                    = 31,     
    I2C1Error                    = 32,     
    I2C2Event                    = 33,     
    I2C2Error                    = 34,     
    SPI1                         = 35,     
    SPI2                         = 36,     
    USART1                       = 37,     
    USART2                       = 38,     
    USART3                       = 39,     
    EXTI15_10                    = 40,     
    RTCAlarm                     = 41,
    USBWakeUp                    = 42,     
    Timer8Break                  = 43,     
    Timer8Update                 = 44,     
    Timer8_TrigCom               = 45,     
    Timer8_CapComp               = 46,     
    ADC3                         = 47,     
    FMC                          = 48,     
    SPI3                         = 51,     
    UART4                        = 52,     
    UART5                        = 53,     
    Timer6_DACUnderrun           = 54,     
    Timer7                       = 55,     
    DMA2_Channel1                = 56,     
    DMA2_Channel2                = 57,     
    DMA2_Channel3                = 58,     
    DMA2_Channel4                = 59,     
    DMA2_Channel5                = 60,     
    ADC4                         = 61,     
    Comparator1_2_3              = 64,     
    Comparator4_5_6              = 65,     
    Comparator7                  = 66,     
    I2C3Event                    = 72,     
    I2C3Error                    = 73,     
    USB_HP                       = 74,     
    USB_LP                       = 75,     
    USBWakeUp_RMP                = 76,     
    Timer20Break                 = 77,     
    Timer20Update                = 78,     
    Timer20_TrigCom              = 79,     
    Timer20_CapCom               = 80,     
    FPU                          = 81,     
    SPI4                         = 84,
};

constexpr uint32_t MAX_VECTOR = 84;

#endif /* Interrupts_h */
