;****************************************************************************** 
;* @file    startup_py32f072xx.s
;* @author  MCU Application Team
;* @brief   PY32F072xx devices vector table for MDK-ARM toolchain.
;*          This module performs:
;*          - Set the initial SP
;*          - Set the initial PC == Reset_Handler
;*          - Set the vector table entries with the exceptions ISR address
;*          - Branches to __main in the C library (which eventually
;*            calls main()).
;*          After Reset the CortexM0+ processor is in Thread mode,
;*          priority is Privileged, and the Stack is set to Main.
;****************************************************************************** 
;* @attention
;*
;* <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
;* All rights reserved.</center></h2>
;*
;* <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
;* All rights reserved.</center></h2>
;*
;* This software component is licensed by ST under BSD 3-Clause license,
;* the "License"; You may not use this file except in compliance with the
;* License. You may obtain a copy of the License at:
;*                        opensource.org/licenses/BSD-3-Clause
;*
;****************************************************************************** 
;* <<< Use Configuration Wizard in Context Menu >>>

; Amount of memory (in bytes) allocated for Stack
; Tailor this value to your application needs
; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000200

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000200

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                DCD     WWDG_IRQHandler                ; 0Window Watchdog
                DCD     PVD_IRQHandler                 ; 1PVD through EXTI Line detect
                DCD     RTC_IRQHandler                 ; 2RTC through EXTI Line
                DCD     FLASH_IRQHandler               ; 3FLASH
                DCD     RCC_IRQHandler                 ; 4RCC
                DCD     EXTI0_1_IRQHandler             ; 5EXTI Line 0 and 1
                DCD     EXTI2_3_IRQHandler             ; 6EXTI Line 2 and 3
                DCD     EXTI4_15_IRQHandler            ; 7EXTI Line 4 to 15
                DCD     LCD_IRQHandler                 ; 8LCD 
                DCD     DMA1_Channel1_IRQHandler       ; 9DMA1 Channel 1
                DCD     DMA1_Channel2_3_IRQHandler     ; 10DMA1 Channel 2 and Channel 3
                DCD     DMA1_Channel4_5_6_7_IRQHandler ; 11DMA1 Channel 4, Channel 5, Channel 6, Channel 7
                DCD     ADC_COMP_IRQHandler            ; 12ADC&COMP1 
                DCD     TIM1_BRK_UP_TRG_COM_IRQHandler ; 13TIM1 Break, Update, Trigger and Commutation
                DCD     TIM1_CC_IRQHandler             ; 14TIM1 Capture Compare
                DCD     TIM2_IRQHandler                ; 15TIM2
                DCD     TIM3_IRQHandler                ; 16TIM3
                DCD     TIM6_LPTIM1_DAC_IRQHandler     ; 17TIM6&LPTIM1&DAC
                DCD     TIM7_IRQHandler                ; 18TIM7
                DCD     TIM14_IRQHandler               ; 19TIM14
                DCD     TIM15_IRQHandler               ; 20TIM15
                DCD     TIM16_IRQHandler               ; 21TIM16
                DCD     TIM17_IRQHandler               ; 22TIM17
                DCD     I2C1_IRQHandler                ; 23I2C1
                DCD     I2C2_IRQHandler                ; 24I2C2
                DCD     SPI1_IRQHandler                ; 25SPI1
                DCD     SPI2_IRQHandler                ; 26SPI2
                DCD     USART1_IRQHandler              ; 27USART1
                DCD     USART2_IRQHandler              ; 28USART2
                DCD     USART3_4_IRQHandler            ; 29USART3&USART4
                DCD     CAN_IRQHandler                 ; 30CAN
                DCD     USB_IRQHandler                 ; 31USB
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  WWDG_IRQHandler                [WEAK]
                EXPORT  PVD_IRQHandler                 [WEAK]
                EXPORT  RTC_IRQHandler                 [WEAK]
                EXPORT  FLASH_IRQHandler               [WEAK]
                EXPORT  RCC_IRQHandler                 [WEAK]
                EXPORT  EXTI0_1_IRQHandler             [WEAK]
                EXPORT  EXTI2_3_IRQHandler             [WEAK]
                EXPORT  EXTI4_15_IRQHandler            [WEAK]   
                EXPORT  LCD_IRQHandler                 [WEAK]
                EXPORT  DMA1_Channel1_IRQHandler       [WEAK]        
                EXPORT  DMA1_Channel2_3_IRQHandler     [WEAK]
                EXPORT  DMA1_Channel4_5_6_7_IRQHandler [WEAK]
                EXPORT  ADC_COMP_IRQHandler            [WEAK]
                EXPORT  TIM1_BRK_UP_TRG_COM_IRQHandler [WEAK]
                EXPORT  TIM1_CC_IRQHandler             [WEAK]
                EXPORT  TIM2_IRQHandler                [WEAK]
                EXPORT  TIM3_IRQHandler                [WEAK]
                EXPORT  TIM6_LPTIM1_DAC_IRQHandler     [WEAK]
                EXPORT  TIM7_IRQHandler                [WEAK]
                EXPORT  TIM14_IRQHandler               [WEAK]
                EXPORT  TIM15_IRQHandler               [WEAK]
                EXPORT  TIM16_IRQHandler               [WEAK]
                EXPORT  TIM17_IRQHandler               [WEAK]
                EXPORT  I2C1_IRQHandler                [WEAK]
                EXPORT  I2C2_IRQHandler                [WEAK]
                EXPORT  SPI1_IRQHandler                [WEAK]
                EXPORT  SPI2_IRQHandler                [WEAK]
                EXPORT  USART1_IRQHandler              [WEAK]
                EXPORT  USART2_IRQHandler              [WEAK]
                EXPORT  USART3_4_IRQHandler            [WEAK]
                EXPORT  CAN_IRQHandler                 [WEAK]
                EXPORT  USB_IRQHandler                 [WEAK]

WWDG_IRQHandler               
PVD_IRQHandler                
RTC_IRQHandler                
FLASH_IRQHandler              
RCC_IRQHandler                
EXTI0_1_IRQHandler            
EXTI2_3_IRQHandler            
EXTI4_15_IRQHandler           
LCD_IRQHandler                
DMA1_Channel1_IRQHandler      
DMA1_Channel2_3_IRQHandler    
DMA1_Channel4_5_6_7_IRQHandler
ADC_COMP_IRQHandler           
TIM1_BRK_UP_TRG_COM_IRQHandler
TIM1_CC_IRQHandler            
TIM2_IRQHandler               
TIM3_IRQHandler               
TIM6_LPTIM1_DAC_IRQHandler    
TIM7_IRQHandler               
TIM14_IRQHandler              
TIM15_IRQHandler              
TIM16_IRQHandler              
TIM17_IRQHandler              
I2C1_IRQHandler               
I2C2_IRQHandler               
SPI1_IRQHandler               
SPI2_IRQHandler               
USART1_IRQHandler             
USART2_IRQHandler             
USART3_4_IRQHandler           
CAN_IRQHandler                
USB_IRQHandler                
                B       .
                ENDP

                ALIGN

; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
                    
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF

                END

;************************ (C) COPYRIGHT Puya *****END OF FILE*******************
