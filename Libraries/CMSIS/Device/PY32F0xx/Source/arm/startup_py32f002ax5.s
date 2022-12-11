;****************************************************************************** 
;* @file              : startup_py32f002ax5.s
;* @brief             : PY32F002Axx devices vector table for MDK-ARM toolchain.
;*                      This module performs:
;*                      - Set the initial SP
;*                      - Set the initial PC == Reset_Handler
;*                      - Set the vector table entries with the exceptions ISR address
;*                      - Branches to __main in the C library (which eventually
;*                        calls main()).
;*                      After Reset the CortexM0+ processor is in Thread mode,
;*                      priority is Privileged, and the Stack is set to Main.
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

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000000

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
                DCD     0                              ; 0Reserved 
                DCD     0                              ; 1Reserved 
                DCD     0                              ; 2Reserved 
                DCD     FLASH_IRQHandler               ; 3FLASH
                DCD     RCC_IRQHandler                 ; 4RCC
                DCD     EXTI0_1_IRQHandler             ; 5EXTI Line 0 and 1
                DCD     EXTI2_3_IRQHandler             ; 6EXTI Line 2 and 3
                DCD     EXTI4_15_IRQHandler            ; 7EXTI Line 4 to 15
                DCD     0                              ; 8Reserved 
                DCD     0                              ; 9Reserved 
                DCD     0                              ; 10Reserved 
                DCD     0                              ; 11Reserved 
                DCD     ADC_COMP_IRQHandler            ; 12ADC&COMP1 
                DCD     TIM1_BRK_UP_TRG_COM_IRQHandler ; 13TIM1 Break, Update, Trigger and Commutation
                DCD     TIM1_CC_IRQHandler             ; 14TIM1 Capture Compare
                DCD     0                              ; 15Reserved 
                DCD     0                              ; 16Reserved 
                DCD     LPTIM1_IRQHandler              ; 17LPTIM1
                DCD     0                              ; 18Reserved 
                DCD     0                              ; 19Reserved 
                DCD     0                              ; 20Reserved 
                DCD     TIM16_IRQHandler               ; 21TIM16
                DCD     0                              ; 22Reserved 
                DCD     I2C1_IRQHandler                ; 23I2C1
                DCD     0                              ; 24Reserved 
                DCD     SPI1_IRQHandler                ; 25SPI1
                DCD     0                              ; 26Reserved
                DCD     USART1_IRQHandler              ; 27USART1
                DCD     0                              ; 28Reserved 
                DCD     0                              ; 29Reserved
                DCD     0                              ; 30Reserved
                DCD     0                              ; 31Reserved
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

                EXPORT  FLASH_IRQHandler               [WEAK]
                EXPORT  RCC_IRQHandler                 [WEAK]
                EXPORT  EXTI0_1_IRQHandler             [WEAK]
                EXPORT  EXTI2_3_IRQHandler             [WEAK]
                EXPORT  EXTI4_15_IRQHandler            [WEAK]
                EXPORT  ADC_COMP_IRQHandler            [WEAK]
                EXPORT  TIM1_BRK_UP_TRG_COM_IRQHandler [WEAK]
                EXPORT  TIM1_CC_IRQHandler             [WEAK]
                EXPORT  LPTIM1_IRQHandler              [WEAK]
                EXPORT  TIM16_IRQHandler               [WEAK]
                EXPORT  I2C1_IRQHandler                [WEAK]
                EXPORT  SPI1_IRQHandler                [WEAK]
                EXPORT  USART1_IRQHandler              [WEAK]

FLASH_IRQHandler              
RCC_IRQHandler                
EXTI0_1_IRQHandler             
EXTI2_3_IRQHandler             
EXTI4_15_IRQHandler   
ADC_COMP_IRQHandler          
TIM1_BRK_UP_TRG_COM_IRQHandler
TIM1_CC_IRQHandler
LPTIM1_IRQHandler
TIM16_IRQHandler
I2C1_IRQHandler
SPI1_IRQHandler
USART1_IRQHandler
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
