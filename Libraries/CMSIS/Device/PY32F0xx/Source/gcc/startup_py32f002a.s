/**
  ******************************************************************************
  * @file      startup_py32f002a.s
  * @brief     PY32F002 devices vector table for GCC toolchain.
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Branches to main in the C library (which eventually calls main()).
  *            After Reset the Cortex-M0 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  */

  .syntax unified
  .cpu cortex-m0plus
  .fpu softvfp
  .thumb

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit
  
/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Call the clock system intitialization function.*/
  bl  SystemInit
/* Call static constructors. Remove this line if compile with `-nostartfiles` reports error */
  bl __libc_init_array
/* Call the application's entry point.*/
  bl main

LoopForever:
    b LoopForever


.size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex M0.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
   .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors


g_pfnVectors:
  .word  _estack
  .word  Reset_Handler                  /* Reset Handler */
  .word  NMI_Handler                    /* NMI Handler */
  .word  HardFault_Handler              /* Hard Fault Handler */
  .word  0                              /* Reserved */
  .word  0                              /* Reserved */
  .word  0                              /* Reserved */
  .word  0                              /* Reserved */
  .word  0                              /* Reserved */
  .word  0                              /* Reserved */
  .word  0                              /* Reserved */
  .word  SVC_Handler                    /* SVCall Handler */
  .word  0                              /* Reserved */
  .word  0                              /* Reserved */
  .word  PendSV_Handler                 /* PendSV Handler */
  .word  SysTick_Handler                /* SysTick Handler */
  .word  0                              /* 0Reserved */
  .word  0                              /* 1Reserved */
  .word  0                              /* 2Reserved */
  .word  FLASH_IRQHandler               /* 3FLASH */
  .word  RCC_IRQHandler                 /* 4RCC */
  .word  EXTI0_1_IRQHandler             /* 5EXTI Line 0 and 1 */
  .word  EXTI2_3_IRQHandler             /* 6EXTI Line 2 and 3 */
  .word  EXTI4_15_IRQHandler            /* 7EXTI Line 4 to 15 */
  .word  0                              /* 8Reserved  */
  .word  0                              /* 9Reserved */
  .word  0                              /* 10Reserved */
  .word  0                              /* 11Reserved */
  .word  ADC_IRQHandler                 /* 12ADC */
  .word  TIM1_BRK_UP_TRG_COM_IRQHandler /* 13TIM1 Break, Update, Trigger and Commutation */
  .word  TIM1_CC_IRQHandler             /* 14TIM1 Capture Compare */
  .word  0                              /* 15Reserved  */
  .word  0                              /* 16Reserved */
  .word  LPTIM1_IRQHandler              /* 17LPTIM1 */
  .word  0                              /* 18Reserved  */
  .word  0                              /* 19Reserved */
  .word  0                              /* 20Reserved  */
  .word  TIM16_IRQHandler               /* 21TIM16 */
  .word  0                              /* 22Reserved */
  .word  I2C1_IRQHandler                /* 23I2C1 */
  .word  0                              /* 24Reserved  */
  .word  SPI1_IRQHandler                /* 25SPI1 */
  .word  0                              /* 26Reserved */
  .word  USART1_IRQHandler              /* 27USART1 */
  .word  0                              /* 28Reserved */
  .word  0                              /* 29Reserved */
  .word  0                              /* 30Reserved */
  .word  0                              /* 31Reserved */

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  .weak      NMI_Handler                
  .thumb_set NMI_Handler,Default_Handler

  .weak      HardFault_Handler                
  .thumb_set HardFault_Handler,Default_Handler

  .weak      SVC_Handler                
  .thumb_set SVC_Handler,Default_Handler

  .weak      PendSV_Handler                
  .thumb_set PendSV_Handler,Default_Handler

  .weak      SysTick_Handler                
  .thumb_set SysTick_Handler,Default_Handler

  .weak      FLASH_IRQHandler                
  .thumb_set FLASH_IRQHandler,Default_Handler

  .weak      RCC_IRQHandler                
  .thumb_set RCC_IRQHandler,Default_Handler

  .weak      EXTI0_1_IRQHandler                
  .thumb_set EXTI0_1_IRQHandler,Default_Handler

  .weak      EXTI2_3_IRQHandler                
  .thumb_set EXTI2_3_IRQHandler,Default_Handler

  .weak      EXTI4_15_IRQHandler                
  .thumb_set EXTI4_15_IRQHandler,Default_Handler

  .weak      ADC_IRQHandler                
  .thumb_set ADC_IRQHandler,Default_Handler

  .weak      TIM1_BRK_UP_TRG_COM_IRQHandler                
  .thumb_set TIM1_BRK_UP_TRG_COM_IRQHandler,Default_Handler

  .weak      TIM1_CC_IRQHandler                
  .thumb_set TIM1_CC_IRQHandler,Default_Handler

  .weak      LPTIM1_IRQHandler                
  .thumb_set LPTIM1_IRQHandler,Default_Handler

  .weak      TIM16_IRQHandler                
  .thumb_set TIM16_IRQHandler,Default_Handler

  .weak      I2C1_IRQHandler                
  .thumb_set I2C1_IRQHandler,Default_Handler

  .weak      SPI1_IRQHandler                
  .thumb_set SPI1_IRQHandler,Default_Handler

  .weak      USART1_IRQHandler                
  .thumb_set USART1_IRQHandler,Default_Handler
