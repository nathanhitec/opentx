/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _HAL_H_
#define _HAL_H_

// Keys
#if defined(GCSV2)
  #define KEYS_GPIO_REG_MENU            GPIOD->IDR
  #define KEYS_GPIO_PIN_MENU            GPIO_Pin_7  // PD.07
  #define KEYS_GPIO_REG_EXIT            GPIOD->IDR
  #define KEYS_GPIO_PIN_EXIT            GPIO_Pin_2  // PD.02
  #define KEYS_GPIO_REG_PAGE            GPIOD->IDR
  #define KEYS_GPIO_PIN_PAGE            GPIO_Pin_3  // PD.03
  #define KEYS_GPIO_REG_PLUS            GPIOE->IDR
  #define KEYS_GPIO_PIN_PLUS            GPIO_Pin_10 // PE.10
  #define KEYS_GPIO_REG_MINUS           GPIOE->IDR
  #define KEYS_GPIO_PIN_MINUS           GPIO_Pin_11 // PE.11
  #define KEYS_GPIO_REG_ENTER           GPIOB->IDR
  #define KEYS_GPIO_PIN_ENTER           GPIO_Pin_3  // PB.3
#endif

// Rotary Encoder
#if defined(GCSV2)
#define ROTARY_ENCODER_NAVIGATION
  #define ROTARY_ENCODER_GPIO           GPIOD
  #define ROTARY_ENCODER_GPIO_PIN_A     GPIO_Pin_12 // PD.12
  #define ROTARY_ENCODER_GPIO_PIN_B     GPIO_Pin_13 // PD.13
  #define ROTARY_ENCODER_POSITION()     (ROTARY_ENCODER_GPIO->IDR >> 12) & 0x03
  #define ROTARY_ENCODER_EXTI_LINE1     EXTI_Line12
  #define ROTARY_ENCODER_EXTI_LINE2     EXTI_Line13
  #define ROTARY_ENCODER_EXTI_IRQn1        EXTI15_10_IRQn
  #define ROTARY_ENCODER_EXTI_IRQHandler1  EXTI15_10_IRQHandler
  #define ROTARY_ENCODER_EXTI_PortSource   EXTI_PortSourceGPIOD
  #define ROTARY_ENCODER_EXTI_PinSource1   EXTI_PinSource12
  #define ROTARY_ENCODER_EXTI_PinSource2   EXTI_PinSource13
#endif

#if defined(ROTARY_ENCODER_NAVIGATION)
#define ROTARY_ENCODER_RCC_APB1Periph   RCC_APB1Periph_TIM5
#define ROTARY_ENCODER_TIMER            TIM5
#define ROTARY_ENCODER_TIMER_IRQn       TIM5_IRQn
#define ROTARY_ENCODER_TIMER_IRQHandler TIM5_IRQHandler
#else
  #define ROTARY_ENCODER_RCC_APB1Periph   0
#endif


//TODO: do I need to configure 3pos switches
// Switches
#if defined(GCSV2)
  #define STORAGE_SWITCH_A
  #define HARDWARE_SWITCH_A
  #define SWITCHES_GPIO_REG_A_H         GPIOD->IDR
  #define SWITCHES_GPIO_PIN_A_H         GPIO_Pin_10 // PD.10
  #define SWITCHES_GPIO_REG_A_L         GPIOD->IDR
  #define SWITCHES_GPIO_PIN_A_L         GPIO_Pin_14 // PD.14
#endif

#if defined(GCSV2)
  #define STORAGE_SWITCH_B
  #define HARDWARE_SWITCH_B
  #define SWITCHES_GPIO_REG_B_H         GPIOD->IDR
  #define SWITCHES_GPIO_PIN_B_H         GPIO_Pin_9 // PD.9
  #define SWITCHES_GPIO_REG_B_L         GPIOD->IDR
  #define SWITCHES_GPIO_PIN_B_L         GPIO_Pin_11 // PD.11
#endif

#if defined(GCSV2)
  #define STORAGE_SWITCH_C
  #define HARDWARE_SWITCH_C
  #define SWITCHES_GPIO_REG_C_H         GPIOE->IDR
  #define SWITCHES_GPIO_PIN_C_H         GPIO_Pin_14 // PE.14
  #define SWITCHES_GPIO_REG_C_L         GPIOE->IDR
  #define SWITCHES_GPIO_PIN_C_L         GPIO_Pin_15 // PE.15
#endif

#if defined(GCSV2)
  #define STORAGE_SWITCH_D
  #define HARDWARE_SWITCH_D
  #define SWITCHES_GPIO_REG_D_H         GPIOE->IDR
  #define SWITCHES_GPIO_PIN_D_H         GPIO_Pin_12  // PE.12
  #define SWITCHES_GPIO_REG_D_L         GPIOE->IDR
  #define SWITCHES_GPIO_PIN_D_L         GPIO_Pin_13  // PE.13
#endif

#if defined(GCSV2)
  #define STORAGE_SWITCH_E
  #define HARDWARE_SWITCH_E
  #define SWITCHES_GPIO_REG_E_H         GPIOE->IDR
  #define SWITCHES_GPIO_PIN_E_H         GPIO_Pin_8  // PE.08
  #define SWITCHES_GPIO_REG_E_L         GPIOE->IDR
  #define SWITCHES_GPIO_PIN_E_L         GPIO_Pin_9  // PE.09
#endif

#if defined(GCSV2)
  #define STORAGE_SWITCH_G
  #define HARDWARE_SWITCH_G
  #define SWITCHES_GPIO_REG_G_H         GPIOB->IDR
  #define SWITCHES_GPIO_PIN_G_H         GPIO_Pin_2  // PB.02
  #define SWITCHES_GPIO_REG_G_L         GPIOE->IDR
  #define SWITCHES_GPIO_PIN_G_L         GPIO_Pin_7  // PE.07
#endif

#if defined(GCSV2)
  #define STORAGE_SWITCH_H
  #define HARDWARE_SWITCH_H
  #define SWITCHES_GPIO_REG_H           GPIOA->IDR
  #define SWITCHES_GPIO_PIN_I           GPIO_PIN_10 //PA.10
#endif

#if defined(GCSV2)
  #define STORAGE_SWITCH_I
  #define HARDWARE_SWITCH_I
  #define SWITCHES_GPIO_REG_I           GPIOC->IDR
  #define SWITCHES_GPIO_PIN_I           GPIO_Pin_13  //PC.13
#endif


#if defined(GCSV2)
  #define KEYS_RCC_AHB1Periph           (RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE)
  #define KEYS_GPIOA_PINS               0
  #define KEYS_GPIOB_PINS               (SWITCHES_GPIO_PIN_G_H | KEYS_GPIO_PIN_ENTER)
  #define KEYS_GPIOC_PINS               (SWITCHES_GPIO_PIN_I)
  #define KEYS_GPIOD_PINS               (ROTARY_ENCODER_GPIO_PIN_B | ROTARY_ENCODER_GPIO_PIN_A | SWITCHES_GPIO_PIN_B_L | SWITCHES_GPIO_PIN_B_H | SWITCHES_GPIO_PIN_A_L | SWITCHES_GPIO_PIN_A_H | KEYS_GPIO_PIN_PAGE | KEYS_GPIO_PIN_EXIT | KEYS_GPIO_PIN_MENU)
  #define KEYS_GPIOE_PINS               (SWITCHES_GPIO_PIN_E_L | SWITCHES_GPIO_PIN_E_H | SWITCHES_GPIO_PIN_D_L | SWITCHES_GPIO_PIN_D_H | SWITCHES_GPIO_PIN_C_L | SWITCHES_GPIO_PIN_C_H | KEYS_GPIO_PIN_PLUS | KEYS_GPIO_PIN_MINUS)
  #define KEYS_GPIOF_PINS               0
  #define KEYS_GPIOG_PINS               0
#endif




//ADC sticks, pots, sliders, battery,
//channel selection 0 for ADC1 on DMA2 using stream4

#define ADC_MAIN                        ADC1
#define ADC_DMA                         DMA2
#define ADC_DMA_SxCR_CHSEL              0 
#define ADC_DMA_Stream                  DMA2_Stream4
										// sets HIFCR reg for stream 4 clear all types of interrupt flags
#define ADC_SET_DMA_FLAGS()             ADC_DMA->HIFCR = (DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4)										
										// if transfer is completed and reg is enabled return true
#define ADC_TRANSFER_COMPLETE()         (ADC_DMA->HISR & DMA_HISR_TCIF4)
#define ADC_SAMPTIME                    2   // sample time = 28 cycles
#define ADC_MAIN_SMPR1               (ADC_SAMPTIME << 0) + (ADC_SAMPTIME << 3) + (ADC_SAMPTIME << 6) + (ADC_SAMPTIME << 9) + (ADC_SAMPTIME << 12) + (ADC_SAMPTIME << 15) + (ADC_SAMPTIME << 18) + (ADC_SAMPTIME << 21) + (ADC_SAMPTIME << 24);
#define ADC_MAIN_SMPR2               (ADC_SAMPTIME << 0) + (ADC_SAMPTIME << 3) + (ADC_SAMPTIME << 6) + (ADC_SAMPTIME << 9) + (ADC_SAMPTIME << 12) + (ADC_SAMPTIME << 15) + (ADC_SAMPTIME << 18) + (ADC_SAMPTIME << 21) + (ADC_SAMPTIME << 24) + (ADC_SAMPTIME << 27);


#if defined(GCSV2)
  #define HARDWARE_POT1
  #define HARDWARE_POT2
  #define HARDWARE_POT3
  #define HARDWARE_POT4

  #define ADC_RCC_AHB1Periph            (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_DMA2)
  #define ADC_RCC_APB1Periph            0
  #define ADC_RCC_APB2Periph            (RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC3)

  #define ADC_GPIO_PIN_STICK_RV         GPIO_Pin_0  // PA.00
  #define ADC_GPIO_PIN_STICK_RH         GPIO_Pin_1  // PA.01
  #define ADC_GPIO_PIN_STICK_LH         GPIO_Pin_2  // PA.02
  #define ADC_GPIO_PIN_STICK_LV         GPIO_Pin_3  // PA.03

  #define ADC_CHANNEL_STICK_RV          ADC_Channel_0  // ADC1_IN0
  #define ADC_CHANNEL_STICK_RH          ADC_Channel_1  // ADC1_IN1
  #define ADC_CHANNEL_STICK_LH          ADC_Channel_2  // ADC1_IN2
  #define ADC_CHANNEL_STICK_LV          ADC_Channel_3  // ADC1_IN3

  #define ADC_GPIO_PIN_POT1             GPIO_Pin_3  // PC.03
  #define ADC_GPIO_PIN_POT2             GPIO_Pin_0  // PB.00
  #define ADC_GPIO_PIN_POT3             GPIO_Pin_5  // PC.05
  #define ADC_GPIO_PIN_POT4             GPIO_Pin_4  // PC.04

  #define ADC_GPIO_PIN_SLIDER1          GPIO_Pin_2 //  PC.02
  #define ADC_GPIO_PIN_SLIDER2          GPIO_Pin_5  // PA.05
  #define ADC_GPIO_PIN_SLIDER3          GPIO_Pin_6  // PA.06
  #define ADC_GPIO_PIN_SLIDER4          GPIO_Pin_1  // PB.01

  #define ADC_GPIO_PIN_BATT             GPIO_Pin_0  // PC.00

  #define ADC_GPIOA_PINS                (ADC_GPIO_PIN_STICK_RV | ADC_GPIO_PIN_STICK_RH | ADC_GPIO_PIN_STICK_LH | ADC_GPIO_PIN_STICK_LV | ADC_GPIO_PIN_SLIDER3 | ADC_GPIO_PIN_SLIDER2)
  #define ADC_GPIOB_PINS                (ADC_GPIO_PIN_POT2 | ADC_GPIO_PIN_SLIDER4)
  #define ADC_GPIOC_PINS                (ADC_GPIO_PIN_POT1 | ADC_GPIO_PIN_POT3 | ADC_GPIO_PIN_POT4 | ADC_GPIO_PIN_SLIDER1 | ADC_GPIO_PIN_BATT)

  #define ADC_CHANNEL_POT1              ADC_Channel_13  // ADC3_IN13
  #define ADC_CHANNEL_POT2              ADC_Channel_8  // ADC1_IN8
  #define ADC_CHANNEL_POT3              ADC_Channel_15 // ADC1_IN15
  #define ADC_CHANNEL_POT4              ADC_Channel_14 // ADC1_IN14

  #define ADC_CHANNEL_SLIDER1           ADC_Channel_12 // ADC3_IN12
  #define ADC_CHANNEL_SLIDER2           ADC_Channel_5  // ADC1_IN5
  #define ADC_CHANNEL_SLIDER3           ADC_Channel_6  // ADC1_IN6
  #define ADC_CHANNEL_SLIDER4           ADC_Channel_9  // ADC1_IN9

  #define ADC_CHANNEL_BATT              ADC_Channel_10 // ADC1_IN10

  #define ADC_EXT                       ADC3
  #define ADC_EXT_DMA                   DMA2
  #define ADC_EXT_DMA_Stream            DMA2_Stream0
										// sets LIFCR reg for stream 0, stream 0 clear all types of interrupt flags
  #define ADC_EXT_SET_DMA_FLAGS()       ADC_DMA->LIFCR = (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0)
  #define ADC_EXT_TRANSFER_COMPLETE()   (ADC_DMA->LISR & DMA_LISR_TCIF0)
  #define ADC_EXT_SAMPTIME              3    // sample time = 56 cycles
  #define ADC_VREF_PREC2                300
#endif

#define PWR_RCC_AHB1Periph              (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE)




// PWR and LEDS
#if defined(GCSV2)
  #define PWR_SWITCH_GPIO               GPIOD
  #define PWR_SWITCH_GPIO_PIN           GPIO_Pin_1  // PD.01
  #define PWR_ON_GPIO                   GPIOD
  #define PWR_ON_GPIO_PIN               GPIO_Pin_0  // PD.00
#endif

#if defined(GCSV2)
  #define STATUS_LEDS
  #define GPIO_LED_GPIO_ON              GPIO_SetBits
  #define GPIO_LED_GPIO_OFF             GPIO_ResetBits
  #define LED_RED_GPIO                  GPIOA
  #define LED_RED_GPIO_PIN              GPIO_Pin_7  // PA.07
  #define LED_GREEN_GPIO                GPIOE
  #define LED_GREEN_GPIO_PIN            GPIO_Pin_3  // PE.03
  #define LED_BLUE_GPIO                 GPIOE
  #define LED_BLUE_GPIO_PIN             GPIO_Pin_4  // PE.04
#endif





// External Module
#if defined(GCSV2)
  #define EXTMODULE_RCC_APB2Periph    (RCC_APB2Periph_TIM8 | RCC_APB2Periph_USART6)
  #define EXTMODULE_RCC_AHB1Periph    (RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA2)
  #define EXTMODULE_PWR_GPIO          GPIOD
  #define EXTMODULE_PWR_GPIO_PIN      GPIO_Pin_8  // PD.08

  #define EXTERNAL_MODULE_PWR_ON()      GPIO_SetBits(EXTMODULE_PWR_GPIO, EXTMODULE_PWR_GPIO_PIN)
  #define EXTERNAL_MODULE_PWR_OFF()     GPIO_ResetBits(EXTMODULE_PWR_GPIO, EXTMODULE_PWR_GPIO_PIN)
  #define IS_EXTERNAL_MODULE_ON()       (GPIO_ReadInputDataBit(EXTMODULE_PWR_GPIO, EXTMODULE_PWR_GPIO_PIN) == Bit_SET)
  #define EXTMODULE_TX_GPIO             GPIOC
  #define EXTMODULE_USART_GPIO          EXTMODULE_TX_GPIO
  #define EXTMODULE_TX_GPIO_PIN         GPIO_Pin_6  // PC.06
  #define EXTMODULE_TX_GPIO_PinSource   GPIO_PinSource6
  #define EXTMODULE_RX_GPIO_PIN         GPIO_Pin_7  // PC.07
  #define EXTMODULE_RX_GPIO_PinSource   GPIO_PinSource7
  #define EXTMODULE_TIMER               TIM8
  #define EXTMODULE_TIMER_FREQ          (PERI2_FREQUENCY * TIMER_MULT_APB2)
  #define EXTMODULE_TIMER_CC_IRQn       TIM8_CC_IRQn
  #define EXTMODULE_TIMER_CC_IRQHandler TIM8_CC_IRQHandler
  #define EXTMODULE_TIMER_TX_GPIO_AF    GPIO_AF_TIM8 // TIM8_CH1
  #define EXTMODULE_TIMER_DMA_CHANNEL           DMA_Channel_7
  #define EXTMODULE_TIMER_DMA_STREAM            DMA2_Stream1
  #define EXTMODULE_TIMER_DMA_STREAM_IRQn       DMA2_Stream1_IRQn
  #define EXTMODULE_TIMER_DMA_STREAM_IRQHandler DMA2_Stream1_IRQHandler
  #define EXTMODULE_TIMER_DMA_FLAG_TC           DMA_IT_TCIF1
  #define EXTMODULE_TIMER_OUTPUT_ENABLE         TIM_CCER_CC1E
  #define EXTMODULE_TIMER_OUTPUT_POLARITY       TIM_CCER_CC1P
  #define EXTMODULE_USART_GPIO_AF               GPIO_AF_USART6
  #define EXTMODULE_USART                       USART6
  #define EXTMODULE_USART_IRQn                  USART6_IRQn
  #define EXTMODULE_USART_IRQHandler            USART6_IRQHandler
  #define EXTMODULE_USART_TX_DMA_CHANNEL        DMA_Channel_5
  #define EXTMODULE_USART_TX_DMA_STREAM         DMA2_Stream6
  #define EXTMODULE_USART_RX_DMA_CHANNEL        DMA_Channel_5
  #define EXTMODULE_USART_RX_DMA_STREAM         DMA2_Stream1
#endif



//SWD debug
#if defined(GCSV2)
#define DEBUG_GPIO                     GPIOA
#define DEBUG_SWDIO_GPIO_PIN_SOURCE    GPIO_PinSource13
#define DEBUG_SWDCLK_GPIO_PIN_SOURCE   GPIO_PinSource14
#define DEBUG_SWDIO_GPIO_PIN           GPIO_PIN_13 //PA.13
#define DEBUG_SWDCLK_GPIO_PIN          GPIO_PIN_14 //PA.14
//add support for pi debugging here, configure UART, simple serial interface for debug prints? 

#endif


//TODO: do I need to configure external clocks?, XTAL1, XTAL2, what clocks to use for perifs? 
#if defined(GCSV2)

#endif

//Baro placeholder 
#if defined(GCSV2)
  #define BARO_GPIO           GPIOE
  #define BARO_GPIO_CS_PIN    GPIO_PIN_1       
#endif


//PI UART placeholder
#if defined(GCSV2)
   #define PI_UART_GPIO      GPOIOD
   #define PI_UART_GPIO_TX_PIN GPIO_PIN_5
   #define PI_UART_GPIO_RX_PIN GPIO_PIN_6   
#endif


// Trainer Port
#if defined(GCSV2)
  #define TRAINER_RCC_AHB1Periph        (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA1)
  #define TRAINER_RCC_APB1Periph        RCC_APB1Periph_TIM3
  #define TRAINER_GPIO                  GPIOC
  #define TRAINER_IN_GPIO_PIN           GPIO_Pin_8  // PC.08
  #define TRAINER_IN_GPIO_PinSource     GPIO_PinSource8
  #define TRAINER_OUT_GPIO_PIN          GPIO_Pin_9  // PC.09
  #define TRAINER_OUT_GPIO_PinSource    GPIO_PinSource9
  #define TRAINER_DETECT_GPIO           GPIOA
  #define TRAINER_DETECT_GPIO_PIN       GPIO_Pin_8  // PA.08
  #define TRAINER_TIMER                 TIM3
  #define TRAINER_TIMER_IRQn            TIM3_IRQn
  #define TRAINER_GPIO_AF               GPIO_AF_TIM3
  #define TRAINER_DMA                   DMA1
  #define TRAINER_DMA_CHANNEL           DMA_Channel_5
  #define TRAINER_DMA_STREAM            DMA1_Stream2
  #define TRAINER_DMA_IRQn              DMA1_Stream2_IRQn
  #define TRAINER_DMA_IRQHandler        DMA1_Stream2_IRQHandler
  #define TRAINER_DMA_FLAG_TC           DMA_IT_TCIF2
  #define TRAINER_TIMER_IRQn            TIM3_IRQn
  #define TRAINER_TIMER_IRQHandler      TIM3_IRQHandler
  #define TRAINER_TIMER_FREQ            (PERI1_FREQUENCY * TIMER_MULT_APB1)
  #define TRAINER_OUT_CCMR2             TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;
  #define TRAINER_IN_CCMR2              TIM_CCMR2_IC3F_0 | TIM_CCMR2_IC3F_1 | TIM_CCMR2_CC3S_0;
  #define TRAINER_OUT_COUNTER_REGISTER  TRAINER_TIMER->CCR4
  #define TRAINER_IN_COUNTER_REGISTER   TRAINER_TIMER->CCR3
  #define TRAINER_SETUP_REGISTER        TRAINER_TIMER->CCR1
  #define TRAINER_OUT_INTERRUPT_FLAG    TIM_SR_CC1IF
  #define TRAINER_OUT_INTERRUPT_ENABLE  TIM_DIER_CC1IE
  #define TRAINER_IN_INTERRUPT_ENABLE   TIM_DIER_CC3IE
  #define TRAINER_IN_INTERRUPT_FLAG     TIM_SR_CC3IF
  #define TRAINER_OUT_CCER              TIM_CCER_CC4E
  #define TRAINER_IN_CCER               TIM_CCER_CC3E
  #define TRAINER_CCER_POLARYTY         TIM_CCER_CC4P
#endif

#if defined(GCSV2)
  #define TRAINER_MODULE_CPPM
  #define TRAINER_MODULE_SBUS
  #define TRAINER_MODULE_RCC_AHB1Periph      0
  #define TRAINER_MODULE_RCC_APB1Periph      RCC_APB1Periph_TIM3
  #define TRAINER_MODULE_RCC_APB2Periph      0
  #define TRAINER_MODULE_CPPM_GPIO           EXTMODULE_USART_GPIO
  #define TRAINER_MODULE_CPPM_GPIO_PIN       EXTMODULE_RX_GPIO_PIN
  #define TRAINER_MODULE_CPPM_GPIO_PinSource EXTMODULE_RX_GPIO_PinSource
  #define TRAINER_MODULE_CPPM_GPIO_AF        GPIO_AF_TIM3
  #define TRAINER_MODULE_CPPM_TIMER          TIM3
  #define TRAINER_MODULE_CPPM_INTERRUPT_ENABLE TIM_DIER_CC2IE
  #define TRAINER_MODULE_CPPM_INTERRUPT_FLAG   TIM_SR_CC2IF
  #define TRAINER_MODULE_CPPM_CCMR1            (TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1 | TIM_CCMR1_CC2S_0)
  #define TRAINER_MODULE_CPPM_CCER             TIM_CCER_CC2E
  #define TRAINER_MODULE_CPPM_TIMER_IRQn       TIM3_IRQn
  #define TRAINER_MODULE_CPPM_TIMER_IRQHandler TIM3_IRQHandler
  #define TRAINER_MODULE_SBUS_GPIO             EXTMODULE_USART_GPIO
  #define TRAINER_MODULE_SBUS_GPIO_PIN         EXTMODULE_RX_GPIO_PIN
  #define TRAINER_MODULE_SBUS_GPIO_AF          EXTMODULE_USART_GPIO_AF
  #define TRAINER_MODULE_SBUS_USART            EXTMODULE_USART
  #define TRAINER_MODULE_SBUS_GPIO_PinSource   EXTMODULE_RX_GPIO_PinSource
  #define TRAINER_MODULE_SBUS_DMA_STREAM       EXTMODULE_USART_RX_DMA_STREAM
  #define TRAINER_MODULE_SBUS_DMA_CHANNEL      EXTMODULE_USART_RX_DMA_CHANNEL
#endif


// No aux
#define AUX2_SERIAL_RCC_AHB1Periph        0
#define AUX2_SERIAL_RCC_APB1Periph        0
#define AUX2_SERIAL_RCC_APB2Periph        0
#define AUX_SERIAL_RCC_AHB1Periph         0
#define AUX_SERIAL_RCC_APB1Periph         0
#define AUX_SERIAL_RCC_APB2Periph         0

// Telemetry dir
#define TELEMETRY_RCC_AHB1Periph        (RCC_AHB1Periph_GPIOD)
#define TELEMETRY_DIR_GPIO              GPIOD
#define TELEMETRY_DIR_GPIO_PIN          GPIO_Pin_4  // PD.04
#define TELEMETRY_DIR_OUTPUT()          TELEMETRY_DIR_GPIO->BSRRL = TELEMETRY_DIR_GPIO_PIN
#define TELEMETRY_DIR_INPUT()           TELEMETRY_DIR_GPIO->BSRRH = TELEMETRY_DIR_GPIO_PIN


// USB Charger
#define USB_CHARGER_RCC_AHB1Periph      RCC_AHB1Periph_GPIOB
#define USB_CHARGER_GPIO                GPIOB
#define USB_CHARGER_GPIO_PIN            GPIO_Pin_5  // PB.05


// USB
#define USB_RCC_AHB1Periph_GPIO         RCC_AHB1Periph_GPIOA
#define USB_GPIO                        GPIOA
#define USB_GPIO_PIN_VBUS               GPIO_Pin_9  // PA.09
#define USB_GPIO_PIN_DM                 GPIO_Pin_11 // PA.11
#define USB_GPIO_PIN_DP                 GPIO_Pin_12 // PA.12
#define USB_GPIO_PinSource_DM           GPIO_PinSource11
#define USB_GPIO_PinSource_DP           GPIO_PinSource12
#define USB_GPIO_AF                     GPIO_AF_OTG1_FS



// BackLight
#if defined(GCSV2)
  #define BACKLIGHT_RCC_AHB1Periph      RCC_AHB1Periph_GPIOE
  #define BACKLIGHT_RCC_APB1Periph      0
  #define BACKLIGHT_RCC_APB2Periph      RCC_APB2Periph_TIM9
  #define BACKLIGHT_TIMER_FREQ          (PERI2_FREQUENCY * TIMER_MULT_APB2)
  #define BACKLIGHT_TIMER               TIM9
  #define BACKLIGHT_GPIO                GPIOE
  #define BACKLIGHT_GPIO_PIN_1          GPIO_Pin_5 // PE.05
  #define BACKLIGHT_GPIO_PIN_2          GPIO_Pin_6 // PE.06
  #define BACKLIGHT_GPIO_PinSource_1    GPIO_PinSource5
  #define BACKLIGHT_GPIO_PinSource_2    GPIO_PinSource6
  #define BACKLIGHT_GPIO_AF_1           GPIO_AF_TIM9
  #define BACKLIGHT_GPIO_AF_2           GPIO_AF_TIM9
#endif

#define KEYS_BACKLIGHT_RCC_AHB1Periph        0



#if defined(GCSV2)
  #define LCD_RCC_AHB1Periph            (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1)
  #define LCD_RCC_APB1Periph            RCC_APB1Periph_SPI3
  #define LCD_SPI_GPIO                  GPIOC
  #define LCD_MOSI_GPIO_PIN             GPIO_Pin_12 // PC.12
  #define LCD_MOSI_GPIO_PinSource       GPIO_PinSource12
  #define LCD_CLK_GPIO_PIN              GPIO_Pin_10 // PC.10
  #define LCD_CLK_GPIO_PinSource        GPIO_PinSource10
  #define LCD_A0_GPIO_PIN               GPIO_Pin_11 // PC.11

  #define LCD_NCS_GPIO                  GPIOB
  #define LCD_NCS_GPIO_PIN              GPIO_Pin_15 // PA.15
  #define LCD_RST_GPIO                  GPIOD
  #define LCD_RST_GPIO_PIN              GPIO_Pin_15 // PD.15
  #define LCD_DMA                       DMA1
  #define LCD_DMA_Stream                DMA1_Stream7
  #define LCD_DMA_Stream_IRQn           DMA1_Stream7_IRQn
  #define LCD_DMA_Stream_IRQHandler     DMA1_Stream7_IRQHandler
  #define LCD_DMA_FLAGS                 (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7)
  #define LCD_DMA_FLAG_INT              DMA_HIFCR_CTCIF7
  #define LCD_SPI                       SPI3
  #define LCD_GPIO_AF                   GPIO_AF_SPI3
#endif
#define LCD_RCC_APB2Periph              0


// I2C Bus: EEPROM and CAT5137 digital pot for volume control
#define I2C_RCC_APB1Periph              RCC_APB1Periph_I2C1
#define I2C                             I2C1
#define I2C_GPIO_AF                     GPIO_AF_I2C1
#if defined(GCSV2)
  #define I2C_RCC_AHB1Periph            (RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD)
  #define I2C_SPI_GPIO                  GPIOB
  #define I2C_SDA_GPIO_PIN              GPIO_Pin_9  // PB.09
  #define I2C_SCL_GPIO_PIN              GPIO_Pin_8  // PB.08
  #define I2C_WP_GPIO                   GPIOD
  #define I2C_WP_GPIO_PIN               GPIO_Pin_7  // PD.07
  #define I2C_SCL_GPIO_PinSource        GPIO_PinSource8
  #define I2C_SDA_GPIO_PinSource        GPIO_PinSource9
#endif
#define I2C_SPEED                       400000
#define I2C_ADDRESS_EEPROM              0xA2
#define I2C_FLASH_PAGESIZE              64


// SPI2-SD
#define SD_RCC_AHB1Periph               (RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1)
#define SD_RCC_APB1Periph               RCC_APB1Periph_SPI2
#define SD_GPIO                         GPIOB
#define SD_GPIO_PIN_CS                  GPIO_Pin_12 // PB.12
#define SD_GPIO_PIN_SCK                 GPIO_Pin_13 // PB.13
#define SD_GPIO_PIN_MISO                GPIO_Pin_14 // PB.14
#define SD_GPIO_PIN_MOSI                GPIO_Pin_15 // PB.15
#define SD_PRESENT_GPIO_PIN             GPIOB
#define SD_PRESENT_GPIO                 GPIO_Pin_6  // PB.06
#define SD_GPIO_AF                      GPIO_AF_SPI2
#define SD_GPIO_PinSource_CS            GPIO_PinSource12
#define SD_GPIO_PinSource_SCK           GPIO_PinSource13
#define SD_GPIO_PinSource_MISO          GPIO_PinSource14
#define SD_GPIO_PinSource_MOSI          GPIO_PinSource15
#define SD_SPI                          SPI2
#define SD_SPI_BaudRatePrescaler        SPI_BaudRatePrescaler_4 // 10.5<20MHZ, make sure < 20MHZ

//BOOT is never defined
#if !defined(BOOT)
  #define SD_USE_DMA                    // Enable the DMA for SD
  #define SD_DMA_Stream_SPI_RX          DMA1_Stream3
  #define SD_DMA_Stream_SPI_TX          DMA1_Stream4
  #define SD_DMA_FLAG_SPI_TC_RX         DMA_FLAG_TCIF3
  #define SD_DMA_FLAG_SPI_TC_TX         DMA_FLAG_TCIF4
  #define SD_DMA_Channel_SPI            DMA_Channel_0
#endif

// Audio
#define AUDIO_RCC_APB1Periph            (RCC_APB1Periph_TIM6 | RCC_APB1Periph_DAC)
#define AUDIO_OUTPUT_GPIO               GPIOA
#define AUDIO_OUTPUT_GPIO_PIN           GPIO_Pin_4  // PA.04
#define AUDIO_DMA_Stream                DMA1_Stream5
#define AUDIO_DMA_Stream_IRQn           DMA1_Stream5_IRQn
#define AUDIO_TIM_IRQn                  TIM6_DAC_IRQn
#define AUDIO_TIM_IRQHandler            TIM6_DAC_IRQHandler
#define AUDIO_DMA_Stream_IRQHandler     DMA1_Stream5_IRQHandler
#define AUDIO_TIMER                     TIM6
#define AUDIO_DMA                       DMA1
#define AUDIO_RCC_AHB1Periph           (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1)

// Xms Interrupt
#define INTERRUPT_xMS_RCC_APB1Periph    RCC_APB1Periph_TIM14
#define INTERRUPT_xMS_TIMER             TIM14
#define INTERRUPT_xMS_IRQn              TIM8_TRG_COM_TIM14_IRQn
#define INTERRUPT_xMS_IRQHandler        TIM8_TRG_COM_TIM14_IRQHandler

// 2MHz Timer
#define TIMER_2MHz_RCC_APB1Periph       RCC_APB1Periph_TIM7
#define TIMER_2MHz_TIMER                TIM7

// Mixer scheduler timer
#define MIXER_SCHEDULER_TIMER_RCC_APB1Periph RCC_APB1Periph_TIM13
#define MIXER_SCHEDULER_TIMER                TIM13
#define MIXER_SCHEDULER_TIMER_FREQ           (PERI1_FREQUENCY * TIMER_MULT_APB1)
#define MIXER_SCHEDULER_TIMER_IRQn           TIM8_UP_TIM13_IRQn
#define MIXER_SCHEDULER_TIMER_IRQHandler     TIM8_UP_TIM13_IRQHandler

#endif // _HAL_H_
