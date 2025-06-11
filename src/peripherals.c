/**
 * BLDC Motor Driver Peripheral Configurations 
 * 
 * Peripherals:      GPIOA, GPIOB, GPIOC, ADC1, TIM2, DMA1, CAN2
 * Requests:         NRST, BOOT0, PH0(OSC_IN), PH1(OSC_OUT), PC0(ADC1_IN10),
 *                   PC1(ADC1_IN11), PC2(ADC1_IN12), PA1(ADC1_IN1), 
 *                   PA2(ADC1_IN2), PA3(ADC1_IN3), PC4(PGOOD), PC6(HALL_A),
 *                   PC7(HALL_B), PC8(HALL_C), PB0(LED_RED), PC10(SD_A),
 *                   PC11(SD_B), PC12(SD_C), PB5(CAN2_RX), PB6(CAN2_TX),
 *                   PA8(TIM1_CH1), PA9(TIM1_CH2), PA10(TIM1_CH3), 
 *                   PA11(OTG_DM), PA12(OTG_DP), PA13(SWDIO), PA14(SWCLK)
 */

#include "bldc_motor_driver.h"

/**
 * System Oscillator Configuration
 * 
 * Oscillator:    16MHz Crystal
 * PLL Input:     1MHz
 * VCO:           240MHz
 * System Clock:  120MHz
 */
void bldc_perip_oscillator(void)
{
   iosc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
   iosc.HSEState = RCC_HSE_ON;
   iosc.PLL.PLLState = RCC_PLL_ON;
   iosc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
   iosc.PLL.PLLM = 16;              /* 1MHz PLL input */
   iosc.PLL.PLLN = 240;             /* 240MHz VCO */
   iosc.PLL.PLLP = RCC_PLLP_DIV2;   /* 120MHz system clock */
   iosc.PLL.PLLQ = 5;               /* 48MHz for USB */
   HAL_RCC_OscConfig(&iosc);
}

/**
 * System Clock Configuration
 * 
 * AHB Clock:     60MHz
 * APB1 Clock:    60MHz
 * APB2 Clock:    60MHz
 */
void bldc_perip_clock(void)
{
   iclk.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
   iclk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
   iclk.AHBCLKDivider = RCC_SYSCLK_DIV2;     
   iclk.APB1CLKDivider = RCC_HCLK_DIV1;      
   iclk.APB2CLKDivider = RCC_HCLK_DIV1;      
   HAL_RCC_ClockConfig(&iclk, FLASH_LATENCY_3);
}

/**
 * General Purpose Timer Configuration
 * 
 * Peripheral:       TIM2, TIM3, TIM8
 * 
 * Set the TIM2 timer with 100us for system interrupts and TIM3 as
 * free-running for speed estimation. TIM8 is used for Sparse Vector
 * PWM.
 */
void bldc_perip_timer(void)
{
   htim2.Instance = TIM2;
   htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   htim2.Init.Prescaler = 60 - 1;   
   htim2.Init.Period = 100 - 1;
   htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
   htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
   HAL_TIM_Base_Init(&htim2);
   
   sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
   HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

   htim3.Instance = TIM3;
   htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   htim3.Init.Prescaler = 6000 - 1;            
   htim3.Init.Period = 0xFFFFFFFF;                 /* ~ 71 minute */
   htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
   htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
   HAL_TIM_Base_Init(&htim3);

   htim1.Instance = TIM1;
   htim1.Init.Prescaler = 0;
   htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
   htim1.Init.Period = 3000 - 1;
   htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   htim1.Init.RepetitionCounter = 0;
   htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
   HAL_TIM_PWM_Init(&htim1);

   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = 0; 
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;

   HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
   HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
   HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
}

/**
 * Analog-to-Digital Converter Configuration
 * 
 * Peripheral:       ADC1
 * Requests:         PA1(ADC1_IN1), PA2(ADC1_IN2), PA3(ADC1_IN3),
 *                   PA4(ADC1_IN4), PC0(ADC1_IN10)
 * 
 * Set the ADC peripheral to read phase currents and
 * temperature sensors.
 */
void bldc_perip_analog(void)
{
   igpio.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
   igpio.Mode = GPIO_MODE_ANALOG; 
   igpio.Pull = GPIO_NOPULL;
   igpio.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOA, &igpio);

   igpio.Pin = GPIO_PIN_10;
   igpio.Mode = GPIO_MODE_ANALOG;
   igpio.Pull = GPIO_NOPULL;
   igpio.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOC, &igpio);

   hdma_adc1.Instance = DMA1_Stream1;
   hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
   hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
   hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
   hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
   hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
   hdma_adc1.Init.Mode = DMA_CIRCULAR;
   hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
   HAL_DMA_Init(&hdma_adc1);

   __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

   HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
   HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

   hadc1.Instance = ADC1;
   hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
   hadc1.Init.Resolution = ADC_RESOLUTION_12B;
   hadc1.Init.ScanConvMode = ENABLE;
   hadc1.Init.ContinuousConvMode = DISABLE;
   hadc1.Init.DiscontinuousConvMode = DISABLE;
   hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO; 
   hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
   hadc1.Init.NbrOfConversion = 5;
   hadc1.Init.DMAContinuousRequests = ENABLE;
   hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
   HAL_ADC_Init(&hadc1);

   uint32_t channels[5] = {
      ADC_CHANNEL_1,          // PA1
      ADC_CHANNEL_2,          // PA2
      ADC_CHANNEL_3,          // PA3
      ADC_CHANNEL_4,          // PA4
      ADC_CHANNEL_10          // PC0 
  };
  for (int i = 0; i < 5; i++) {
      sConfig.Channel = channels[i];
      sConfig.Rank = i + 1;
      sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
      HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  }
}

/**
 * Digital Input/Output Configuration
 * 
 * Peripherals:      GPIOA, GPIOB, GPIOC
 * Requests:         PB0(LED_RED), PC10(SD_A), PC11(SD_B), 
 *                   PC12(SD_C), PC6(HALL_A), PC7(HALL_B), 
 *                   PC8(HALL_C), PA8(TIM1_CH1), PA9(TIM1_CH2),
 *                   PA10(TIM1_CH3)
 * 
 * Set the digital input/output modes.
 */
void bldc_perip_digital(void)
{
   /* PC10(SD_A), PC11(SD_B), PC12(SD_C) */
   igpio.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
   igpio.Mode = GPIO_MODE_OUTPUT_PP;
   igpio.Pull = GPIO_NOPULL;
   igpio.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOC, &igpio);
   
   /* PB0(LED_RED) */
   igpio.Pin = GPIO_PIN_0;
   igpio.Mode = GPIO_MODE_OUTPUT_PP;
   igpio.Pull = GPIO_NOPULL;
   igpio.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOB, &igpio);

   /* PC6(HALL_A), PC7(HALL_B), PC8(HALL_C) */
   igpio.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
   igpio.Mode = GPIO_MODE_IT_RISING_FALLING; 
   igpio.Pull = GPIO_NOPULL;
   igpio.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOC, &igpio);

   HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

   /* PA8(TIM1_CH1), PA9(TIM1_CH2), PA10(TIM1_CH3) */
   igpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
   igpio.Mode = GPIO_MODE_AF_PP;
   igpio.Pull = GPIO_NOPULL;
   igpio.Speed = GPIO_SPEED_FREQ_HIGH;
   igpio.Alternate = GPIO_AF1_TIM1;
   HAL_GPIO_Init(GPIOA, &igpio);

   /* PB5(CAN2_RX), PB6(CAN2_TX) */
   igpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
   igpio.Mode = GPIO_MODE_AF_PP;
   igpio.Pull = GPIO_NOPULL;
   igpio.Speed = GPIO_SPEED_FREQ_HIGH;
   igpio.Alternate = GPIO_AF9_CAN2;
   HAL_GPIO_Init(GPIOB, &igpio);

   /* PB10(USART3_TX) */
   igpio.Pin = GPIO_PIN_10;
   igpio.Mode = GPIO_MODE_AF_PP;
   igpio.Pull = GPIO_NOPULL;
   igpio.Speed = GPIO_SPEED_FREQ_HIGH;
   igpio.Alternate = GPIO_AF7_USART3;
   HAL_GPIO_Init(GPIOB, &igpio);

   /* PA6(TIM3_CH1) */
   igpio.Pin = GPIO_PIN_6;
   igpio.Mode = GPIO_MODE_AF_PP;
   igpio.Pull = GPIO_NOPULL;
   igpio.Speed = GPIO_SPEED_FREQ_HIGH;
   igpio.Alternate = GPIO_AF2_TIM3;
   HAL_GPIO_Init(GPIOA, &igpio);
}

/**
 * Serial Line from MCU to Host Machine
 * 
 * Peripherals:      USART3
 * Requests:         PB10(USART3_TX)
 * 
 * Set the USART3 peripheral to communicate with the 
 * host machine over a serial line. The baud rate is 
 * set to 115200. 
 */
void bldc_perip_serial(void)
{
   huart3.Instance = USART3;
   huart3.Init.BaudRate = 115200;
   huart3.Init.WordLength = UART_WORDLENGTH_8B;
   huart3.Init.StopBits = UART_STOPBITS_1;
   huart3.Init.Parity = UART_PARITY_NONE;
   huart3.Init.Mode = UART_MODE_TX;
   huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart3.Init.OverSampling = UART_OVERSAMPLING_16;
   HAL_UART_Init(&huart3);
}

/**
 * Data Transmission over CAN Bus
 * 
 * Peripherals:      CAN2
 * Requests:         PB5(CAN2_RX), PB6(CAN2_TX)
 * 
 * Baud Rate = APB1_CLK / (Prescalar + (1 + BS1 + BS2))
 * Set the CAN bus to receive/transmit data with 500Kbps.
 */
void bldc_perip_comm(void)
{
   hcan2.Instance = CAN2;
   hcan2.Init.Prescaler = 6;         
   hcan2.Init.Mode = CAN_MODE_NORMAL;
   hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ; 
   hcan2.Init.TimeSeg1 = CAN_BS1_11TQ; 
   hcan2.Init.TimeSeg2 = CAN_BS2_8TQ;
   hcan2.Init.TimeTriggeredMode = DISABLE;
   hcan2.Init.AutoBusOff = DISABLE;
   hcan2.Init.AutoWakeUp = DISABLE;
   hcan2.Init.AutoRetransmission = DISABLE;
   hcan2.Init.ReceiveFifoLocked = DISABLE;
   hcan2.Init.TransmitFifoPriority = DISABLE;
   HAL_CAN_Init(&hcan2);

   filterConfig.FilterBank = 14;
   filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
   filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
   filterConfig.FilterIdHigh = 0x0000;
   filterConfig.FilterIdLow = 0x0000;
   filterConfig.FilterMaskIdHigh = 0x0000;
   filterConfig.FilterMaskIdLow = 0x0000;
   filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
   filterConfig.FilterActivation = ENABLE;
   filterConfig.SlaveStartFilterBank = 14;
   HAL_CAN_ConfigFilter(&hcan2, &filterConfig);

   HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 2, 0);
   HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
}
