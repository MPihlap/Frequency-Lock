
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "pdm2pcm.h"

/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <math.h>
#include "note_freq.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2S_HandleTypeDef hi2s2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2S2_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM6_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
// static void SPI2_NVIC_INIT(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint16_t PDM_BUF_1[PDM_BUF_SIZE];  // PDM buffer1
uint16_t PCM_BUF_1[PCM_BUF_SIZE];  // PCM buffer1
uint16_t PCM_BUF_2[PCM_BUF_SIZE];  // PCM buffer2
float32_t fft_output_buffer[PCM_BUF_SIZE * 2];
float32_t fft_input_buffer[PCM_BUF_SIZE];
float32_t fft_mag_buffer[PCM_BUF_SIZE];
uint32_t local_pcm_pointer = 0;  // Keeps track of PCM buffer
uint8_t PCM_switch_flag = 0;
uint16_t *current_PCM_buffer;  // Pointer to array to be recorded to
uint8_t RECORD_ENABLE = 0;     // Recording control flag
uint8_t open_lock_counter = 0;
uint32_t sequence[] = MY_HEART_WILL_GO_ON;
// Desired index of maximum magnitude, initially 100 = 781.25 ... 789 Hz
uint32_t desiredIndex = 100;
uint16_t error = 3;  // How many neighbouring indexes will be considered legit
arm_rfft_fast_instance_f32 S;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void) {
  /* USER CODE BEGIN 1 */
  //__HAL_DBGMCU_UNFREEZE_IWDG();
  /* USER CODE END 1 */

  /* MCU
   * Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_I2S2_Init();
  MX_CRC_Init();
  MX_PDM2PCM_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  arm_rfft_fast_init_f32(&S, PCM_BUF_SIZE);
  desiredIndex = GET_DESIRED_INDEX(sequence[open_lock_counter]);
  error = ERROR_INDEX(sequence[open_lock_counter]);
  LOCK_ENABLE();
  // SPI2_NVIC_INIT();
  HAL_GPIO_WritePin(GPIOE, SPI1_NCS_PIN, GPIO_PIN_SET);
  RECORD_ENABLE = 1;  // Enable I2S reading
  HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_SET);
  uint8_t PCM_switch_prev = PCM_switch_flag;

  current_PCM_buffer = PCM_BUF_1;
  volatile float32_t maxmag;
  volatile uint32_t index;

  HAL_I2S_Receive_IT(&hi2s2, PDM_BUF_1, 64);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t handshake[4] = {1000, 1100, 1200, 1300};

  while (1) {
    // Check if we have a full buffer of PCM data
    if (PCM_switch_flag != PCM_switch_prev) {
      PCM_switch_prev = PCM_switch_flag;  // Set flag to new value
      // Convert data to float for signal processing
      for (uint16_t i = 0; i < PCM_BUF_SIZE; i++) {
        fft_input_buffer[i] = (float32_t)PCM_BUF_1[i];
      }

      // Perform real fft
      arm_rfft_fast_f32(&S, fft_input_buffer, fft_output_buffer, 0);
      // Find magnitudes of complex values
      arm_cmplx_mag_f32(fft_output_buffer, fft_mag_buffer, PCM_BUF_SIZE);
      // Find maximum magnitude, ignoring first element
      arm_max_f32(&(fft_mag_buffer[1]), PCM_BUF_SIZE - 1, &maxmag, &index);

      // Only react to high enough magnitudes
      if (maxmag > 50) {
        // Check if the highest magnitude corresponds to the desired frequency
        if ((index > desiredIndex - error) && (index < desiredIndex + error)) {
          if (open_lock_counter == 0) {
            HAL_TIM_Base_Start_IT(&htim6);
            HAL_GPIO_WritePin(GPIOD, LED4_PIN, GPIO_PIN_SET);
          }
          HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_SET);
          // Move to next stage
          open_lock_counter++;
          desiredIndex = GET_DESIRED_INDEX(sequence[open_lock_counter]);
          error = ERROR_INDEX(sequence[open_lock_counter]);

          // Success! open lock
          if (open_lock_counter >= LOCK_STAGES) {
            LOCK_DISABLE();
            HAL_GPIO_WritePin(LED_PORT, LED1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_PORT, LED4_PIN, GPIO_PIN_SET);
            // Start timer to keep lock open for 5 seconds
            htim6.Instance->CNT = 0;
            HAL_TIM_Base_Start_IT(&htim6);
          }
        }
      }

      // Resume recording after signal processing
      RECORD_ENABLE = 1;
      __HAL_I2S_CLEAR_OVRFLAG(&hi2s2);
      HAL_I2S_Receive_IT(&hi2s2, PDM_BUF_1, 64);
    }
    HAL_Delay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  /**Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure the Systick interrupt time
  */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CRC init function */
static void MX_CRC_Init(void) {
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  __HAL_CRC_DR_RESET(&hcrc);
}

/* I2S2 init function */
static void MX_I2S2_Init(void) {
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_LSB;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* SPI1 init function */
static void MX_SPI1_Init(void) {
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM6 init function */
static void MX_TIM6_Init(void) {
  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 2600 * TIME_TO_UNLOCK;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 32000;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_SR_UIF);
}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
     PA4   ------> I2S3_WS
*/
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2 | CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin,
                    GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  HAL_GPIO_TogglePin(GPIOD, LED4_PIN);
  HAL_GPIO_WritePin(LED_PORT, LED1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PORT, LED4_PIN, GPIO_PIN_RESET);
  HAL_TIM_Base_Stop_IT(&htim6);
  LOCK_ENABLE();
  open_lock_counter = 0;
  desiredIndex = GET_DESIRED_INDEX(sequence[0]);
  error = ERROR_INDEX(sequence[open_lock_counter]);
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
  PDM_Filter(PDM_BUF_1, &PCM_BUF_1[local_pcm_pointer], &PDM1_filter_handler);

  HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_RESET);
  local_pcm_pointer = local_pcm_pointer + 16;  // The filter returns 16 samples
  if (local_pcm_pointer == PCM_BUF_SIZE) {
    local_pcm_pointer = 0;
    RECORD_ENABLE = 0;
    HAL_GPIO_WritePin(GPIOD, LED2_PIN, GPIO_PIN_RESET);
    PCM_switch_flag ^= 1;
  }
  if (RECORD_ENABLE == 1) {
    HAL_StatusTypeDef result;
    result = HAL_I2S_Receive_IT(&hi2s2, PDM_BUF_1, DECIMATION_FACTOR);

    if (result == HAL_OK) {
      HAL_GPIO_WritePin(GPIOD, LED2_PIN, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(GPIOD, LED2_PIN, GPIO_PIN_RESET);
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
   */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
