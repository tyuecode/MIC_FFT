/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "key.h"
#include "OLED.h"
#include "stm32_dsp.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t mode=0;				//模式位

uint8_t data_ready=0;		//数据就位标志
uint8_t data_index=0;		//ADC缓存数组下标
uint8_t data_select=0;		//FFT运算数组下标

int16_t AD_Value_buf[2][256]; // ADC双缓冲区
uint32_t lBufOutArray[256]; // FFT 运算的输出数组
uint32_t dr;				//波形显示变量
uint32_t i=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	OLED_Clear();
	HAL_ADC_Init(&hadc1);
	HAL_ADC_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(mode==0)			//谐波显示模式
	{
		if(i>127){i=0;OLED_Update();OLED_Clear();}
		dr=HAL_ADC_GetValue(&hadc1);			  
		OLED_DrawPoint(i,(float)dr/4095*63);
		i++;	
	}
	else if(mode==1 || data_ready == 1)//频谱显示模式
	{
		data_ready=0;
		data_select=(data_index+1)/2;
		
		cr4_fft_256_stm32(lBufOutArray, AD_Value_buf[data_select], 256);

		int16_t real, imag;
		int magnitude[64];  // 存储幅值结果（只取前64个频率点，适配OLED显示）

		// 计算1~63号频率点的幅值
		for (i = 1; i < 64; i++) 
		{
			real = (lBufOutArray[i] << 16) >> 16;  // 提取实部（低16位）
			imag = (lBufOutArray[i] >> 16);        // 提取虚部（高16位）
			// 计算幅值（简化版，sqrt(real² + imag²)，并做缩放适配OLED显示）
			magnitude[i] = ((int)sqrt(real * real + imag * imag) >> 3) + 2;
		}

		// 单独处理0号频率点（直流分量）
		real = (lBufOutArray[0] << 16) >> 16;
		imag = (lBufOutArray[0] >> 16);
		magnitude[0] = (int)sqrt(real * real + imag * imag) >> 8;  //低频受直流分量影响过高，降低使画面平缓
		
		for(i = 0; i < 64; i++)
		{
			OLED_DrawLine(i*2,63,i*2,(63-magnitude[i]));
			OLED_DrawLine((i*2+1),63,(i*2+1),(63-magnitude[i]));
		}
		OLED_Update();
		OLED_Clear();
	}  
   
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc == &hadc1) 
	{	
		data_ready=1;
		data_index=(data_index+1)/2;
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AD_Value_buf[data_index], 256);
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim4)
	{
		HAL_TIM_Base_Stop_IT(&htim4);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==GPIO_PIN_SET)
		{
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
			if(mode==0)
			{ 
				mode=1;											//频谱模式
				ADC_ChannelConfTypeDef sConfig = {0};
				sConfig.Rank = ADC_REGULAR_RANK_1;
				sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;//固定最大采样时间，确保采样精度

				hadc1.Init.ContinuousConvMode = DISABLE;			//关闭连续转换
				hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;//定时器事件触发
				HAL_ADC_Init(&hadc1);

				HAL_TIM_Base_Start_IT(&htim3);
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AD_Value_buf[1], 256);
			}
			else
			{
				mode=0;											//谐波模式
				ADC_ChannelConfTypeDef sConfig = {0};
				sConfig.Rank = ADC_REGULAR_RANK_1;
				sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//选择最小通道采样时间，提高最大可见频率
				
				hadc1.Init.ContinuousConvMode = ENABLE;			//开启连续转换模式
				hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;//软件触发采样
				HAL_ADC_Init(&hadc1);
				
				HAL_TIM_Base_Stop_IT(&htim3);
				HAL_ADC_Start(&hadc1);
			}
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
