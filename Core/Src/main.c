/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "oled.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include <assert.h>
#include<stdbool.h>
#include<math.h>
#include <stdint.h>
#include "MPU.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// //云台模式选择
// typedef enum
// {
//   Gimbal_Mode_Normal = 0,
//   Gimbal_Mode_PID
// }GimbalMode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
float yaw = 90.0f, pitch = 90.0f;    //偏航角、俯仰角

// static PID_TypeDef PID_Yaw;
// static PID_TypeDef PID_Pitch;
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// GimbalMode Read_Button(GPIO_TypeDef *Port, uint16_t Pin, GPIO_PinState CurrentState, GPIO_PinState CandidateState);    //读取按键状态
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  Wake_MPU();    //唤醒MPU6050

  OLED_Init();    //初始化OLED
  OLED_Clear();    //清屏
  OLED_ShowString(0, 0, "Yaw:", 16, 0);
  OLED_ShowString(0, 2, "Pitch:", 16, 0);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);    //启动偏航舵机PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);    //启动俯仰舵机PWM

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Calculate_Oula();    //计算欧拉角

    //限制偏航角的范围
    if (yaw < 0.0f)
    {
      yaw = 0.0f;
    }
    if (yaw > 180.0f)
    {
      yaw = 180.0f;
    }

    //限制俯仰角的范围
    if (pitch < 60.0f)
    {
      pitch = 60.0f;
    }
    if (pitch > 120.0f)
    {
      pitch = 120.0f;
    }

    uint8_t Yaw_Angle;    //普通模式偏航舵机角度
    uint8_t Pitch_Angle;    //普通模式俯仰舵机角度

    Yaw_Angle = (uint8_t)(yaw);    //将偏航角转换为偏航舵机角度
    Pitch_Angle = (uint8_t)(pitch);    //将俯仰角转换为俯仰舵机角度

    control_Yaw_Angle(Yaw_Angle);
    control_Pitch_Angle(Pitch_Angle);

    OLED_ShowNum(32, 0, Yaw_Angle, 3, 16, 0);
    OLED_ShowNum(48, 2, Pitch_Angle, 3, 16, 0);

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
}

/* USER CODE BEGIN 4 */
//用状态机消抖的方式读取按键的状态
// GimbalMode Read_Button(GPIO_TypeDef *Port, uint16_t Pin, GPIO_PinState CurrentState, GPIO_PinState CandidateState)
// {

//   uint32_t CandidateTime = 0U;  //出现候选状态的时间

//   GPIO_PinState NowState = HAL_GPIO_ReadPin(Port, Pin);    //按键现在的状态
//   uint32_t NowTime = HAL_GetTick();   //现在的时间

//   //检测到候选电平改变
//   if(NowState != CandidateState)
//   {
//     CandidateState = NowState;
//     CandidateTime = NowTime; 
//   }
//   else if((CandidateState != CurrentState) && (NowTime - CandidateTime) >= 20U)
//   {
//     CurrentState = CandidateState;
//   }

//   return (CurrentState == GPIO_PIN_RESET) ? Gimbal_Mode_PID : Gimbal_Mode_Normal;
// }

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
#ifdef USE_FULL_ASSERT
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
