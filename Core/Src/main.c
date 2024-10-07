/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "ps2.h"
#include "Motor.h"
#include "stm32f1xx_it.h"

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
uint16_t distance;
int16_t Num;		//定义待被旋转编码器调节的变量

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t GetCCRFromAngle(float InputAngle){
    float Ret=InputAngle/180 * 2000 +500;
    return Ret;
}

/*//将旋转编码器计的数转换为距离
uint16_t Distance_Count(int16_t Retcount){

}

void hhSerialSendByte(uint8_t Byte){
    HAL_UART_Transmit(&huart1, &Byte, 1, HAL_MAX_DELAY);
}
uint8_t Serial_RxFlag;
uint8_t Serial_GetRxFlag(void)
{
    if (Serial_RxFlag == 1)
    {
        Serial_RxFlag = 0;
        return 1;
    }
    return 0;
}
uint8_t Serial_RxPacket[4];
uint8_t ByteRecv;
//接收4个字节的数据�????????
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart == &huart1){
        static uint8_t RxState = 0;
        static uint8_t pRxPacket = 0;
        uint8_t RxData=ByteRecv;
        if (RxState == 0)
            {
                if (RxData == 0xFF)
                    {
                        RxState = 1;
                        pRxPacket = 0;
                    }
            }
        else if (RxState == 1)
            {
                Serial_RxPacket[pRxPacket] = RxData;
                pRxPacket ++;
                if (pRxPacket >= 4)
                    {
                        RxState = 2;
                    }
            }
        else if (RxState == 2)
            {
                if (RxData == 0xFE)
                {
                    RxState = 0;
                    Serial_RxFlag = 1;
                }
            }
//打开中断接收，为下次接收数据做准�????????
HAL_UART_Receive_IT(&huart1, &ByteRecv, 1);

    }
}*/
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
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

    OLED_Init();
    Motor_Init();
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 /* while (1){
      //打开中断接收，若接收到数据则会触发中�????????
      HAL_UART_Receive_IT(&huart1, &ByteRecv, 1);
      distance= Distance_Count(GetCountRet_M1());
      //接收串口数据�????????
      if(Serial_GetRxFlag()==1){

      }
    Car_Forward(1000);
    if(distance==100)
    {
        //停车
        Car_Stop();

        //以下代码对openmv传来的数据进行处理，控制舵机抓取小球


        //�????????出循环，进入遥控
        break;
    }
    }*/
  while (1)
  {
      Car_Stop();
      PS2_Read_Data();
      if(PS2_Data.Key_L_Up){
          Car_Forward(1000);
      } else if(PS2_Data.Key_L_Down){
          Car_Backward(1000);
      } else if (PS2_Data.Key_L_Left){
          Car_TurnLeft(500);
      } else if(PS2_Data.Key_L_Right){
          Car_TurnRight(500);
      }
      if (PS2_Data.Key_L1){
          __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, GetCCRFromAngle(0));
      } else if (PS2_Data.Key_R1){
          __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, GetCCRFromAngle(180));
      }
      OLED_ShowHexNum(1,1,PS2_Data.Key_L1,4);
      OLED_ShowHexNum(2,1,PS2_Data.Key_R1,4);
      OLED_ShowHexNum(3,1,PS2_Data.Rocker_LX,4);


      /*HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      HAL_Delay(500);*/



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
