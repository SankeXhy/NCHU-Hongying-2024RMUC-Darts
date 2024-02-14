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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "can3508.h"
#include "pid.h"
#include "control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
RC_ctrl ctl={0};
Filter_RC_ty Filter_RC;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern pid_struct_t M3508_1_PID_S;
extern pid_struct_t M3508_2_PID_S;
extern pid_struct_t M2006_1_PID_S;

extern pid_struct_t M3508_1_PID_P;
extern pid_struct_t M3508_2_PID_P;
extern M3508_information M3508_1;
extern M3508_information M3508_2;

extern M3508_information M2006_1;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int M3508_1_Speed=0;
int M3508_2_Speed=0;
int M2006_1_Speed=0;

int M3508_1_Position=6000;
int M3508_2_Position=6000;
int Dart_shoot_flag=0;
uint16_t Dart_Key_flag=0;
uint8_t sbus_buf[18u]={0};

uint8_t Step_Enable[]={0x03,0xF3,0xAB,0x01,0x00,0x6B};
uint8_t Step_Disable[]={0x03,0xF3,0xAB,0x00,0x00,0x6B};
uint8_t Step_CW[]={0x03,0xF6,0x01,0x01,0x3B,0x0A,0x00,0x6B};//上
uint8_t Step_CCW[]={0x03,0xF6,0x00,0x01,0x3B ,0x0A,0x00,0x6B};//下
uint8_t Step_Stop[]={0x03,0xFE,0x98,0x00,0x6B};
uint8_t Step_90_CW[]={0x01,0xFD,0x00,0x00,0xDC,0x01,0x00,0x00,0x03,0x20,0x00,0x00,0x6B};//16细分下3200个脉冲表示一圈

uint16_t pwm_SE=500;
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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  can_filter_init();
  M3508_PID_init();
  HAL_CAN_Start(&hcan1);
  HAL_UART_Receive_DMA(&huart1, sbus_buf, 18);
  Filter_RC.Filter_a=0.47;

  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,500);//扳机归位
  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,1500);//送弹归位
  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,500);//四号弹归位
  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,500);//三号弹归位
  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,500);//二号弹归位
  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,500);//一号弹归位

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    if(Dart_shoot_flag==1)//启动�???次发�???
	  {
		  HAL_Delay(200);
		  M3508_1_Speed=-2000;
		  M3508_2_Speed=2000;
		  for(int i=0;;i++)//回拉上膛，扳机打�???
		  {
			  HAL_Delay(5);
			  if(i==200)
			  {
				  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1500);
			  }
			  Dart_Key_flag=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1);
			  if(Dart_Key_flag==0)
			  {
				  M3508_1_Speed=0;
				  M3508_2_Speed=0;
				  break;
			  }
		  }
		  HAL_Delay(500);
		  for(int i=0;i<=200;i++)//扳机关闭
		  {
			  HAL_Delay(5);
			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,500);
		  }
		  M3508_1_Speed=2000;
		  M3508_2_Speed=-2000;
		  for(int i=0;i<=400;i++)
		  {
			  HAL_Delay(5);
			  Dart_Key_flag=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1);
			  if(Dart_Key_flag==0)
			  {
				  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, RESET);
			  }
		  }
		  M3508_1_Speed=0;
		  M3508_2_Speed=0;
		  for(int i=0;i<=20;i++)
		  {
			  HAL_Delay(5);
		  }
		  HAL_Delay(500);
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1500);//发射
		  HAL_Delay(1000);
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,500);//归位
		  Dart_shoot_flag=0;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_DMA(&huart1, sbus_buf, 18);
	if(huart==&huart1)
	{
		DR16_Analysis(sbus_buf);
		// if(ctl.rc.ch1>=30||ctl.rc.ch1<=-30)//右�?�道运动合成
		// {
		// 	Filter_RC.Filter_Now_ch1=Filter_RC.Filter_a*Filter_RC.Now_Sap_ch1+(1-Filter_RC.Filter_a)*Filter_RC.Filter_Last_ch1;
		// 	M3508_1_Speed=-msp(Filter_RC.Filter_Now_ch1,-660,660,-3000,3000);
		// 	M3508_2_Speed=msp(Filter_RC.Filter_Now_ch1,-660,660,-3000,3000);
		// 	LIMIT_Speed_M3508();
		// 	Filter_RC.Filter_Last_ch1=Filter_RC.Filter_Now_ch1;
		// }
		// else
		// {
		// 	M3508_1_Speed=0;
		// 	M3508_2_Speed=0;
		// }
//右拨杆
		if(ctl.rc.s1==3)
		{
			// M3508_1_Speed=0;
			// M3508_2_Speed=0;
      // HAL_UART_Transmit_IT(&huart6,Step_Stop,sizeof(Step_Stop));
		}
		else if(ctl.rc.s1==2)
		{
			// M3508_1_Speed=-2000;
			// M3508_2_Speed=2000;
			Dart_shoot_flag=1;
      // HAL_UART_Transmit_IT(&huart6,Step_CCW,sizeof(Step_CCW));
		}
		else if(ctl.rc.s1==1)
		{
			// M3508_1_Speed=2000;
			// M3508_2_Speed=-2000;
      // HAL_UART_Transmit_IT(&huart6,Step_CW,sizeof(Step_CW));
		}
//左摇杆-前后
		if(ctl.rc.ch3>=30||ctl.rc.ch3<=-30)//左�?�道运动合成
		{
			Filter_RC.Filter_Now_ch3=Filter_RC.Filter_a*Filter_RC.Now_Sap_ch3+(1-Filter_RC.Filter_a)*Filter_RC.Filter_Last_ch3;
			M2006_1_Speed=msp(Filter_RC.Filter_Now_ch3,-660,660,-15000,15000);
			LIMIT_Speed_M2006();
			Filter_RC.Filter_Last_ch3=Filter_RC.Filter_Now_ch3;
		}
		else
		{
			M2006_1_Speed=0;
		}
//左拨杆
		if(ctl.rc.s2==3)
		{
      // __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,500);//扳机归位
      // __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,500);//一号弹归位
		}
		else if(ctl.rc.s2==2)
		{
	    // __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1500);//扳机打开90度
      // __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,1100);//一号弹放下
		}
//右摇杆-左右
    if(ctl.rc.ch0>=30||ctl.rc.ch0<=-30)
    {
      if(ctl.rc.ch0>=30) HAL_UART_Transmit_IT(&huart6,Step_CW,sizeof(Step_CW));
      if(ctl.rc.ch0<=-30) HAL_UART_Transmit_IT(&huart6,Step_CCW,sizeof(Step_CCW));
    }
    else
    {
      HAL_UART_Transmit_IT(&huart6,Step_Stop,sizeof(Step_Stop));
    }
    
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim7)
	{
		  M3508_1_PID_S.output=pid_calc(&M3508_1_PID_S, M3508_1_Speed, M3508_1.rotor_speed);
		  M3508_2_PID_S.output=pid_calc(&M3508_2_PID_S, M3508_2_Speed, M3508_2.rotor_speed);
		  M2006_1_PID_S.output=pid_calc(&M2006_1_PID_S, M2006_1_Speed, M2006_1.rotor_speed);
		  set_M3508_1_motor_voltage(&hcan1,M3508_1_PID_S.output,M3508_2_PID_S.output,
				  M2006_1_PID_S.output,0);
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
