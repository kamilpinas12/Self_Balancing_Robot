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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>


#include "stepper_lib.h"
#include "mpu6050.h"
#include "low_pass_filter.h"
#include "helpers.h"
#include "pid_lib.h"
#include "uart_interface.h"
#include "user_functions.h"
#include "robot.h"

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

// uart interface
extern DMA_HandleTypeDef hdma_usart1_rx;
uart_interface_typedef uart_interface;

volatile stepper_typedef stepper1;
volatile stepper_typedef stepper2;


extern robot_typedef robot;


uint8_t encoder_data_buffer[2];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == stepper1.htim){
		stepper_update(&stepper1);
	}
	else if(htim == stepper2.htim){
		stepper_update(&stepper2);
	}
	else if (htim == &htim4){
		mpu_get_data_x_angle_DMA(robot.mpu);
		HAL_I2C_Mem_Read_IT(&hi2c2, 0x6C, 0x0C, 1, encoder_data_buffer, 2);
	}
	else if(htim == &htim1){
		HAL_ADC_Start_IT(&hadc1);
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc == &hadc1){
		robot.battery_voltage = HAL_ADC_GetValue(&hadc1) * 0.0045934;
		if(robot.battery_voltage < 10.8){
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			if(robot.battery_voltage < 10.5){
				robot_stop();
			}
		}
		else{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		}

	}
}

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // stepper motor setup
  stepper_init(&stepper1, &htim2, TIM_CHANNEL_1, ENABLE_GPIO_Port, ENABLE_Pin,
		  DIR1_GPIO_Port, DIR1_Pin, 6000, -1);
  stepper_init(&stepper2, &htim3, TIM_CHANNEL_1, ENABLE_GPIO_Port, ENABLE_Pin,
		  DIR2_GPIO_Port, DIR2_Pin, 6000, 1);


  // MPU setup
  mpu6050_typedef mpu = mpu_init(&hi2c1, 0xD0);
  if(mpu_who_am_i(&mpu) != HAL_OK){
	  while(1){
		  printf("Error while connecting to mpu 6050\n");
		  mpu = mpu_init(&hi2c1, 0xD0);
		  HAL_Delay(1000);
	  }
  }
  set_gyro_scale(&mpu, range_250);
  set_accelerometer_scale(&mpu, range_2g);
  mpu_low_pass_filter(&mpu, Acc94Hz_Gyro98Hz);
  HAL_Delay(800);

  mpu_gyro_calibration(&mpu);




  // UART interface init
  user_function_typedef user_function_array[] = {
  		{&led, "led", 1},
  		{&comunication_test, "test", 0},
		{&motor_test, "motor_test", 2},
		{&motor_enable, "motor_enable", 1},
		{&controler_start, "start", 0},
		{&controler_stop, "stop", 0},
		{&send_data, "send_data", 1},
		{&battery_voltage, "battery", 0},
  		{&set_position, "set_pos", 1},
		{&rotate_deg, "rotate", 1},
		{&get_angle, "angle", 0},
		{&move, "move", 1}
  };

  uart_interface_init(&uart_interface, &huart1, &hdma_usart1_rx, user_function_array, sizeof(user_function_array) / sizeof(user_function_typedef));


  // robot init
  robot.stepper1 = &stepper1;
  robot.stepper2 = &stepper2;
  robot.mpu = &mpu;
  robot_init();



  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim1);

  HAL_Delay(50);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  start_uart_interface(&uart_interface);


  while (1){
	  if(fabsf(robot.mpu->x_angle) < 0.01 && !robot.control_on){
		  robot_start();
	  }
	  else if(fabs(robot.mpu->x_angle) > 0.7 && robot.control_on){
		  robot_stop();
	  }


	  if(uart_interface.command_received_flag){
		  execute_received_command(&uart_interface);
		  start_uart_interface(&uart_interface);
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  robot_stop();

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
