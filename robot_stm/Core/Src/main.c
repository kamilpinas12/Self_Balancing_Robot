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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>


#include "stepper_lib.h"
#include "mpu6050.h"
#include "helpers.h"
#include "robot.h"
#include <communication_interface.h>

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


volatile stepper_typedef stepper1;
volatile stepper_typedef stepper2;

mpu6050_typedef mpu;

extern robot_t robot;
extern communication_interface_t communication_interface;
extern reg_typedef registerMap[REGISTER_MAP_SIZE];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


int __io_putchar(int ch)  //for printf
{
    if (ch == '\n') {
        uint8_t ch2 = '\r';
        HAL_UART_Transmit(&huart1, &ch2, 1, HAL_MAX_DELAY);
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == stepper1.htim){
		stepper_update(&stepper1);
	}
	else if(htim == stepper2.htim){
		stepper_update(&stepper2);
	}
	else if (htim == &htim4){
		mpu_get_data_x_angle_DMA(&mpu);
		//HAL_I2C_Mem_Read_IT(&hi2c2, 0x6C, 0x0C, 1, encoder_data_buffer, 2);  //encoder
	}
	else if(htim == &htim1){
		HAL_ADC_Start_IT(&hadc1);
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc == &hadc1){
		battery_voltage_update(HAL_ADC_GetValue(&hadc1));
	}
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){   // DMA transfer complete mpu
	if(hi2c == mpu.hi2c){
		DMA_transfer_complete_callback(&mpu);
		if(robot.control_enable) robot.data_ready = 1;

	}
	else if (hi2c == &hi2c2){  // encoder
		calculate_encoder_angle();
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


  /*##################################################
   * 			SETUP begin
   * ##################################################
   */

  robot_init();


  // stepper motor setup
   stepper_init(&stepper1, &htim2, TIM_CHANNEL_1, ENABLE_GPIO_Port, ENABLE_Pin,
 		  DIR1_GPIO_Port, DIR1_Pin, 45, -1);
   stepper_init(&stepper2, &htim3, TIM_CHANNEL_1, ENABLE_GPIO_Port, ENABLE_Pin,
 		  DIR2_GPIO_Port, DIR2_Pin, 45, 1);


   // MPU6050 setup
   mpu_init(&mpu, &hi2c1, 0xD0);
   set_gyro_scale(&mpu, range_250);
   set_accelerometer_scale(&mpu, range_2g);
   mpu_low_pass_filter(&mpu, Acc94Hz_Gyro98Hz);

   HAL_Delay(1000); // gives time to leave the robot stable after turn on
   mpu_gyro_calibration(&mpu);



   // communication protocol setup
   communication_interface_init(&huart1, &hdma_usart1_rx);

   registerMap[0].ptr = &robot.robot_enable;
   registerMap[0].dataSize = sizeof(robot.robot_enable);

   registerMap[1].ptr = &robot.set_pos;
   registerMap[1].dataSize = sizeof(robot.set_pos);

   registerMap[2].ptr = &robot.K1;
   registerMap[2].dataSize = sizeof(robot.K1);

   registerMap[3].ptr = &robot.K2;
   registerMap[3].dataSize = sizeof(robot.K2);

   registerMap[4].ptr = &robot.K3;
   registerMap[4].dataSize = sizeof(robot.K3);

   registerMap[5].ptr = &robot.K4;
   registerMap[5].dataSize = sizeof(robot.K4);

   registerMap[6].ptr = &robot.K5;
   registerMap[6].dataSize = sizeof(robot.K5);
   /*##################################################
    * 			SETUP end
    * ##################################################
    */


   HAL_TIM_Base_Start_IT(&htim4);
   HAL_TIM_Base_Start_IT(&htim1);

   HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */









   while (1)
  {
	if(robot.robot_enable){
		if(fabs(mpu.x_angle) < 0.05 && !robot.control_enable){
			robot_start();
		}
		if(fabs(mpu.x_angle)> 0.8 && robot.control_enable){
			robot_stop();
			HAL_Delay(2000);
		}
  	}
   	else{
   		robot_stop();
   		robot.control_enable = 0;
   	}

	if(robot.robot_enable){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	}

	if(robot.control_enable && robot.data_ready){
		control_step();

		//send data
		if(communication_interface.uart_tx_ready){
//		uint16_t size = snprintf((char*)communication_interface.transmit_buffer, Tx_BUFFER_SIZE,
//				"%.3f, %.3f, %.3f, %ld\r\n", mpu.x_angle, robot.d_angle, robot.speed_delta, HAL_GetTick());
		uint16_t size = snprintf((char*)communication_interface.transmit_buffer, Tx_BUFFER_SIZE,
				"%.3f,%.3f,%.3f,%0.3f\n", mpu.x_angle, robot.d_angle, robot.pos, robot.d_pos);
		uart_send_buffer(size);
		}

	}

	execute_command();

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
