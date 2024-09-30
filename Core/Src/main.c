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


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define mSEC2SEC 0.001

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
    if (ch == '\n') {
        uint8_t ch2 = '\r';
        HAL_UART_Transmit(&huart1, &ch2, 1, HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
}



volatile stepper_typedef stepper1;
volatile stepper_typedef stepper2;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == stepper1.htim){
		stepper_update(&stepper1);
	}
	else if(htim == stepper2.htim){
		stepper_update(&stepper2);
	}
}


void test_fun(mpu6050_typedef* mpu){
	HAL_Delay(1000);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

//	for(int i = 0; i < 3000; i++){
//		mpu_calc_x_angle(mpu);
//		printf("%f,%f,%ld\n", mpu_get_acc_x_angle(mpu), mpu->gy, mpu->lst_time_x_angle);
//	}


// measure stepper motor time response

	HAL_Delay(5000);
	int speed = 0;

	stepper_set_speed(&stepper1, speed);
	stepper_set_speed(&stepper2, speed);

	for(int i = 0; i < 450; i++){
		if(i == 60) {
			speed = 15;
			stepper_enable(&stepper1, 1);
			stepper_enable(&stepper2, 1);
		}

		stepper_set_speed(&stepper1, speed);
		stepper_set_speed(&stepper2, speed);
		mpu_get_data(mpu);
		printf("%.5f,%d,%ld\n", mpu->gy, speed, HAL_GetTick());
	}

	speed = 0;
	stepper_enable(&stepper1, 0);
	stepper_enable(&stepper2, 0);
	stepper_set_speed(&stepper1, speed);
	stepper_set_speed(&stepper2, speed);


// robot's step response
//	int speed = 0;
//	stepper_enable(&stepper1, 1);
//	stepper_enable(&stepper2, 1);
//
//	while(1){
//
//		mpu_calc_x_angle(mpu);
//		speed = 0;
//	    stepper_set_speed(&stepper1, speed);
//	    stepper_set_speed(&stepper2, speed);
//
//		if(fabs(mpu->x_angle) < 0.03){
//			while(fabs(mpu->x_angle) < 0.7){
//				mpu_calc_x_angle(mpu);
//				if(mpu->x_angle > 0.15){
//					speed = -40;
//				}
//				else if(mpu->x_angle < -0.15){
//					speed = 40;
//				}
//			    stepper_set_speed(&stepper1, speed);
//			    stepper_set_speed(&stepper2, speed);
//
//
//				printf("%.5f,%d,%ld\n", mpu->x_angle, speed, HAL_GetTick());
//
//
//			}
//		}


//	stepper_enable(&stepper1, 1);
//	stepper_enable(&stepper2, 1);
//	int speed = 0;
//
//
//	for(int i = 0; i < 500; i++){
//		mpu_get_data(mpu);
//		if(i == 200){
//			stepper_set_speed(&stepper1, -30);
//			stepper_set_speed(&stepper2, -30);
//			speed = 1;
//		}
//		printf("%f,%d,%ld\n", mpu->ax, speed, HAL_GetTick());
//
//	}
//
//	stepper_set_speed(&stepper1, 0);
//	stepper_set_speed(&stepper2, 0);
//
//	stepper_enable(&stepper1, 0);
//	stepper_enable(&stepper2, 0);


	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	HAL_Delay(100000);

}



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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // init stepper
  stepper_init(&stepper1, &htim2, TIM_CHANNEL_1, ENABLE_GPIO_Port, ENABLE_Pin,
		  DIR1_GPIO_Port, DIR1_Pin, 6000, -1);
  stepper_init(&stepper2, &htim3, TIM_CHANNEL_1, ENABLE_GPIO_Port, ENABLE_Pin,
		  DIR2_GPIO_Port, DIR2_Pin, 6000, 1);


  // init mpu6050
  mpu6050_typedef mpu = mpu_init(&hi2c1, 0xD0);

  if(mpu_who_am_i(&mpu) != HAL_OK)
  {
	  while(1)
	  {
		  printf("Error while connecting to mpu 6050\n");
		  HAL_Delay(1000);
	  }
  }

  set_gyro_scale(&mpu, range_250);
  set_accelerometer_scale(&mpu, range_2g);
  mpu_low_pass_filter(&mpu, Acc21Hz_Gyro20Hz);

  HAL_Delay(1000);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // parameters
  int delay = 3;
  float kp = 750;
  float ki = 3500;
  float kd = 2.5;

  float p = 0;
  float i = 0;
  float d = 0;
  float pid = 0;


  float error = 0;
  float lst_error = 0;

  float set_angle = 0;
  float set_speed = 0;


  float k_set_angle = 0.004;
  //float k_set_speed = 0.05;

  //float set_pos = 0;

  unsigned long lst_time = HAL_GetTick();
  float time_delta = 0;
  //float angle_offset = 0;


  mpu_gyro_calibration(&mpu);





  while (1)
  {
	  HAL_Delay(delay);
	  stepper_enable(&stepper1, 0);
	  stepper_enable(&stepper2, 0);
	  stepper_set_speed(&stepper1, 0);
	  stepper_set_speed(&stepper2, 0);


	  mpu_calc_x_angle(&mpu);



	  if(fabsf(mpu.x_angle) < 0.05){

		  lst_time = HAL_GetTick();
		  lst_error = 0;
		  i = 0;
		  set_angle = 0;

		  stepper_enable(&stepper1, 1);
		  stepper_enable(&stepper2, 1);
		  stepper1.step_counter = 0;
		  stepper2.step_counter = 0;

		  // main loop
		  while(fabsf(mpu.x_angle) < 0.7){
			  if((HAL_GetTick() - mpu.lst_time_x_angle) >= delay){
				  mpu_calc_x_angle(&mpu);


//				  set_speed = (set_pos - stepper1.step_counter) * k_set_speed;
//				  //saturation
//				  if(set_speed > 10) set_speed = 10;
//				  if(set_speed < -10) set_speed = -10;
//
//				  set_angle += (set_speed - pid) * k_set_angle * time_delta;



				  time_delta = (mpu.lst_time_x_angle - lst_time) * mSEC2SEC;

				  if (pid > 0) set_angle += time_delta * k_set_angle;
				  if (pid < 0) set_angle -= time_delta * k_set_angle;


				  error = set_angle - mpu.x_angle;
				  lst_time = mpu.lst_time_x_angle;


				  p = error * kp;
				  i += ((error + lst_error) * time_delta * ki) / 2;

				  // integral saturation
				  if(i > 60) i = 60;
				  if(i < -60) i = -60;

				  d = kd * (error - lst_error)/time_delta;

				  pid = p + i + d;

				  //if(fabs(pid) < 0.04) pid = 0;

				  //saturation
				  if(pid > 100) pid = 100;
				  if(pid < -100) pid = -100;
				  stepper_set_speed(&stepper1, pid);
				  stepper_set_speed(&stepper2, pid);

				  lst_error = error;
				  //printf("%d, %d\n", stepper1.step_counter, stepper2.step_counter);
				  printf("%.3f; %.1f; %.1f; %.1f, %.3f, %.3f\n", mpu.x_angle, p, i, d, set_speed, set_angle);
				  //printf("%.5f,%.3f,%ld\n", mpu.x_angle, pid, mpu.lst_time_x_angle);

				  //printf("%.3f,%.1f,%ld\n", mpu.x_angle, pid, HAL_GetTick());
			  }


		  }
		  // for debuding
		  stepper_enable(&stepper1, 0);
		  stepper_enable(&stepper2, 0);
		  i = 0;
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
