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
#include "low_pass_filter.h"
#include "helpers.h"

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
volatile stepper_typedef stepper1;
volatile stepper_typedef stepper2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


int __io_putchar(int ch)
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


  // MPU setup
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
  mpu_low_pass_filter(&mpu, Acc44Hz_Gyro42Hz);
  HAL_Delay(300);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // balancing parameters
  int delay = 3;
  float kp = 500;
  float ki = 8;
  float kd = 1500;

  float i = 0;
  float pid = 0;

  float lst_error = 0;

//  derivative filter
//  float derivative_filter_alpha = 0.7;
//  float prev_error_d;

  // pos hold
  float pos_hold_angle = 0;
  float k_pos_hold = 0.00004;
  float d_pos_hold = 0.02;
  float speed_threshold = 0.05;


  long prev_pos;
  float prev_speed;

  float alfa_filter_speed = 0.1;



  double target_angle = 0;
  float k_target_angle = 0.003;


  unsigned long lst_time = HAL_GetTick();


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
		  i = 0;
		  pid = 0;
		  lst_error = 0;
		  target_angle = 0;
		  prev_pos = 0;
		  prev_speed = 0;
		  //prev_error_d = 0;

		  stepper_enable(&stepper1, 1);
		  stepper_enable(&stepper2, 1);
		  stepper1.step_counter = 0;
		  stepper2.step_counter = 0;

		  // main loop
		  while(fabsf(mpu.x_angle) < 0.7){
			  if((HAL_GetTick() - mpu.lst_time_x_angle) >= delay){
				  mpu_calc_x_angle(&mpu);
				  float time_delta = (mpu.lst_time_x_angle - lst_time);

				  if(abs(stepper1.step_counter) > 0){  // 0 for test
					  pos_hold_angle = -stepper1.step_counter * k_pos_hold;

					  //pos_hold_angle saturation
					  if(pos_hold_angle > 0.04) pos_hold_angle = 0.04;
					  else if(pos_hold_angle < -0.04) pos_hold_angle = -0.04;
				  }
				  else{
					  pos_hold_angle = 0;
				  }


				  float speed = (stepper1.step_counter - prev_pos) / time_delta;
				  prev_pos = stepper1.step_counter;

				  //speed filter

				  speed = alfa_filter_speed * speed + (1 - alfa_filter_speed) * prev_speed;

				  if(fabs(speed) < speed_threshold) speed = 0;

				  prev_speed = speed;

				  float derivative_pos = 0;

				  if(fabs(speed) > speed_threshold){
					  derivative_pos = speed * d_pos_hold;
					  pos_hold_angle -= derivative_pos;
				  }

				  double error = pos_hold_angle + target_angle - mpu.x_angle;
				  lst_time = mpu.lst_time_x_angle;


				  float p = error * kp;
				  i += ((error + lst_error) * time_delta * ki) / 2;

				  // integral saturation
				  if(i > 60) i = 60;
				  if(i < -60) i = -60;

				  float d = kd * (error - lst_error)/time_delta;

				  pid = p + i + d;


				  //saturation
				  if(pid > 100) pid = 100;
				  if(pid < -100) pid = -100;
				  stepper_set_speed(&stepper1, pid);
				  stepper_set_speed(&stepper2, pid);

				  lst_error = error;


				  double delta_target_angle = time_delta * k_target_angle * error;
				  target_angle += delta_target_angle;



				  //printf("%d, %d\n", stepper1.step_counter, stepper2.step_counter);
				  printf("%.4f,%.4f,%.4f,%.4f,%.4f,%ld\n", mpu.x_angle, p, speed, derivative_pos, target_angle, stepper1.step_counter);
				  //printf("%.3f, %.3f\n", speed, pid);
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
