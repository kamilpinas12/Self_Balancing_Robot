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

extern DMA_HandleTypeDef hdma_usart1_rx;

volatile stepper_typedef stepper1;
volatile stepper_typedef stepper2;

volatile int32_t set_pos = 0;

uart_interface_typedef uart_interface; // global !!!

volatile uint16_t spin_duration_ms = 0;
volatile int8_t spin_value = 0;


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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // stepper motor setup
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



  user_function_typedef user_function_array[] = {
  		{&led, "led", 1},
  		{&comunication_test, "halo", 0},
  		{&set_position, "set_pos", 1},
		{&rotate, "rotate", 2}
  };


  uart_interface_init(&uart_interface, &huart1, &hdma_usart1_rx, user_function_array, sizeof(user_function_array) / sizeof(user_function_typedef));

  start_uart_interface(&uart_interface);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // balancing parameters
  int delay = 4;


  pid_typedef angle_pid = pid_init(450, 6, 1200);
  //pid_typedef pos_pid = pid_init(0.00004, 0, 0.02);

  filter_typedef speed_filter = filter_init(0.1);
  filter_typedef set_pos_filter = filter_init(0.01);


  // pos hold
  float k_pos_hold = 0.00004;
  float d_pos_hold = 0.03;
  float speed_threshold = 0.05;







  long prev_pos;
  float prev_speed;

  float alfa_filter_speed = 0.1;



  float target_angle = 0;



  unsigned long lst_time = HAL_GetTick();


  mpu_gyro_calibration(&mpu);


  while (1)
  {


	  //stop motors
	  stepper_enable(&stepper1, 0);
	  stepper_enable(&stepper2, 0);
	  stepper_set_speed(&stepper1, 0);
	  stepper_set_speed(&stepper2, 0);

	  mpu_calc_x_angle(&mpu);

	  execute_uart_command(&uart_interface);
	  start_uart_interface(&uart_interface);


	  if(fabsf(mpu.x_angle) < 0.01){


		  pid_reset(&angle_pid);
		  //pid_reset(&pos_pid);

		  reset_filter(&speed_filter);
		  reset_filter(&set_pos_filter);


		  lst_time = HAL_GetTick();


		  target_angle = 0;
		  prev_pos = 0;
		  prev_speed = 0;
		  set_pos = 0;


		  // start motor
		  stepper_enable(&stepper1, 1);
		  stepper_enable(&stepper2, 1);

		  //restart motor position
		  stepper1.step_counter = 0;
		  stepper2.step_counter = 0;

		  // control loop
		  while(fabsf(mpu.x_angle) < 0.5){
			  if((HAL_GetTick() - mpu.lst_time_x_angle) >= delay){

				  mpu_calc_x_angle(&mpu);
				  float time_delta = (mpu.lst_time_x_angle - lst_time);
				  lst_time = mpu.lst_time_x_angle;


				  float pos_error = set_pos - stepper1.step_counter;

				  saturation(-900, 900, &pos_error);
				  pos_error = filter(&set_pos_filter, pos_error);

				  float pos_hold_angle = pos_error * k_pos_hold;
				  saturation(-0.25, 0.25, &pos_hold_angle);


				  float speed = (stepper1.step_counter - prev_pos) / time_delta;
				  prev_pos = stepper1.step_counter;

				  //speed filter
				  speed = alfa_filter_speed * speed + (1 - alfa_filter_speed) * prev_speed;


				  // turn off derivative part if robot is balanced and in correct position
				  if(fabs(speed) < speed_threshold) speed = 0;


				  prev_speed = speed;

				  float derivative_pos = 0;

				  if(fabs(speed) > speed_threshold){
					  derivative_pos = speed * d_pos_hold;
					  pos_hold_angle -= derivative_pos;
				  }


				  // angle PID
				  angle_pid.error = pos_hold_angle + target_angle - mpu.x_angle;

				  // P
				  float p = angle_pid.error * angle_pid.kp;

				  // I with staturation
				  angle_pid.i += ((angle_pid.error + angle_pid.prev_error) * time_delta * angle_pid.ki) / 2;
				  saturation(-60, 60, &angle_pid.i);

				  // D
				  float d = angle_pid.kd * (angle_pid.error - angle_pid.prev_error)/time_delta;

				  float pid = p + angle_pid.i + d;

				  if(spin_duration_ms > 0){
					  stepper_set_speed(&stepper1, pid + spin_value);
				  	  stepper_set_speed(&stepper2, pid - spin_value);
				  	  spin_duration_ms --;
				  }
				  else{
					  stepper_set_speed(&stepper1, pid);
				  	  stepper_set_speed(&stepper2, pid);
				  }
				  angle_pid.prev_error = angle_pid.prev_error;



				  execute_uart_command(&uart_interface);
				  start_uart_interface(&uart_interface);

				  //printf("%d, %d\n", stepper1.step_counter, stepper2.step_counter);
				  //printf("%.4f,%.4f,%.4f,%.4f,%.4f,%ld\n", mpu.x_angle, p, speed, derivative_pos, target_angle, stepper1.step_counter);
				  //printf("%.3f, %.3f\n", speed, pid);
				  //printf("%.5f,%.3f,%ld\n", mpu.x_angle, pid, mpu.lst_time_x_angle);
				  //printf("%.3f,%.1f,%ld\n", mpu.x_angle, pid, HAL_GetTick());
			  }


		  }
		  // for debug
		  stepper_enable(&stepper1, 0);
		  stepper_enable(&stepper2, 0);
	  }


	  HAL_Delay(delay);
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
