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

int32_t set_pos = 0;



pid_typedef pos_pid;

uart_interface_typedef uart_interface; // global !!!

int16_t spin_duration_ms = 0;
int8_t spin_value = 0;
float set_angle = 0;
float robot_angle;




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
  mpu_low_pass_filter(&mpu, Acc21Hz_Gyro20Hz);
  HAL_Delay(300);


  mpu_gyro_calibration(&mpu);



  user_function_typedef user_function_array[] = {
  		{&led, "led", 1},
  		{&comunication_test, "halo", 0},
  		{&set_position, "set_pos", 1},
		{&set_angle_fun, "set_angle", 2},
		{&rotate_deg, "rotate", 2}

  };


  uart_interface_init(&uart_interface, &huart1, &hdma_usart1_rx, user_function_array, sizeof(user_function_array) / sizeof(user_function_typedef));


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // control parameters
  int delay = 4;


  pid_typedef angle_pid = pid_init(480, 6, 1200);
  pid_typedef pos_pid = pid_init(0.003, 0.000002, 0.006);


  filter_typedef pos_pid_filter = filter_init(0.05);
  filter_typedef pos_pid_derivative_filter = filter_init(0.005);
  filter_typedef angle_pid_derivative_filter = filter_init(0.3);
  filter_typedef output_filter = filter_init(0.8);
  filter_typedef gain_factor_filter = filter_init(0.1);
  filter_typedef robot_speed_filter = filter_init(0.1);
  filter_typedef target_angle_filter = filter_init(0.4);
  filter_typedef desired_speed_fitler = filter_init(0.1);



  unsigned long lst_time = HAL_GetTick();


  while (1)
  {
	  mpu_calc_x_angle(&mpu);

	  execute_received_command(&uart_interface);
	  start_uart_interface(&uart_interface);

	  if(fabsf(mpu.x_angle) < 0.01){
		  // set up before entering main loop
		  lst_time = HAL_GetTick();

		  // start motors
		  stepper_enable(&stepper1, 1);
		  stepper_enable(&stepper2, 1);

		  //restart motor position
		  stepper1.step_counter = 0;
		  stepper2.step_counter = 0;
		  set_pos = 0;

		  spin_duration_ms = 0;

		  // main loop
		  while(fabsf(mpu.x_angle) < 0.7){
			  if((HAL_GetTick() - mpu.lst_time_x_angle) >= delay){

				  mpu_calc_x_angle(&mpu);
				  float time_delta = (mpu.lst_time_x_angle - lst_time);
				  lst_time = mpu.lst_time_x_angle;


				  float robot_speed = filter(&robot_speed_filter, (stepper1.speed + stepper2.speed) / 2);
				  float position_error = -((stepper2.step_counter + stepper1.step_counter)/2) - set_pos;
				  robot_angle = (360.0 * 21.3)/(2*3.14*15.5 * 1600.0) * (stepper2.step_counter - stepper1.step_counter);


				  // pos hold PID
				  float desired_speed = (position_error * 0.05);
				  saturation(-60, 60, &desired_speed);
				  desired_speed = filter(&desired_speed_fitler, desired_speed);

				  float error = robot_speed - desired_speed;
				  saturation(-50, 50, &error);

				  pos_pid.error = filter(&pos_pid_filter, error);
				  pos_pid.measurement = filter(&pos_pid_derivative_filter, robot_speed);


				  float gain_factor = 1;
				  if(fabs(position_error) < 130 && fabs(robot_speed) < 30 && fabs((pos_pid.measurement - pos_pid.prev_measurement)/time_delta) < 1){
					  gain_factor = fabs(position_error) / 130;
					  pos_pid.ki = 0.000002;
				  }
				  else{
					  pos_pid.ki = 0.000001;
				  }

				  gain_factor = filter(&gain_factor_filter, gain_factor);


				  // pos P
				  float p_pos = gain_factor * pos_pid.error * pos_pid.kp;

				  // pos I
				  pos_pid.i += 0.5 * (pos_pid.error + pos_pid.prev_error) * pos_pid.ki * time_delta;
				  saturation(-0.55, 0.55, &pos_pid.i);

				  // pos D
				  float d_pos = gain_factor * pos_pid.kd * (pos_pid.measurement - pos_pid.prev_measurement) / time_delta;



				  //target angle
				  float target_angle = filter(&target_angle_filter, p_pos + pos_pid.i + d_pos);
				  saturation(-0.5, 0.5, &target_angle);



				  // angle PID
				  angle_pid.error =  target_angle - mpu.x_angle;
				  angle_pid.measurement = -mpu.x_angle;

				  // P angle
				  float p = angle_pid.error * angle_pid.kp;

				  // I angle
				  angle_pid.i += ((angle_pid.error + angle_pid.prev_error) * time_delta * angle_pid.ki) / 2;
				  saturation(-60, 60, &angle_pid.i);

				  // D angle
				  pos_pid.measurement = filter(&angle_pid_derivative_filter, angle_pid.measurement);
				  float d = angle_pid.kd * (pos_pid.measurement - angle_pid.prev_measurement)/time_delta;

				  float pid = p + angle_pid.i + d;

				  // delete spikes
				  float delta = pid - output_filter.prev_value;
				  saturation(-4.5, 4.5, &delta);

				  pid = filter(&output_filter, output_filter.prev_value + delta);

				  // delete peaks




				  stepper_set_speed(&stepper1, pid);
				  stepper_set_speed(&stepper2, pid);


//				  if(spin_duration_ms > 0){
//					  stepper_set_speed(&stepper1, pid + spin_value);
//				  	  stepper_set_speed(&stepper2, pid - spin_value);
//				  	  spin_duration_ms --;
//				  }
//				  else{
//					  stepper_set_speed(&stepper1, pid);
//					  stepper_set_speed(&stepper2, pid);
//				  }




				  pid_step(&angle_pid);
				  pid_step(&pos_pid);


				  execute_received_command(&uart_interface);
				  start_uart_interface(&uart_interface);

				  //printf("%d, %d\n", stepper1.step_counter, stepper2.step_counter);
				  //printf("%.4f,%.4f,%.4f,%.4f,%.4f,%ld\n", mpu.x_angle, p, speed, derivative_pos, target_angle, stepper1.step_counter);
				  //printf("%.3f, %.3f\n", speed, pid);
				  //printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", robot_speed, desired_speed, target_angle, p_pos, pos_pid.i, d_pos);
				  //printf("%.3f, %.3f, %.3f, %.3f, %ld\n", mpu.x_angle, target_angle, desired_speed, robot_speed, stepper1.step_counter);
				  //printf("%.5f,%.3f,%ld\n", mpu.x_angle, pid, mpu.lst_time_x_angle);
				  //printf("%.3f,%.1f,%ld\n", mpu.x_angle, pid, HAL_GetTick());


				  // send data

				  uint8_t buffer[BUFFER_SIZE_TX];
				  //uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "%ld, %ld, %.3f\r\n", stepper1.step_counter, stepper2.step_counter, robot_angle);
				  uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", robot_speed, target_angle, delta, p_pos, pos_pid.i, d_pos, position_error);
				  //uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "%.3f, %.3f\n", robot_speed - desired_speed, pos_pid.error);

				  //uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "%.3f, %.3f\n", mpu.x_angle, target_angle);

				  uart_send(&uart_interface, buffer, size, 0);


			  }
		  }
		  //stop motors
		  stepper_set_speed(&stepper1, 0);
		  stepper_set_speed(&stepper2, 0);
		  stepper_enable(&stepper1, 0);
		  stepper_enable(&stepper2, 0);

		  pid_reset(&angle_pid);
		  pid_reset(&pos_pid);


		  reset_filter(&pos_pid_filter);
		  reset_filter(&angle_pid_derivative_filter);
		  reset_filter(&pos_pid_derivative_filter);
		  reset_filter(&output_filter);
		  reset_filter(&gain_factor_filter);
		  reset_filter(&robot_speed_filter);
		  reset_filter(&target_angle_filter);
		  reset_filter(&desired_speed_fitler);


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

  stepper_set_speed(&stepper1, 0);
  stepper_set_speed(&stepper2, 0);
  stepper_enable(&stepper1, 0);
  stepper_enable(&stepper2, 0);

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
