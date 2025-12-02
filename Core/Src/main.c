/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
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

#include "AS5600.h"
#include <dwt_delay.h>
#include <stdio.h>
#include "pid.h"
#include "modbus.h"
#include "FOC.h"

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

FOC motor_foc = {0};
Tim motor_tim = {0};
Sensor motor_sensor = {0};
PID_Controller motor_position_pid = {0};

uint8_t TIM4_flag = 0;
uint16_t TIM4_fre = 0;

uint8_t TIM2_flag = 0;
uint16_t TIM2_fre = 0;

uint16_t sys_fre = 0;
uint32_t fre = 0;

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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  motor_foc.voltage_power_supply = 12.0f;
  motor_foc.voltage_limit = 12.0f;
  motor_foc.voltage_sensor_align = 3.0f;
  motor_foc.pole_pairs = 7;
  motor_foc.dir = 1;
  motor_foc.target_velocity = 100.0f;
  motor_foc.Uq = 0.0f;
  motor_foc.control_target = 3;
  motor_foc.zero_electric_angle = 2.502505f;

  motor_tim.port = htim3;
  motor_tim.channelA = TIM_CHANNEL_2;
  motor_tim.channelB = TIM_CHANNEL_3;
  motor_tim.channelC = TIM_CHANNEL_4;

  motor_position_pid.Kp = 10.0f;
  motor_position_pid.Ki = 0.0f;
  motor_position_pid.Kd = 1.0f;
  motor_position_pid.direction = 1;
  motor_position_pid.setpoint = 1.0f;

  motor_foc.velocity_pid.Kp = 0.2f;
  motor_foc.velocity_pid.Ki = 0.01f;
  motor_foc.velocity_pid.Kd = 0.0f;
  motor_foc.velocity_pid.direction = -1;
  motor_foc.velocity_pid.setpoint = 15.0f;

  Motor_LinkPositionPID(&motor_foc, &motor_position_pid);
  Motor_LinkSensor(&motor_foc, &motor_sensor);
  Motor_LinkTim(&motor_foc, &motor_tim);

  DWT_Timer_Init();
  modbus_init();

  uint8_t magnet_status = AS5600_checkMagnet();

  if (magnet_status == 0xFF)
  {
    // printf("No Magnet Found! Please check the magnet installation!\r\n");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // 常亮
  }
  else if (magnet_status == 2)
  {
    // printf("Magnet Too Strong! Please use a weaker magnet or increase the distance!\r\n");
  }
  else if (magnet_status == 1)
  {
    // printf("Magnet Too Weak! Please use a stronger magnet or decrease the distance!\r\n");
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // 常亮
    // printf("Magnet Detected! Starting Motor Control...\r\n");
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); // LED ON

  while (modbus_get_motor_start_flag() == 0)
  {
    modbus_loop();
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); // LED OFF

  foc_Init(&motor_foc);

  HAL_TIM_Base_Start_IT(&htim4); // 启动定时�?4中断
  HAL_TIM_Base_Start_IT(&htim2); // 启动定时�?2中断

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // sys_fre++;

    modbus_loop();

    if (TIM4_flag)
    {
      fre++;

      Pidloop(&motor_foc);
      foc_loop(&motor_foc);
      foc_move(&motor_foc);

      TIM4_flag = 0;
    }

    if (TIM2_flag)
    {
      // printf("Freq: %d Hz,angle_with_rotation: %f,Uq: %f,sys_fre: %d,target_angle: %f\r\n", fre, motor_foc.sensor.angle_with_rotations, motor_foc.Uq, sys_fre, motor_foc.position_pid.setpoint);
      //      printf("%f,%f,%f\r\n", motor_foc.position_pid.setpoint, motor_foc.sensor.angle_with_rotations, motor_foc.Uq);
      fre = 0;
      sys_fre = 0;
      TIM2_flag = 0;
    }
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
  return ch;
}

// TIM4中断服务函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
  {
    TIM4_flag = 1;
  }
  else if (htim->Instance == TIM2)
  {
    TIM2_flag = 1;
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
