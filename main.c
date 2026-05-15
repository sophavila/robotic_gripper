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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float s_angle=0;

#define T 0.005
#define taud 0.015
float e_m1;
int volt_M1;
int volt_M2;
int theta_M1;
int theta_M2;
int theta_M1_1= 0.0000;
int theta_M2_1= 0.0000;
float omega_M1 = 0.0000;
float omega_M2 = 0.0000;
float omegaScale_M1 = 0.0000;
float omegaScale_M2 = 0.0000;
float ki_M1 = 16;//0.25;//for tuning
float kp_M1 =1;//16;//for tuning
float kd_M1 =0;
float yd_M1;
float ki_M2 = 16;//for tuning
float kp_M2 = 1;//for tuning
float yd_M2;
volatile int pulse=0;
volatile int pulse_2=0;
float kp_pos=0.4;
float ki_pos=0.5;
float kd_pos=0.0007;
float kp_pos_2=0.1;
float ki_pos_2=0.0005;
float kd_pos_2=0.00100;
int volt_out_M1=0;
int volt_out_M2=0;
float integral_pos = 0;
float integral_spd = 0;
float prev_e_pos = 0;
float prev_e_spd = 0;
float integral_pos_2 = 0;
float integral_spd_2 = 0;
float prev_e_pos_2 = 0;
float prev_e_spd_2 = 0;
float angle=0;
float target_angle=0;
float pulses_per_degree=13.33333;
//float PULSES_PER_CM = 295;
float PULSES_PER_CM=11.0555;
float target_dist_cm =0;
float distance=0;
float actual_angle=0;
float cm=0;
int count=0;
uint8_t rx_data;
typedef enum {
    MODE_IDLE,      // Target 0
    MODE_LIFT,      // Target -20
    MODE_SPIN,      // Target -270
} ControlMode_t;

ControlMode_t current_mode = MODE_IDLE;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void voltage_M1 (int volt_M1){
	if (volt_M1 > 100){
		volt_M1=100;

	}
	if (volt_M1<-100){
		volt_M1=-100;
	}
	if (volt_M1>0){
	TIM1->CCR2 = volt_M1;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 0);
	}else{
	TIM1->CCR2 = -volt_M1;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 1);
	}
}
void voltage_M2 (int volt_M2){
	if (volt_M2 > 100){
		volt_M2=100;

	}
	if (volt_M2<-100){
		volt_M2=-100;
	}
	if (volt_M2>0){
	TIM1->CCR4 = volt_M2;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 1);
	}else{
	TIM1->CCR4 = -volt_M2;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0);
	}
}
void encoder(){
	theta_M1 = TIM5-> CNT;
	theta_M2 = TIM2-> CNT;
}
void speedBackward(){
	omega_M1 = (2.0/(T+2.0*taud))*(theta_M1-theta_M1_1)-((T-2.0*taud)/(T+2.0*taud))*omega_M1;
	omega_M2 = (2.0/(T+2.0*taud))*(theta_M2-theta_M2_1)-((T-2.0*taud)/(T+2.0*taud))*omega_M2;

	omegaScale_M2 = omega_M2/71;
	omegaScale_M1 = omega_M1/72;//kp_pos=0.5,ki_pos=0.003,Kd_pos=0.00001
}
float pid_control(float ref, float fb, float kp, float ki,float kd,float ofset, float *integralptr,float *prev_e_ptr,float output ){
	    float error = ref-fb;
	    e_m1=error;
	    *integralptr +=  + error * T;
	    float derivative = (error - (*prev_e_ptr)) / T;
	    *prev_e_ptr = error; // Save current error for the next 5ms cycle
	    output = kp * error + ki * (*integralptr) + (kd * derivative);
	    // Limit reference to safe value
	    if (output >  100.0f) { output =  100.0f; *integralptr -= error * T; }
	    if (output < -100.0f) { output = -100.0f; *integralptr -= error * T; }
	    if (fabs (error)<ofset){
	    	output=0;
	    }
	    return output;
}
void set_motor_angle(float angle) {
    target_angle = angle;
    pulse_2 = angle * pulses_per_degree;
    actual_angle=theta_M2/pulses_per_degree;
}
void UART_SendState(uint8_t state) {
    // Only send if the UART is not currently busy transmitting
    if (huart1.gState == HAL_UART_STATE_READY) {
        HAL_UART_Transmit_IT(&huart1, &state, 1);
    }
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART1)
//    {
//        /* * Logic: Use the received byte (rx_data) to set states.
//         * You can send '0', '1', or '2' from your computer/controller.
//         */
//        switch(rx_data)
//        {
//        	case 0: // Mode IDLE
//        		current_mode = MODE_IDLE;
//        		pulse = 0;
//        		set_motor_angle(0);
//            break;
//
//            case 1: // Mode IDLE
//                current_mode = MODE_IDLE;
//                pulse = -1000;
//                set_motor_angle(0);
//                break;
//
//            case 2: // Mode LIFT
//                current_mode = MODE_LIFT;
//                pulse = 50;        // Example value for M1
//                set_motor_angle(-90);
//                break;
//          case 3: // Mode LIFT
//                current_mode = MODE_LIFT;
//                pulse = 50;        // Example value for M1
//                set_motor_angle(-70); // Example value for M2
//                break;
//            case 4: // Mode SPIN
//                current_mode = MODE_SPIN;
//                pulse = 50;       // Example value for M1
//                set_motor_angle(180);// Example value for M2
//                break;
//            case 5: // Mode SPIN
//                current_mode = MODE_SPIN;
//                pulse = -1000;       // Example value for M1
//                set_motor_angle(180);// Example value for M2
//                break;
//            default:
//                // Handle other numbers as raw values if needed
//                break;
//        }
//        UART_SendState(rx_data);
//
//        /* CRITICAL: Restart the interrupt to receive the next byte */
//        count++ ;
//        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
//    }
//}
void set_servo_angle(float s_angle) {
    if (s_angle < 0) s_angle = 0;
    if (s_angle > 270) s_angle = 270;

    // Pulse range for DS3218: 500 (0 deg) to 2500 (270 deg)
    uint32_t pulse_value = 500 + (uint32_t)((s_angle / 270.0f) * 2000.0f);

    // Ensure this uses the calculated pulse_value
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pulse_value);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Instance == TIM3) {
     encoder();
     speedBackward();
     theta_M1_1 = theta_M1;
     theta_M2_1 = theta_M2;

     // Update Motor 2 target based on current_mode angle
     set_motor_angle(target_angle);//0 -90(down for grabing)

     // Motor 1 Position/Speed Control
//     yd_M1 = pid_control(pulse, theta_M1, kp_pos, ki_pos, kd_pos, 1.00, &integral_pos, &prev_e_pos, yd_M1);
//     if (yd_M1 > 90)  yd_M1 = 90;
//     if (yd_M1 < -90) yd_M1 = -90;
//     volt_out_M1 = pid_control(yd_M1, omegaScale_M1, kp_M1, ki_M1, 0, 0.00001, &integral_spd, &prev_e_spd, volt_out_M1);

     // Motor 2 Position/Speed Control (FIXED: Uses yd_M2 and Motor 2 pointers)
     yd_M2 = pid_control(pulse_2, theta_M2, kp_pos_2, ki_pos_2, kd_pos_2, 1.00, &integral_pos_2, &prev_e_pos_2, yd_M2);
     if (yd_M2 > 50)  yd_M2 = 50;
     if (yd_M2 < -50) yd_M2 = -50;
     volt_out_M2 = pid_control(yd_M2, omegaScale_M2, kp_M2, ki_M2, 0, 0.00001, &integral_spd_2, &prev_e_spd_2, volt_out_M2);

//     voltage_M1(volt_out_M1);
 	 set_servo_angle(s_angle);//0-90(close)
     voltage_M2(volt_out_M2);
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//      HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
//      HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
//      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11,1);
//      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,1);
//      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//      __HAL_TIM_MOE_ENABLE(&htim1);
//      HAL_UART_Receive_IT(&huart1, &rx_data, 1);
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//      HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
//      HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
//      HAL_TIM_Base_Start_IT(&htim3);
//      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11,1);
//      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,1);
//      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//      __HAL_TIM_MOE_ENABLE(&htim1);
//            HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
      HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
      HAL_TIM_Base_Start_IT(&htim3);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11,1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,1);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
      __HAL_TIM_MOE_ENABLE(&htim1);
      HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  target_angle=-90;
//	  HAL_Delay(4000);
//	  target_angle=-70;
//	  HAL_Delay(2000);
//	  target_angle=190;
//	  HAL_Delay(3000);
//	  target_angle=0;
//	  HAL_Delay(3000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 20000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE9 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
