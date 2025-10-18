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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- Các thông số của Encoder và Bánh xe ---
#define PULSES_PER_REVOLUTION   20.0f   // Số xung trên một vòng quay của đĩa encoder
#define WHEEL_DIAMETER_M        0.065f  // Đường kính bánh xe (mét), ví dụ: 6.5cm
#define PI                      3.14159f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
volatile float giatrido_hientai, error = 0, error_truoc = 0;
volatile static float integral = 0, derivative = 0;
volatile float setpoint = 400, Kp = 10, Ki = 4, Kd = 1;
volatile float Delta_t = 0.01;
volatile float output;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define GPIO_NN_Thuan GPIOB
#define GPIO_PIN_NN_Thuan GPIO_PIN_13
#define GPIO_NN_Nghich GPIOB
#define GPIO_PIN_NN_Nghich GPIO_PIN_14
#define GPIO_NN_Dung GPIOB
#define GPIO_PIN_NN_Dung GPIO_PIN_15
#define GPIO_IN1 GPIOB
#define GPIO_PIN_IN1 GPIO_PIN_3
#define GPIO_IN2 GPIOB
#define GPIO_PIN_IN2 GPIO_PIN_4
#define GPIO_ENA GPIOB
#define GPIO_PIN_ENA GPIO_PIN_5

int TrangThai = 0;

void Xuat_PWM(TIM_HandleTypeDef *htim, uint32_t Channel, float Duty_Cycle) {
Duty_Cycle = Duty_Cycle / 100 * htim->Instance->ARR;
__HAL_TIM_SET_COMPARE(htim, Channel, (uint16_t)Duty_Cycle);
}

void QuayThuan() {
HAL_GPIO_WritePin(GPIO_IN1, GPIO_PIN_IN1, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIO_IN2, GPIO_PIN_IN2, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIO_ENA, GPIO_PIN_ENA, GPIO_PIN_SET);
}
void QuayNghich() {
HAL_GPIO_WritePin(GPIO_IN1, GPIO_PIN_IN1, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIO_IN2, GPIO_PIN_IN2, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIO_ENA, GPIO_PIN_ENA, GPIO_PIN_SET);
}
void Dung() {
HAL_GPIO_WritePin(GPIO_IN1, GPIO_PIN_IN1, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIO_IN2, GPIO_PIN_IN2, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIO_ENA, GPIO_PIN_ENA, GPIO_PIN_RESET);
}
void KiemTra_NN() {
if (HAL_GPIO_ReadPin(GPIO_NN_Thuan, GPIO_PIN_NN_Thuan) == 0) {
TrangThai = 1;
}
if (HAL_GPIO_ReadPin(GPIO_NN_Nghich, GPIO_PIN_NN_Nghich) == 0) {
TrangThai = 2;
}
if (HAL_GPIO_ReadPin(GPIO_NN_Dung, GPIO_PIN_NN_Dung) == 0) {
TrangThai = 0;
}
}

volatile long xung;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // 1. Kiểm tra xem có đúng là ngắt từ PB9 không
  if(GPIO_Pin == GPIO_PIN_9)
  {
      // 2. KIỂM TRA CHIỀU QUAY MÀ PID ĐANG RA LỆNH
      if (output > 0) {
          xung++; // Nếu PID đang lệnh quay thuận, đếm lên
      } else if (output < 0) {
          xung--; // Nếu PID đang lệnh quay nghịch, đếm xuống
      }
    }
  }


void DieuKhienViTri() {
if (output > 0) {
QuayThuan();
Xuat_PWM(&htim4, TIM_CHANNEL_1, output);
}
if (output < 0) {
QuayNghich();
Xuat_PWM(&htim4, TIM_CHANNEL_1, -output);
}
if (output == 0) {
Dung();
Xuat_PWM(&htim4, TIM_CHANNEL_1, output);
}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
if (htim == &htim2) {
giatrido_hientai = xung;
error = setpoint - giatrido_hientai;
integral = integral + (error * Delta_t);
if (integral > 100) integral = 100;
else if (integral < -100) integral = -100;
derivative = (error - error_truoc) / Delta_t;
output = (Kp * error) + (Ki * integral) + (Kd * derivative);
if (output > 100)
output = 100;
if (output < -100)
output = -100;
error_truoc = error;
DieuKhienViTri();
}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1
	// Các biến để tính toán tốc độ
	  uint32_t last_calc_time = 0;
	  int rpm = 0;
	  float speed_mps = 0.0f; // Tốc độ mét/giây
	  char usb_buffer[100];
	  const float wheel_circumference_m = WHEEL_DIAMETER_M * PI;
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
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  // Hàm set giá trị so sánh của PWM 75%;

  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
	  char str[200];
	  sprintf(str, "vi=%.2f D=%.2f er=%.2f in=%.2f \r\n",
	  giatrido_hientai,
	  output, error, integral);
	  CDC_Transmit_FS((uint8_t*) str, strlen(str));
	  HAL_Delay(50);

      /* 1. Luôn kiểm tra và điều khiển động cơ
      KiemTra_NN();
      if (TrangThai == 0) { Dung(); }
      if (TrangThai == 1) { QuayThuan(); }
      if (TrangThai == 2) { QuayNghich(); }

      /* 2. Tính toán và gửi dữ liệu tốc độ mỗi 1 giây
      uint32_t current_time = HAL_GetTick();
      if (current_time - last_calc_time >= 1000)
      {
          last_calc_time = current_time;

          // Tạm thời vô hiệu hóa ngắt để đọc và reset biến 'xung' một cách an toàn
          __disable_irq();
          long current_pulses = xung;
          xung = 0;
          __enable_irq();

          // Tính toán
          // (xung/giây) / (xung/vòng) = vòng/giây
          float rps = (float)current_pulses / PULSES_PER_REVOLUTION; // Revolutions Per Second
          rpm = rps * 60; // Revolutions Per Minute

          // (vòng/giây) * (mét/vòng) = mét/giây
          speed_mps = rps * wheel_circumference_m;

          // Chuẩn bị chuỗi dữ liệu để gửi đi
          sprintf(usb_buffer, "RPM: %d | Speed: %.2f m/s\r\n", rpm, speed_mps);
          CDC_Transmit_FS((uint8_t*)usb_buffer, strlen(usb_buffer));
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
