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
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_1        1
#define SERVO_2        2
#define SERVO_3        3
#define SERVO_4        4
#define SERVO_5        5
#define UART_TIMEOUT   100

// TCA9548A I2C Multiplexer address
#define TCA9548A_ADDR  0x70

// Magnetic Encoder (AS5600) address
#define ENCODER_ADDR   0x36

// Number of servos
#define NUM_SERVOS     5

// Phase shift between servos (pi/4 radians)
#define PHASE_SHIFT    0.785398163f  // pi/4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t servo_packet[10];
uint16_t encoder_values[NUM_SERVOS];
char uart_buffer[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void lx16a_send_packet(uint8_t id, int cmd, uint8_t len, uint16_t angle, uint16_t time);
void tca9548a_select_channel(uint8_t channel);
uint16_t read_encoder(uint8_t channel);
void send_telemetry(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void lx16a_send_packet(uint8_t id, int cmd, uint8_t len, uint16_t angle, uint16_t time) {
    // Packet format: 0x55 0x55 [ID] [LEN=7] CMD [PARAM_1] ... [PARAM_N]  [CHECKSUM]
    servo_packet[0] = 0x55;
    servo_packet[1] = 0x55;
    servo_packet[2] = id;
    servo_packet[3] = len;
    servo_packet[4] = angle & 0xFF;
    servo_packet[5] = (angle >> 8) & 0xFF;
    servo_packet[6] = time & 0xFF;
    servo_packet[7] = (time >> 8) & 0xFF;
    servo_packet[8] = 0x00;

    // Calculate checksum (XOR of bytes 2-8)
    uint8_t checksum = 0;
    for (int i = 2; i < 9; i++) {
        checksum ^= servo_packet[i];
    }
    servo_packet[9] = checksum;

    // Send over UART (blocking)
    HAL_UART_Transmit(&huart1, servo_packet, 9, UART_TIMEOUT);
}

/**
  * @brief Select a channel on the TCA9548A I2C multiplexer.
  * @param channel Channel number (0-7)
  */
void tca9548a_select_channel(uint8_t channel) {
    uint8_t cmd = 1 << channel;
    HAL_I2C_Master_Transmit(&hi2c1, TCA9548A_ADDR, &cmd, 1, 10);
}

/**
  * @brief Read the angle from a magnetic encoder on a specific multiplexer channel.
  * @param channel Channel number (0-4 for servos 1-5)
  * @return 12-bit angle value (0-4095)
  */
uint16_t read_encoder(uint8_t channel) {
    uint8_t raw[2];
    uint16_t angle = 0;

    // Select the multiplexer channel
    tca9548a_select_channel(channel);

    // Read 2 bytes from the encoder (AS5600 raw angle register)
    // AS5600: register 0x0C (high byte) and 0x0D (low byte)
    uint8_t reg = 0x0C;
    HAL_I2C_Master_Transmit(&hi2c1, ENCODER_ADDR << 8, &reg, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, ENCODER_ADDR << 8, raw, 2, 10);

    angle = ((uint16_t)raw[0] << 8) | raw[1];
    return angle;
}

/**
  * @brief Send telemetry data over UART (for debugging)
  */
void send_telemetry(void) {
    int len = sprintf(uart_buffer, "E: %4d %4d %4d %4d %4d\r\n",
                      encoder_values[0], encoder_values[1],
                      encoder_values[2], encoder_values[3],
                      encoder_values[4]);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len, 100);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
 // Initialize variables
  uint16_t angles[NUM_SERVOS];
  float time = 0.0f;
  int servo_index;

  // Initialize encoder values array
  for (int i = 0; i < NUM_SERVOS; i++) {
      encoder_values[i] = 0;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    // Calculate and move each servo with phase shift
    for (servo_index = 0; servo_index < NUM_SERVOS; servo_index++) {
        // Calculate angle: center 500, amplitude 400, phase shift per servo
        float phase = (float)servo_index * PHASE_SHIFT;
        angles[servo_index] = 500 + (uint16_t)(400 * sin(time + phase));

        // Send position to servo (ID = servo_index + 1)
        lx16a_send_packet(servo_index + 1, LX16A_SERVO_MOVE_TIME_WRITE, 7, angles[servo_index], 20);

        // Read encoder value from corresponding multiplexer channel
        encoder_values[servo_index] = read_encoder(servo_index);
    }

    // Send telemetry data over UART
    send_telemetry();

    // Delay 10ms (100Hz)
    HAL_Delay(10);
    time += 0.1f;

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  char buffer[50];
  sprintf(buffer, "I2C Initialized\n");
  printf("%s", buffer);
  /* USER CODE END I2C1_Init 2 */

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
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  char buffer[50];
  sprintf(buffer, "UART1 Initialized\n");
  printf("%s", buffer);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  char buffer[50];
  sprintf(buffer, "UART2 Initialized\n");
  printf("%s", buffer);
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  char buffer[50];
  sprintf(buffer, "GPIO Initialized\n");
  printf("%s", buffer);
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
