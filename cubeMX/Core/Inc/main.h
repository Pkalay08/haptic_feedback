/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define LX16A_BROADCAST_ID 0xFE
#define LX16A_SERVO_MOVE_TIME_WRITE 1
#define LX16A_SERVO_MOVE_TIME_READ 2
#define LX16A_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LX16A_SERVO_MOVE_TIME_WAIT_READ 8
#define LX16A_SERVO_MOVE_START 11
#define LX16A_SERVO_MOVE_STOP 12
#define LX16A_SERVO_ID_WRITE 13
#define LX16A_SERVO_ID_READ 14
#define LX16A_SERVO_ANGLE_OFFSET_ADJUST 17
#define LX16A_SERVO_ANGLE_OFFSET_WRITE 18
#define LX16A_SERVO_ANGLE_OFFSET_READ 19
#define LX16A_SERVO_ANGLE_LIMIT_WRITE 20
#define LX16A_SERVO_ANGLE_LIMIT_READ 21
#define LX16A_SERVO_VIN_LIMIT_WRITE 22
#define LX16A_SERVO_VIN_LIMIT_READ 23
#define LX16A_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LX16A_SERVO_TEMP_MAX_LIMIT_READ 25
#define LX16A_SERVO_TEMP_READ 26
#define LX16A_SERVO_VIN_READ 27
#define LX16A_SERVO_POS_READ 28
#define LX16A_SERVO_OR_MOTOR_MODE_WRITE 29
#define LX16A_SERVO_OR_MOTOR_MODE_READ 30
#define LX16A_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LX16A_SERVO_LOAD_OR_UNLOAD_READ 32
#define LX16A_SERVO_LED_CTRL_WRITE 33
#define LX16A_SERVO_LED_CTRL_READ 34
#define LX16A_SERVO_LED_ERROR_WRITE 35
#define LX16A_SERVO_LED_ERROR_READ 36
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
