/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
    
#include "stm32h7xx_hal.h"
#include "stm32h750b_discovery.h"
#include "stm32h750b_discovery_ts.h"
#include "stm32h750b_discovery_lcd.h"
#include "stm32h750b_discovery_mmc.h"
#include "stm32h750b_discovery_sdram.h"
#include "stm32_lcd.h"


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
    STATE_F1,
    STATE_F2,
    STATE_F3,
    STATE_F4,
} AppState;

typedef struct {
  AppState currentState;

  // F1 - screen where user enters voltage and can start/stop PWM
  uint16_t voltage; // register holding voltage that has been validated and is ready to be sent to PWM
  uint16_t inputValue; // register holding current value of "input field"
  bool isVoltageEntered; // flag if we are ready to start PWM. Maybe redundant, we can check against voltage register. But its more robust this way
  bool isPwmRunning;
  bool displayCursor;
  char message[64]; // ad hoc message to display

  // F2 - screen where user sets three calibration points - TODO later
  uint16_t calibrationPoints[3];
  uint8_t calibrationIndex; // which calibration point are we entering right now?

  uint16_t animationIndex;
} AppContext;

typedef void (*CallbackWithContext)(AppContext *ctx);
typedef void (*CallbackWithParam)(uint16_t);
typedef void (*CallbackFunction)(void);

typedef enum { // A bit atypical, but I want to be able to read data from PC connected through UART for better UX
  KEY_0 = '0',
  KEY_1 = '1',
  KEY_2 = '2',
  KEY_3 = '3',
  KEY_4 = '4',
  KEY_5 = '5',
  KEY_6 = '6',
  KEY_7 = '7',
  KEY_8 = '8',
  KEY_9 = '9',
  KEY_Enter = '\r', // enter on keyboard
  KEY_Clear = 'c',
  KEY_BkSp = 127, // backspace on keyboard
  KEY_Start = 'S',
  KEY_Stop = 's',
  KEY_ESC = 27, // esc on keyboard
  KEY_F1 = '!',
  KEY_F2 = '@',
  KEY_F3 = '#',
  KEY_F4 = '$',
  KEY_F5 = '%',
  KEY_Dot = '.',
  KEY_Lock = 'l',
  KEY_OFF = 'f',
  KEY_ON = 'n',
  KEY_NULL = 'N' // HACK - no key pressed
} KeyboardButton;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

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
