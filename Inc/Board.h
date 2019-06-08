#pragma once

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>
/////////////////////////////////////////////
#include "clcd.h"
#include "Typedef.h"
#include "System.h"
#include "Serial.h"
#include "Queue.h"
#include "Telemetry.h"
#include "Menu.h"

extern imu_t imu;
extern rc RC;
extern rc RC_Raw;
extern st_t state;
extern gps_t GPS;
extern ms5611_t ms5611;
extern flags_t f;
extern int16_t motor[4];
extern pid_t PID;
extern float VBAT, Temperature;
extern uint8_t hours,minutes, seconds;

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif /* M_PI */

#define D2R (3.141592653f / 180.0f)
#define R2D (180.0f / 3.141592653f)

#define TRUE 1
#define FALSE 0

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define map(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

//void resetConf(void);


//#ifdef

#define LED0_GPIO   GPIOC
#define LED0_PIN    GPIO_PIN_13 // PC13 (GREEN LED)
#define LED0_TOGGLE              HAL_GPIO_TogglePin(LED0_GPIO, LED0_PIN);
#define LED0_OFF                 HAL_GPIO_WritePin(LED0_GPIO, LED0_PIN, GPIO_PIN_RESET);
#define LED0_ON                  HAL_GPIO_WritePin(LED0_GPIO, LED0_PIN, GPIO_PIN_SET);
////////////////////////////////////////////

