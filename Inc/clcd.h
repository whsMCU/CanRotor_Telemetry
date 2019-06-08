#pragma once

#include "Board.h"

#define RS_GPIO   GPIOB
#define RS_PIN    GPIO_PIN_10 // PB10 (RS)
#define RS_L      HAL_GPIO_WritePin(RS_GPIO, RS_PIN, GPIO_PIN_RESET);
#define RS_H      HAL_GPIO_WritePin(RS_GPIO, RS_PIN, GPIO_PIN_SET);

//#define RS_L GPIOB->BSRR = 0x04000000
//#define RS_H  GPIOB->BSRR = 0x04000400

#define RW_GPIO   GPIOB
#define RW_PIN    GPIO_PIN_0 // PB0 (RW)
#define RW_L      HAL_GPIO_WritePin(RW_GPIO, RW_PIN, GPIO_PIN_RESET);
#define RW_H      HAL_GPIO_WritePin(RW_GPIO, RW_PIN, GPIO_PIN_SET);

//#define RW_L GPIOB->BSRR = 0x00010000
//#define RW_H  GPIOB->BSRR = 0x00010001

#define EN_GPIO   GPIOB
#define EN_PIN    GPIO_PIN_11 // PB11 (EN)
#define EN_L      HAL_GPIO_WritePin(EN_GPIO, EN_PIN, GPIO_PIN_RESET);
#define EN_H      HAL_GPIO_WritePin(EN_GPIO, EN_PIN, GPIO_PIN_SET);

//#define EN_L GPIOB->BSRR = 0x08000000
//#define EN_H  GPIOB->BSRR = 0x08000800


void EN_PULSE(void);
void LCD_DATA(char data);
void LCD_print(uint8_t data);
void LCD_print_SC(uint8_t data);
uint8_t LCD_readCMD(void);
void LCD_CMD(char cmd);
void Set_Cursor(void);
void Clear_Cursor(void);
void LCD_INIT(void);
void LCD_XY(char x, char y);
void LCD_CLEAR(void);
void LCD_PUTS(char *str);
void output_V(void);
void output_ALT(void);
void output_SAT(void);
void output_TIME(void);
void output_HEAD(void);
void output_TEMP(void);
void output_ANGLE(float angle);
void output_CYCLETIME(void);
void output_PID(float value);
void LCDprintInt16(int16_t v);
void lcdprint_uint32(uint32_t v);
