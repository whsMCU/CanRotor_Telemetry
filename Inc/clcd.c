#include "clcd.h"
 
uint32_t Data_pin[4] = {GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_13, GPIO_PIN_12};  // MSBFIRST
// RS-B10, EN-B11, D4-B11, D5-B13, D6-B14, D7-B15

char digit10000(uint16_t v) {return '0' + v / 10000;}
char digit1000(uint16_t v) {return '0' + v / 1000 - (v/10000) * 10;}
char digit100(uint16_t v) {return '0' + v / 100 - (v/1000) * 10;}
char digit10(uint16_t v) {return '0' + v / 10 - (v/100) * 10;}
char digit1(uint16_t v) {return '0' + v - (v/10) * 10;}

void EN_PULSE(void) {
	EN_H;
	delay_us(50);//250
	EN_L;
	delay_us(50);
}

void write_4bit(uint8_t data){

		GPIOB->BSRR = 0xF0000000|((data&0x0F)<<12);
		EN_PULSE();
}

uint8_t LCD_readCMD(void) {
	GPIOB->CRH = (GPIOB->CRH & 0x0000FFFF)|(0x44440000);
	RS_L;
  RW_H;
	EN_H;
	delay_us(1);
	uint16_t temp_H = (GPIOB->IDR & 0xF000)>>8;
	EN_L;
	delay_us(1);
	EN_H;
	delay_us(1);
	uint16_t temp_L = (GPIOB->IDR & 0xF000)>>12;
	EN_L;
	RW_L;
  uint8_t temp = temp_H|temp_L;
	
  GPIOB->CRH = (GPIOB->CRH & 0x0000FFFF)|(0x22220000);
	
	return temp;
}

void LCD_CMD(char cmd) {
  RS_L;
	//while(LCD_readCMD() == 0x80)
	write_4bit(cmd>>4);
	//while(LCD_readCMD() == 0x80)
	write_4bit(cmd);	 
}
 
void LCD_DATA(char data) {
  RS_H;
	//while(LCD_readCMD() == 0x80)
	write_4bit(data>>4);
	//while(LCD_readCMD() == 0x80)
	write_4bit(data);
}

void LCD_print(uint8_t data) {
	data += '0';
  RS_H;
	//while(LCD_readCMD() == 0x80)
	write_4bit(data>>4);
	//while(LCD_readCMD() == 0x80)
	write_4bit(data);
}

void LCD_print_SC(uint8_t data) {
  RS_H;
	//while(LCD_readCMD() == 0x80)
	write_4bit(data>>4);
	//while(LCD_readCMD() == 0x80)
	write_4bit(data);
}

void Set_Cursor(void) {
	LCD_CMD(0x0F);
}

void Clear_Cursor(void) {
	LCD_CMD(0x0C);
}
 
void LCD_INIT(void) {
  HAL_Delay(100);
  LCD_CMD(0x03); HAL_Delay(45);// 4 bits, 2 line, 5x8 font
	LCD_CMD(0x03); HAL_Delay(45);
	LCD_CMD(0x03); HAL_Delay(45);
	LCD_CMD(0x02); HAL_Delay(100);
	LCD_CMD(0x28);//4bit mode, 5x8Dot, 2Line
	LCD_CMD(0x0C);//Display on
	LCD_CMD(0x06);//font direction left
	LCD_CMD(0x01);//Display Clear
	HAL_Delay(100);
}
 
void LCD_XY(char x, char y) {
  if     (y==0) LCD_CMD(0x80 + x);
  else if(y==1) LCD_CMD(0xC0 + x);
  else if(y==2) LCD_CMD(0x94 + x);
  else if(y==3) LCD_CMD(0xD4 + x);
}
 
void LCD_CLEAR(void) { LCD_CMD(0x01); HAL_Delay(3); }
 
void LCD_PUTS(char *str) { while(*str) LCD_DATA(*str++); }

void output_V(void){
	static char line[14] = "--.-V";
  //                      0123
	line[0] = digit100(VBAT);
  line[1] = digit10(VBAT);
  line[3] = digit1(VBAT);
	LCD_PUTS(line);
}

void output_ALT(void){
	static char line[14] = "---m";
  //                     0123
	line[0] = digit100(ms5611.EstAlt);
  line[1] = digit10(ms5611.EstAlt);
  line[2] = digit1(ms5611.EstAlt);
	LCD_PUTS(line);
}

void output_SAT(void){
	static char line[14] = "0--";
  //                     012
  line[1] = digit10(GPS.satellites);
  line[2] = digit1(GPS.satellites);
	LCD_PUTS(line);
}

void output_TIME(void){
	static char line[14] = "--:--";
  //                     01234
  line[0] = digit10(minutes);
  line[1] = digit1(minutes);
	line[3] = digit10(seconds);
  line[4] = digit1(seconds);
	LCD_PUTS(line);
}

void output_HEAD(void){
	static char line[14] = "---";
  //                     012
	line[0] = digit100(imu.heading);
  line[1] = digit10(imu.heading);
  line[2] = digit1(imu.heading);
	LCD_PUTS(line);
}

void output_TEMP(void){
	static char line[14] = "--";
  //                      01
  line[0] = digit10(Temperature);
  line[1] = digit1(Temperature);
	LCD_PUTS(line);
}

void output_ANGLE(float angle){
	static char line[14] = "----";
  //                      0123
	if(angle < 0) {
		line[0] = '-';
	} else {
		line[0] = ' ';
	}
		
	line[1] = digit100(angle);
	line[2] = digit10(angle);
  line[3] = digit1(angle);
	LCD_PUTS(line);
}

void output_CYCLETIME(void){
	static char line[14] = "----us";
  //                      012345
  line[0] = digit1000(state.cycleTime);
  line[1] = digit100(state.cycleTime);
	line[2] = digit10(state.cycleTime);
  line[3] = digit1(state.cycleTime);
	LCD_PUTS(line);
}

void output_PID(float value){
	static char line[14] = "--.-";
  //                      0123
	if(value < 100){
		line[0] = ' ';
	}else {
	line[0] = digit100(value);
	}
  line[1] = digit10(value);
  line[3] = digit1(value);
	LCD_PUTS(line);
}

void LCDprintInt16(int16_t v) {
  uint16_t unit;
  char line[7]; // = "      ";
  if (v < 0 ) {
    unit = -v;
    line[0] = '-';
  } else {
    unit = v;
    line[0] = ' ';
  }
  line[1] = digit10000(unit);
  line[2] = digit1000(unit);
  line[3] = digit100(unit);
  line[4] = digit10(unit);
  line[5] = digit1(unit);
  line[6] = 0;
  LCD_PUTS(line);
}
void lcdprint_uint32(uint32_t v) {
  static char line[14] = "-.---.---.---";
  //                      0 2 4 6 8   12
  line[0]  = '0' + v  / 1000000000;
  line[2]  = '0' + v  / 100000000 - (v/1000000000) * 10;
  line[3]  = '0' + v  / 10000000  - (v/100000000)  * 10;
  line[4]  = '0' + v  / 1000000   - (v/10000000)   * 10;
  line[6]  = '0' + v  / 100000    - (v/1000000)    * 10;
  line[7]  = '0' + v  / 10000     - (v/100000)     * 10;
  line[8]  = '0' + v  / 1000      - (v/10000)      * 10;
  line[10]  = '0' + v  / 100       - (v/1000)       * 10;
  line[11] = '0' + v  / 10        - (v/100)        * 10;
  line[12] = '0' + v              - (v/10)         * 10;
  LCD_PUTS(line);
}
//--------------------------------------------------------------------
/*void setup() {
  for(int i=2; i<8; i++) pinMode(i, OUTPUT);
  LCD_INIT(); 
}
 
void loop() {
  LCD_XY(0, 0); LCD_PUTS((char *)"LCD Display test");
  LCD_XY(0, 1); LCD_PUTS((char *)"Hello World.....");
  LCD_XY(0, 2); LCD_PUTS((char *)"LCD Display test");
  LCD_XY(0, 3); LCD_PUTS((char *)"Hello Kitty.....");
  HAL_Delay(500);
  for(int i=0; i<20; i++){
    LCD_CMD(0x1C);
    delay(300);
  }
  delay(500);
  for(int i=0; i<20; i++){
    LCD_CMD(0x18);
    delay(300);
  }
  delay(500);
}*/

//  for (int i=0; i<4; i++) {
//    if(cmd & (0x08>>i)) HAL_GPIO_WritePin(GPIOB, Data_pin[i], GPIO_PIN_SET);
//    else HAL_GPIO_WritePin(GPIOB, Data_pin[i], GPIO_PIN_RESET);
//  }
