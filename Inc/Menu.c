#include "Menu.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart2;

const int numOfInputs = 4;
uint16_t inputPins[numOfInputs] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7};
int inputState[numOfInputs];
int lastInputState[numOfInputs] = {0,0,0,0};
bool inputFlags[numOfInputs] = {0,0,0,0};
uint32_t lastDebounceTime[numOfInputs] = {0,0,0,0};
uint32_t debounceDelay = 5;
uint32_t pressDelay = 1000;
uint32_t press_count;
uint32_t pre_time_pressed;
bool press_state;
bool press_done;
int setting = 0;
	
const int numOfScreens = 2;
int currentScreen = 0;
int currentSelcet = 0;
const int numOfParams = 6;
const int numOfSelection = 2;
int currentParameter = 0;


void setInputFlags() {
  for(int i = 0; i < numOfInputs; i++) {
    int reading = HAL_GPIO_ReadPin(GPIOA, inputPins[i]);
    if (reading != lastInputState[i]) {
      lastDebounceTime[i] = millis();
    }
    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      if (reading != inputState[i]) {
        inputState[i] = reading;
        if (inputState[i] == true) {
          inputFlags[i] = true;
        }
      }
    }
    lastInputState[i] = reading;
  }
	int read_pin = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
  if(press_state != read_pin){
		if(press_state == false){
			pre_time_pressed = millis();
		}
		press_state = read_pin;
	}
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)){
		if((millis() - pre_time_pressed) >=1000 && currentScreen == 1){
			press_done = true;
			currentParameter = 0;
			setting = 0;
		}
	}
}

void resolveInputFlags() {
  for(int i = 0; i < numOfInputs; i++) {
    if(inputFlags[i] == true) {
      inputAction(i);
      inputFlags[i] = false;
    }
  }
}

void inputAction(int input) {
  if(input == 0) {
    if (currentScreen == numOfScreens-1) {
      currentScreen = 0;
    }else{
      currentScreen++;
    }
		setting = 0;
		currentParameter = 0;
  }else if(input == 1) {
		if (currentParameter == 0 && setting == 1 && currentScreen == 1 && press_done != 1) {
			currentParameter = numOfParams-1;
		}else if(setting == 1 && currentScreen == 1 && press_done != 1){
			currentParameter--;
		}
		
		if (currentSelcet == 0 && setting == 1 && press_done == 1) {
			currentSelcet = numOfSelection-1;
		}else if(setting == 1 && press_done == 1){
			currentSelcet--;
		}
		parameterChange(0);
		parameterChange(2);
  }else if(input == 2) {
		if (currentParameter == numOfParams-1 && setting == 1 && currentScreen == 1 && press_done != 1) {
			currentParameter = 0;
		}else if(setting == 1 && currentScreen == 1 && press_done != 1){
			currentParameter++;
		}
		
		if (currentSelcet == numOfSelection-1 && setting == 1 && press_done == 1) {
			currentSelcet = 0;
		}else if(setting == 1 && press_done == 1){
			currentSelcet++;
		}
    parameterChange(1);
		parameterChange(2);
  }else if(input == 3) {
					LED0_TOGGLE;
		if(setting == 2) {
			setting = 0;
		}else{
			setting++;
		}
  }
}

void parameterChange(int key) {
  if(key == 0 && currentParameter == 0 && setting == 2 && currentScreen == 1 && press_done != 1) {
    PID.kp[ROLL] += 10;
		headSerial(0, 4, TELEMERY_PIDSET_RP_P);
		serialize32(PID.kp[ROLL]);
		tailSerialReply();
  }else if(key == 1 && currentParameter == 0 && setting == 2 && currentScreen == 1 && press_done != 1) {
    PID.kp[ROLL] -= 10;
		headSerial(0, 4, TELEMERY_PIDSET_RP_P);
		serialize32(PID.kp[ROLL]);
		tailSerialReply();
  }
	
	if(key == 0 && currentParameter == 1 && setting == 2 && currentScreen == 1 && press_done != 1) {
    PID.ki[ROLL] += 10;
		headSerial(0, 4, TELEMERY_PIDSET_RP_I);
		serialize32(PID.ki[ROLL]);
		tailSerialReply();
  }else if(key == 1 && currentParameter == 1 && setting == 2 && currentScreen == 1 && press_done != 1) {
    PID.ki[ROLL] -= 10;
		headSerial(0, 4, TELEMERY_PIDSET_RP_I);
		serialize32(PID.ki[ROLL]);
		tailSerialReply();
  }
	
	if(key == 0 && currentParameter == 2 && setting == 2 && currentScreen == 1 && press_done != 1) {
    PID.kd[ROLL] += 10;
		headSerial(0, 4, TELEMERY_PIDSET_RP_D);
		serialize32(PID.kd[ROLL]);
		tailSerialReply();
  }else if(key == 1 && currentParameter == 2 && setting == 2 && currentScreen == 1 && press_done != 1) {
    PID.kd[ROLL] -= 10;
		headSerial(0, 4, TELEMERY_PIDSET_RP_D);
		serialize32(PID.kd[ROLL]);
		tailSerialReply();
  }
	
	if(key == 0 && currentParameter == 3 && setting == 2 && currentScreen == 1 && press_done != 1) {
    PID.kp[YAW] += 10;
		headSerial(0, 4, TELEMERY_PIDSET_Y_P);
		serialize32(PID.kp[YAW]);
		tailSerialReply();
  }else if(key == 1 && currentParameter == 3 && setting == 2 && currentScreen == 1 && press_done != 1) {
    PID.kp[YAW] -= 10;
		headSerial(0, 4, TELEMERY_PIDSET_Y_P);
		serialize32(PID.kp[YAW]);
		tailSerialReply();
  }
	
	if(key == 0 && currentParameter == 4 && setting == 2 && currentScreen == 1 && press_done != 1) {
    PID.ki[YAW] += 10;
		headSerial(0, 4, TELEMERY_PIDSET_Y_I);
		serialize32(PID.ki[YAW]);
		tailSerialReply();
  }else if(key == 1 && currentParameter == 4 && setting == 2 && currentScreen == 1 && press_done != 1) {
    PID.ki[YAW] -= 10;
		headSerial(0, 4, TELEMERY_PIDSET_Y_I);
		serialize32(PID.ki[YAW]);
		tailSerialReply();
  }
	
	if(key == 0 && currentParameter == 5 && setting == 2 && currentScreen == 1 && press_done != 1) {
    PID.kd[YAW] += 10;
		headSerial(0, 4, TELEMERY_PIDSET_Y_D);
		serialize32(PID.kd[YAW]);
		tailSerialReply();
  }else if(key == 1 && currentParameter == 5 && setting == 2 && currentScreen == 1 && press_done != 1) {
    PID.kd[YAW] -= 10;
		headSerial(0, 4, TELEMERY_PIDSET_Y_D);
		serialize32(PID.kd[YAW]);
		tailSerialReply();
  }
	
	if(key == 2 && currentSelcet == 0 && setting == 2 && currentScreen == 1 && press_done == 1) {
		headSerial(0, 0, TELEMERY_PID_SAVE);
		tailSerialReply();
		setting = 0;
		currentParameter = 0;
		currentSelcet = 0;
		press_done = false;
	}else if(key == 2 && currentSelcet == 1 && setting == 2 && currentScreen == 1 && press_done == 1) {
		setting = 0;
		currentParameter = 0;
		currentSelcet = 0;
		press_done = false;
	}
}

void printScreen() {
  LCD_CLEAR();
	if(currentScreen == 1 && press_done == 1){
		Setting_SAVE();
	}else {
		switch (currentScreen)
		{
			case 0:
				Disply_Status();
			break;
			
			case 1:
				Disply_PID();
			break;
			
		}
	}
}

void Disply_Status(void){
	LCD_XY(0, 0); LCD_PUTS("M");
	LCD_XY(1, 0); LCD_print(f.ARMED);
		
	LCD_XY(2, 0); LCD_PUTS("H");
	LCD_XY(3, 0); LCD_print(f.HEADFREE_MODE);
		
	LCD_XY(4, 0); LCD_PUTS("E");
	LCD_XY(5, 0); LCD_print(state.error);

	LCD_XY(8, 0); output_V();
		
	LCD_XY(15, 0);
	if(ms5611.EstAlt < 0) LCD_PUTS("-");
	else LCD_PUTS("+");
	LCD_XY(16, 0); output_ALT();
		
	LCD_XY(8, 1); output_TIME();
		
	LCD_XY(15, 1); LCD_PUTS("H");
	LCD_XY(16, 1); output_HEAD();
	LCD_XY(19, 1); LCD_print_SC(223);
		
	LCD_XY(8, 2); LCD_PUTS("T");
	LCD_XY(10, 2); output_TEMP();
	LCD_XY(12, 2); LCD_print_SC(223);
		
	LCD_XY(15, 2);
	if(GPS.fixquality == 3) LCD_PUTS("S");
	else LCD_PUTS("s");
	output_SAT();
		
	LCD_XY(0, 1); LCD_PUTS("R");
	LCD_XY(1, 1); output_ANGLE(imu.angle[ROLL]);
	LCD_XY(5, 1); LCD_print_SC(223);
	
	LCD_XY(0, 2); LCD_PUTS("P");
	LCD_XY(1, 2); output_ANGLE(imu.angle[PITCH]);
	LCD_XY(5, 2); LCD_print_SC(223);
		
	LCD_XY(0, 3); LCD_PUTS("Y");
	LCD_XY(1, 3); output_ANGLE(imu.angle[YAW]);
	LCD_XY(5, 3); LCD_print_SC(223);
		
	LCD_XY(8, 3); LCD_PUTS("CT");
	LCD_XY(10, 3); output_CYCLETIME();
}

void Disply_PID(void){
	LCD_XY(5, 0);  LCD_PUTS("P");
	LCD_XY(11, 0); LCD_PUTS("I");
	LCD_XY(17, 0); LCD_PUTS("D");
	
	LCD_XY(0, 1); LCD_PUTS("R");
	LCD_XY(0, 2); LCD_PUTS("P");
	LCD_XY(0, 3); LCD_PUTS("Y");
	
	LCD_XY(3, 1);	output_PID(PID.kp[ROLL]);
	LCD_XY(9, 1);	output_PID(PID.ki[ROLL]);
	LCD_XY(15, 1);output_PID(PID.kd[ROLL]);
	LCD_XY(3, 2);	output_PID(PID.kp[ROLL]);
	LCD_XY(9, 2);	output_PID(PID.ki[ROLL]);
	LCD_XY(15, 2);output_PID(PID.kd[ROLL]);
	LCD_XY(3, 3);	output_PID(PID.kp[YAW]);
	LCD_XY(9, 3);	output_PID(PID.ki[YAW]);
	LCD_XY(15, 3);output_PID(PID.kd[YAW]);
	
	if(currentParameter == 0 && setting == 1){
		LCD_XY(2, 1);  LCD_PUTS(">");
		LCD_XY(2, 2);  LCD_PUTS(">");
	}else if(currentParameter == 0 && setting == 2){
		LCD_XY(2, 1);  LCD_print_SC(126);;
		LCD_XY(2, 2);  LCD_print_SC(126);;
	}
	
	
	if(currentParameter == 1 && setting == 1){
		LCD_XY(8, 1);  LCD_PUTS(">");
		LCD_XY(8, 2);  LCD_PUTS(">");
	}else if(currentParameter == 1 && setting == 2){
		LCD_XY(8, 1);  LCD_print_SC(126);
		LCD_XY(8, 2);  LCD_print_SC(126);
	}
		
	if(currentParameter == 2 && setting == 1){
		LCD_XY(14, 1);  LCD_PUTS(">");
		LCD_XY(14, 2);  LCD_PUTS(">");
	}else if(currentParameter == 2 && setting == 2){
		LCD_XY(14, 1);  LCD_print_SC(126);
		LCD_XY(14, 2);  LCD_print_SC(126);
	}
		
	if(currentParameter == 3 && setting == 1){
		LCD_XY(2, 3);  LCD_PUTS(">");
	}else if(currentParameter == 3 && setting == 2){
		LCD_XY(2, 3);  LCD_print_SC(126);
	}
		
	if(currentParameter == 4 && setting == 1){
		LCD_XY(8, 3);  LCD_PUTS(">");
	}else if(currentParameter == 4 && setting == 2){
		LCD_XY(8, 3);  LCD_print_SC(126);
	}
		
	if(currentParameter == 5 && setting == 1){
		LCD_XY(14, 3);  LCD_PUTS(">");
	}else if(currentParameter == 5 && setting == 2){
		LCD_XY(14, 3);  LCD_print_SC(126);
	}
	LCD_XY(0, 0); LCD_print(currentParameter);
	LCD_XY(1, 0); LCD_print(setting);
	LCD_XY(2, 0); LCD_print(press_done);
}

void Setting_SAVE(void){
		LCD_XY(5, 0); LCD_PUTS("<Warning>");
		LCD_XY(0, 1); LCD_PUTS("Do You Want To Save");
		LCD_XY(0, 2); LCD_PUTS("The Current Param?");
		LCD_XY(5, 3); LCD_PUTS("YES!");
		LCD_XY(15, 3); LCD_PUTS("NO!");
	
	if(currentSelcet == 0 && setting == 1){
		LCD_XY(3, 3);  LCD_PUTS(">");
	}else if(currentSelcet == 0 && setting == 2){
		LCD_XY(3, 3);  LCD_print_SC(126);
	}
	
	if(currentSelcet == 1 && setting == 1){
		LCD_XY(13, 3);  LCD_PUTS(">");
	}else if(currentSelcet == 1 && setting == 2){
		LCD_XY(13, 3);  LCD_print_SC(126);
	}
	
	LCD_XY(0, 0); LCD_print(currentSelcet);
	LCD_XY(1, 0); LCD_print(setting);
	LCD_XY(2, 0); LCD_print(press_done);
	
}
