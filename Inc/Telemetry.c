#include "Board.h"

uint8_t telemetry_loop_counter = 0;

void send_telemetry_data(void){
	telemetry_loop_counter++;
	if(telemetry_loop_counter == 1) STATUS_request(); //1580
	//if(telemetry_loop_counter == 2) RADIO_request();  //2400
	//if(telemetry_loop_counter == 3) IMU_request();  //2800
	//if(telemetry_loop_counter == 4) PID_request(); //6950
	if(telemetry_loop_counter ==5) telemetry_loop_counter = 0;
}
