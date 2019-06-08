#include "Board.h"

char Buf[64];

volatile unsigned char command=0;
volatile unsigned char m = 0;
int MSP_TRIM[3]={0, 0, 0};
int i_COM = 0;

imu_t imu;
rc RC;
rc RC_Raw;
st_t state;
gps_t GPS;
ms5611_t ms5611;
flags_t f;
int16_t motor[4];
pid_t PID;
float VBAT=0, Temperature=0;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern UART_HandleTypeDef huart2;

extern Queue_t Q_buffer[UART_MAX_CH];
////////////////////////////////////////////
volatile uint8_t rx_buffer[16];
//////////// MSP //////////////////
#define INBUF_SIZE 128
typedef  struct mspPortState_t {
//    serialPort_t *port;
    uint8_t checksum;
    uint8_t indRX;
    uint8_t inBuf[INBUF_SIZE];
    uint8_t cmdMSP;
    uint8_t offset;
    uint8_t dataSize;
    serialState_t c_state;
	
} mspPortState_t;

static mspPortState_t ports[UART_MAX_CH];
static mspPortState_t *currentPortState = &ports[0];

///////////////////////////////////////////////////////

int fputc(int ch, FILE *f) // for printf
{
   uint8_t tmp[1]={ch};
   HAL_UART_Transmit(&huart2, tmp, 1, 1);
	 //HAL_UART_Transmit_DMA(&huart1, tmp, 1);
   return(ch);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1) //current USART
		{
			write_Q(&Q_buffer[UART1], rx_buffer[0]);
			HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buffer, 1);
		}
}

void TX_CHR(char ch){
	while(!(USART1->SR & 0x80));
	USART1->DR = ch;
}

void clearbuffer1(void){			// Reset index vector Get char USART
	int i = 0;
	m = 0;
	for (i = 0; i < 20; i++) rx_buffer[i] = 0;
	i_COM = 0;
}

void clearbuffer2(void){			// Reset index vector Get char USART and Reset command
	int i = 0;
	m = 0;
	for (i = 0; i < 20; i++) rx_buffer[i] = 0;
	command = 0;
	i_COM = 0;
}
///////////////////////////////////////////////////
void serialize8(uint8_t a)
{
    TX_CHR(a);
    currentPortState->checksum ^= a;
}

void serialize32(uint32_t a)
{
    serialize8(a & 0xFF);
    serialize8((a >> 8) & 0xFF);
    serialize8((a >> 16) & 0xFF);
    serialize8((a >> 24) & 0xFF);
}

void serialize16(int16_t a)
{
    serialize8(a & 0xFF);
    serialize8((a >> 8) & 0xFF);
}

uint8_t read8(void)
{
    return currentPortState->inBuf[currentPortState->indRX++] & 0xff;
}

uint16_t read16(void)
{
    uint16_t t = read8();
    t += (uint16_t)read8() << 8;
    return t;
}

uint32_t read32(void)
{
    uint32_t t = read16();
    t += (uint32_t)read16() << 16;
    return t;
}

void headSerialResponse(uint8_t err, uint8_t s)
{
    serialize8('$');
    serialize8('M');
    serialize8(err ? '!' : '<');
    currentPortState->checksum = 0;               // start calculating a new checksum
    serialize8(s);
    serialize8(currentPortState->cmdMSP);
}

void headSerialReply(uint8_t s)
{
    headSerialResponse(0, s);
}

void headSerialError(uint8_t s)
{
    headSerialResponse(1, s);
}

void tailSerialReply(void)
{
    serialize8(currentPortState->checksum);
}

void headSerial(uint8_t err, uint8_t s, uint8_t cmdMSP)
{
    serialize8('$');
    serialize8('M');
    serialize8(err ? '!' : '<');
    currentPortState->checksum = 0;               // start calculating a new checksum
    serialize8(s);
    serialize8(cmdMSP);
}

void IMU_request(void)
{
	headSerial(0, 0, MSP_RAW_IMU);
	tailSerialReply();
}

void RADIO_request(void)
{
	headSerial(0, 0, MSP_RC);
	tailSerialReply();
}

void STATUS_request(void)
{
	headSerial(0, 0, MSP_STATUS);
	tailSerialReply();
}

void PID_request(void)
{
	headSerial(0, 0, MSP_PID);
	tailSerialReply();
}

void SET_PID(void)
{
	headSerial(0, 36, MSP_SET_PID);
	for (int i = 0; i < 3; i++){
   serialize32(PID.kp[i]*10);
	 serialize32(PID.ki[i]*10);
	 serialize32(PID.kd[i]*10);
	}
	tailSerialReply();
}
///////////////////////////////////////////////////

void SerialCom(void) {
	volatile uint8_t c;	
	for(int i = 0; i < 2; i++){
    currentPortState = &ports[i];
//		printf("Q_a : %2.1d, %2.1d \r\n",Q_buffer[0].head, Q_buffer[0].tail);
    while(QueueAvailable(&Q_buffer[i]) > 0){

		  c = read_Q(&Q_buffer[i]);
    if (currentPortState->c_state == IDLE) {
      currentPortState->c_state = (c=='$') ? HEADER_START : IDLE;

    }
    else if (currentPortState->c_state == HEADER_START) {
      currentPortState->c_state = (c=='M') ? HEADER_M : IDLE;
			
    }
    else if (currentPortState->c_state == HEADER_M) {
      currentPortState->c_state = (c=='>') ? HEADER_ARROW : IDLE;

    }
    else if (currentPortState->c_state == HEADER_ARROW) {

      if (c > INBUF_SIZE) {  // now we are expecting the payload size
        currentPortState->c_state = IDLE;
        continue;
      }
        currentPortState->dataSize = c;
        currentPortState->offset = 0;
        currentPortState->checksum = 0;
				currentPortState->indRX = 0;
        currentPortState->checksum ^= c;
        currentPortState->c_state = HEADER_SIZE;
    }
    else if (currentPortState->c_state == HEADER_SIZE) {
      currentPortState->cmdMSP = c;
      currentPortState->checksum ^= c;
      currentPortState->c_state = HEADER_CMD;
    }
    else if (currentPortState->c_state == HEADER_CMD && currentPortState->offset < currentPortState->dataSize) {
      currentPortState->checksum ^= c;
      currentPortState->inBuf[currentPortState->offset++] = c;
    }
    else if (currentPortState->c_state == HEADER_CMD && currentPortState->offset >= currentPortState->dataSize) {

      if (currentPortState->checksum == c) {
				evaluateCommand();
      }

      currentPortState->c_state = IDLE;
    }
   }
  }
 }

 void evaluateCommand(void) {
	 uint32_t i=0;
	 switch(currentPortState->cmdMSP){
		 case TELEMERY_ERROR:
					 state.error = read8();
				break;
		 
		 case TELEMERY_ARMED_MODE:
					 f.ARMED = read8();
			  break;
		 
		 case TELEMERY_HEADFREE_MODE:
					 f.HEADFREE_MODE = read8();
			  break;
		 
		 case TELEMERY_CYCLE_TIME:
					 state.cycleTime = read16();
			  break;

		 case TELEMERY_BAT_VOLT:
					 VBAT = read32();
			  break;

		 case TELEMERY_TEMPERATURE:
					 Temperature = read32();
				   Temperature /=10;
			  break;

		 case TELEMERY_ANGLE_ROLL:
					 imu.angle[ROLL] = (float)read32()-400;
			  break;

		 case TELEMERY_ANGLE_PITCH:
					 imu.angle[PITCH] = (float)read32()-400;
			  break;

		 case TELEMERY_ANGLE_YAW:
					 imu.angle[YAW] = (float)read32()-400;
		 			sprintf(Buf, "%f\r\n ", imu.angle[YAW]);
    		HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf),1000);
			  break;
		 
		 case TELEMERY_HEADING:
					 imu.heading = (float)read32();
			  break;

		 case TELEMERY_ARMD_TIME:
					 state.armedTime = read32();
			  break;			 
		 
		 case TELEMERY_BARO_EST:
					 ms5611.EstAlt = (int32_t)read32();
			  break;	

		 case TELEMERY_PID_RP_P:
					 PID.kp[0] = (float)read32();
			  break;	

		 case TELEMERY_PID_RP_I:
					 PID.ki[0] = (float)read32();
			  break;	

		 case TELEMERY_PID_RP_D:
					 PID.kd[0] = (float)read32();
			  break;	

		 case TELEMERY_PID_Y_P:
					 PID.kp[2] = (float)read32();
			  break;	

		 case TELEMERY_PID_Y_I:
					 PID.ki[2] = (float)read32();
			  break;	

		 case TELEMERY_PID_Y_D:
					 PID.kd[2] = (float)read32();
			  break;	

		 case TELEMERY_NUM_SATS:
					 GPS.satellites = read8();
			  break;	

		 case TELEMERY_FIX_TYPE:
					 GPS.fixquality = read8();
			  break;	

		 case TELEMERY_GPS_LAT:
					 GPS.latitudeDegrees = (float)read32();
			  break;

		 case TELEMERY_GPS_LON:
					 GPS.longitudeDegrees = (float)read32();
			  break;		 
		 
		 
		 
		 case MSP_ARM:
//			 mwArm();
				sprintf(Buf, "LOCK : %d, %d, %d, %d, %d\r\n ", currentPortState->inBuf[0], currentPortState->inBuf[1], currentPortState->inBuf[2], currentPortState->inBuf[3], currentPortState->inBuf[4]);
    		HAL_UART_Transmit_IT(&huart1, (uint8_t*)Buf, strlen(Buf));
			 break;
		 
		 case MSP_DISARM:
//			 mwDisarm();
				sprintf(Buf, "UNLOCK : %d, %d, %d, %d, %d\r\n ", currentPortState->inBuf[0], currentPortState->inBuf[1], currentPortState->inBuf[2], currentPortState->inBuf[3], currentPortState->inBuf[4]);
    		HAL_UART_Transmit_IT(&huart1, (uint8_t*)Buf, strlen(Buf));
			 break;
		 
		 case MSP_RC:
				for(i=0; i < 5; i++){
					RC.rcCommand[i] = read16();
				}
			 break;
				
			case MSP_RAW_IMU:
				for(i=0; i < 3; i++){
					imu.angle[i] = (float)read32();
					imu.angle[i] = imu.angle[i]-400;
				}
			 break;		
				
	    case MSP_STATUS:
	      state.cycleTime    = read16();
	      state.mode         = read8();
	      state.error        = read8();
	      state.errors_count = read8();
	      break;
					
		 case MSP_PID:
			 for(i=0; i < 3; i++){
				 PID.kp[i] = read32();
				 PID.kp[i]/=10;
				 PID.ki[i] = read32();
				 PID.ki[i]/=10;
				 PID.kd[i] = read32();
				 PID.kd[i]/=10;
				}

       break;				
				
		 	case MSP_TRIM_UP:
				MSP_TRIM[PITCH] += 1;
				sprintf(Buf, "MSP_TRIM_UP : %d, %d, %d, %d, %d\r\n ", currentPortState->inBuf[0], currentPortState->inBuf[1], currentPortState->inBuf[2], currentPortState->inBuf[3], currentPortState->inBuf[4]);
    		HAL_UART_Transmit_IT(&huart1, (uint8_t*)Buf, strlen(Buf));
			 break;
				 
			case MSP_TRIM_DOWN:
				MSP_TRIM[PITCH] -= 1;
				sprintf(Buf, "MSP_TRIM_DOWN : %d, %d, %d, %d, %d\r\n ", currentPortState->inBuf[0], currentPortState->inBuf[1], currentPortState->inBuf[2], currentPortState->inBuf[3], currentPortState->inBuf[4]);
    		HAL_UART_Transmit_IT(&huart1, (uint8_t*)Buf, strlen(Buf));
			 break;
						 
			case MSP_TRIM_LEFT:
				MSP_TRIM[ROLL] -= 1;
				sprintf(Buf, "MSP_TRIM_LEFT : %d, %d, %d, %d, %d\r\n ", currentPortState->inBuf[0], currentPortState->inBuf[1], currentPortState->inBuf[2], currentPortState->inBuf[3], currentPortState->inBuf[4]);
    		HAL_UART_Transmit_IT(&huart1, (uint8_t*)Buf, strlen(Buf));
			 break;
								 
			case MSP_TRIM_RIGHT:
				MSP_TRIM[ROLL] += 1;
				sprintf(Buf, "MSP_TRIM_RIGHT : %d, %d, %d, %d, %d\r\n ", currentPortState->inBuf[0], currentPortState->inBuf[1], currentPortState->inBuf[2], currentPortState->inBuf[3], currentPortState->inBuf[4]);
    		HAL_UART_Transmit_IT(&huart1, (uint8_t*)Buf, strlen(Buf));
			 break;
		 
		 case MSP_RAW_GPS:
//          GPSValues result = p.evalGPS(inBuf);
//          logger.logGPS(result);
			 break;
	 }
	 //tailSerialReply();
 }
