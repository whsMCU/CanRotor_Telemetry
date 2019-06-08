#include <Board.h>

static volatile uint32_t millis_cnt = 0;

void HAL_SYSTICK_Callback(void)
{
	millis_cnt++;
}
uint32_t millis(void){
	return millis_cnt;
}
//uint32_t micros(void)
//{
//	 register uint32_t ms, cycle_cnt;
//	    do {
//	        ms = msTicks;
//	        cycle_cnt = SysTick->VAL;
//	    } while (ms != msTicks);
//	    return (ms * 1000) + (72 * 1000 - cycle_cnt) / 72; //168
//}

uint32_t micros(){
    uint32_t val = SysTick->VAL;
    uint32_t load = SysTick->LOAD;
    return (millis_cnt&0x3FFFFF)*1000 + (load-val)/((load+1)/1000);
}
 
void delay_us(uint32_t us){
    uint32_t temp = micros();
    uint32_t comp = temp + us;
    uint8_t  flag = 0;
    while(comp > temp){
        if((flag==0)&&((millis_cnt&0x3FFFFF)==0)&&(millis_cnt>0x3FFFFF)){
            flag = 1;
        }
        if(flag) temp = micros() + 0x400000UL * 1000;
        else     temp = micros();
    }
}
