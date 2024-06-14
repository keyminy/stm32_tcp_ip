#include "pwm_led.h"
#include "time.h"
#include <string.h>

extern uint8_t led_flag;
extern uint8_t udp_data[100];
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart3;
uint8_t led[20];

void turn_on_LED_in_PWM_manner(){
	if(led_flag==1){
		// udp_data = "LED:"
		led_flag = 0;
		strncpy(led,udp_data+4,2);
		HAL_UART_Transmit(&huart3, led, strlen(led), 10);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,atoi(led));
		delay_us(20);
	}
}
