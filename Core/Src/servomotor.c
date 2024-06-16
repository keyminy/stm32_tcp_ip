#include "servomotor.h"

extern volatile int TIM11_1ms_servo_count;
extern TIM_HandleTypeDef htim2;
extern uint8_t servo_flag;
extern uint8_t udp_data[100];
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart3;
uint8_t servo[20];

void servo_motor_in_PWM_manner(void) {
	static uint8_t servo_state = 0;
	if(servo_flag==1){
		servo_flag=0;
		// udp_data = "SERVO:"
		strncpy(servo,udp_data+6,2);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,atoi(servo));
		delay_us(20);
	}

}
