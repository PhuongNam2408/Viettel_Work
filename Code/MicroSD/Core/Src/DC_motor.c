#include "DC_motor.h"

void DC_motor_Enable(uint8_t state){
	if(state == DC_MOTOR_ENABLE){
		//Set EN_Pin to Enable motor
		HAL_GPIO_WritePin(DC_motor_Port,DC_motor_L298_EN_Pin, GPIO_PIN_SET);
	}
	else if(state == DC_MOTOR_DISABLE){
		//Reset EN_Pin to Disable motor
		HAL_GPIO_WritePin(DC_motor_Port,DC_motor_L298_EN_Pin, GPIO_PIN_RESET);
	}
}


/*Ham tra ve %toc do theo nhiet do theo temp_max va temp_min*/
uint32_t DC_motor_Speed_Based_on_Temp(uint32_t current_temp)
{
	uint32_t speed_temperature;	//%speed based on current_temp
	current_temp = current_temp > temp_max ? temp_max : current_temp;
	current_temp = current_temp < temp_min ? temp_min : current_temp;
	speed_temperature = 100 * (current_temp - temp_min) / (temp_max - temp_min);
	return speed_temperature;
}	

/*Ham cho phep thay doi toc do dong co*/
void DC_motor_PWM(uint32_t current_speed)
{
	PWM_Speed(TIM_CHANNEL_1, current_speed);
	PWM_Speed(TIM_CHANNEL_2, 0);
}



