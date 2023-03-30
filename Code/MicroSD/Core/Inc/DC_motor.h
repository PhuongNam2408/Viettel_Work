/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DC_MOTOR_H
#define __DC_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
extern uint32_t temp_max;
extern uint32_t temp_min;
/*Define output PWM pins*/

//Use Timer3 in PWM mode, with channel 1 - PA6 and channel 2 - PA7
#define DC_motor_Port							GPIOA
#define DC_motor_L298_IN_1_Pin		GPIO_PIN_6
#define DC_motor_L298_IN_2_Pin		GPIO_PIN_7
#define DC_motor_L298_EN_Pin			GPIO_PIN_5


/*Declare function*/
void DC_motor_Enable(uint8_t state);
uint32_t DC_motor_Speed_Based_on_Temp(uint32_t current_temp);		
void DC_motor_PWM(uint32_t current_speed);

/*State of motor*/
#define DC_MOTOR_ENABLE		1U
#define DC_MOTOR_DISABLE	0U

#endif /* __DC_MOTOR_H */
