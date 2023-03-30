
#include "main.h"

/*Define port and pin*/	
#define 	KEYPAD_PORT				GPIOB
#define 	KEYPAD_1_PIN			GPIO_PIN_0 
#define 	KEYPAD_2_PIN			GPIO_PIN_1
#define 	KEYPAD_3_PIN			GPIO_PIN_3
#define 	KEYPAD_4_PIN			GPIO_PIN_4


/*Declare Function*/
uint8_t keypad_read(void);


