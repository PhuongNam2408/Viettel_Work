

#include "keypad.h"


/**
  * @brief  Read the value of 1x4 keypad
  * @retval num keypad, if dont have any key => return 0xFF
  */
uint8_t keypad_read(void){
	if (HAL_GPIO_ReadPin(KEYPAD_PORT,KEYPAD_1_PIN) == GPIO_PIN_RESET){
		//Nút 1 duoc an
		
		while (HAL_GPIO_ReadPin(KEYPAD_PORT,KEYPAD_1_PIN) == GPIO_PIN_RESET);
		//Doi den khi nut 1 duoc nha ra
		return 1;
	}
	if (HAL_GPIO_ReadPin(KEYPAD_PORT,KEYPAD_2_PIN) == GPIO_PIN_RESET){
		//Nút 2 duoc an
		
		while (HAL_GPIO_ReadPin(KEYPAD_PORT,KEYPAD_2_PIN) == GPIO_PIN_RESET);
		//Doi den khi nut 2 duoc nha ra
		return 2;
	}
	if (HAL_GPIO_ReadPin(KEYPAD_PORT,KEYPAD_3_PIN) == GPIO_PIN_RESET){
		//Nút 3 duoc an
		
		while (HAL_GPIO_ReadPin(KEYPAD_PORT,KEYPAD_3_PIN) == GPIO_PIN_RESET);
		//Doi den khi nut 3 duoc nha ra
		return 3;
	}
	if (HAL_GPIO_ReadPin(KEYPAD_PORT,KEYPAD_4_PIN) == GPIO_PIN_RESET){
		//Nút 4 duoc an
		
		while (HAL_GPIO_ReadPin(KEYPAD_PORT,KEYPAD_4_PIN) == GPIO_PIN_RESET);
		//Doi den khi nut 4 duoc nha ra
		return 4;
	}
	return 0xFF;
}

	



