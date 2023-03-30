

#include "ds18b20.h"
/*****************************************************
  * @brief Initialize Ds18b20 
	* @variable None
  * @retval presence pulse. If presence == 0 -> correct, else wrong
  ****************************************************/
uint8_t Ds18b20_Init (void)
{
	uint8_t presence = 100;
	GPIO_Output(DS18B20_Port,DS18B20_Data_Pin);
	//Dua chan len cao de reset chan
	HAL_GPIO_WritePin(DS18B20_Port, DS18B20_Data_Pin, GPIO_PIN_SET);
	//Pull Data_Pin low for 500us
	HAL_GPIO_WritePin(DS18B20_Port, DS18B20_Data_Pin, GPIO_PIN_RESET);
	delay_us(480);
	//Release the bus by set the Data_Pin to Input mode
	GPIO_Input(DS18B20_Port,DS18B20_Data_Pin);
	//delay 60us according to the datasheet
	delay_us(80);
	//Receive the presence pulse
	presence = HAL_GPIO_ReadPin(DS18B20_Port,DS18B20_Data_Pin);
	delay_us(400);
	return presence;
}

/*****************************************************
  * @brief Write 1 bit from MCU to Ds18b20
	* @variable bit: data transmitted
  * @retval None
  ****************************************************/
void Ds18b20_Write1Bit (uint8_t bit)
{
	//Set the Data_pin to Output mode
	GPIO_Output(DS18B20_Port,DS18B20_Data_Pin);
	//Dua chan len cao de reset chan
	HAL_GPIO_WritePin(DS18B20_Port, DS18B20_Data_Pin, GPIO_PIN_SET);
	//Pulling Data_pin low
	HAL_GPIO_WritePin(DS18B20_Port,DS18B20_Data_Pin,GPIO_PIN_RESET);
	if(bit == 1){
		//delay 15us
		delay_us(15);
		//Release the bus by set the Data_Pin to Input mode
		GPIO_Input(DS18B20_Port,DS18B20_Data_Pin);
	}
	else if (bit == 0){
		delay_us(75);
		//Release the bus by set the Data_Pin to Input mode
		GPIO_Input(DS18B20_Port,DS18B20_Data_Pin);
	}
	//Delay 10us recovery time for next operate
	delay_us(10);
}

/*****************************************************
  * @brief MCU read 1 bit from Ds18b20
	* @variable None
  * @retval 1 bit data received
  ****************************************************/
uint8_t Ds18b20_Read1Bit (void){
	uint8_t temp = 0;		
	//Set the Data_pin to Output mode to pulling low for 5us
	GPIO_Output(DS18B20_Port,DS18B20_Data_Pin);
	//Dua chan len cao de reset chan
	HAL_GPIO_WritePin(DS18B20_Port, DS18B20_Data_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DS18B20_Port,DS18B20_Data_Pin,GPIO_PIN_RESET);
	delay_us(2);
	//Release the bus by set the Data_Pin to Input mode
	GPIO_Input(DS18B20_Port,DS18B20_Data_Pin);
	delay_us(9);
	//Read the value on the bus
	temp = HAL_GPIO_ReadPin(DS18B20_Port,DS18B20_Data_Pin);
	/*Delay 55us + 10us for the rest of read operate and 
	  the recovery time for next operate*/
	delay_us(55+10);
	return temp; 
}

/*****************************************************
  * @brief a 8-bit command from MCU to Ds18b20
	* @variable command: Function Command Set in datasheet
  * @retval None
  ****************************************************/
void Ds18b20_WriteCommand (uint8_t command){
	//Transmit LSB first
	int i=8;
	while(i--){
		Ds18b20_Write1Bit(command & 1);
		command >>= 1;
	}
}

/*****************************************************
  * @brief MCU read 8-bit data from Ds18b20
	* @variable None
  * @retval 8-bit data received
  ****************************************************/
uint8_t Ds18b20_ReadData (void){
	uint8_t temp = 0;
	//receive LSB first 
  for (int i=0; i<8; i++){
		temp |= Ds18b20_Read1Bit()<<i;
	}
	return temp;
}

/*****************************************************
  * @brief MCU read 8-bit data from Ds18b20
	* @variable None
  * @retval 8-bit data received
  ****************************************************/
uint8_t Ds18b20_Read (void){
	uint8_t temp = 0;
	//receive LSB first 
  for (int i=0; i<8; i++){
		temp |= Ds18b20_Read1Bit()<<i;
	}
	return temp;
}

/*****************************************************
  * @brief MCU read 8-bit data from Ds18b20
	* @variable None
  * @retval 8-bit data received
  ****************************************************/
uint32_t Ds18b20_NhietDo(){
	uint32_t flag = 1, temp1, temp2;
	/*Sequence 1: Convert Temperature*/
	//Init
	flag = Ds18b20_Init();
	if(flag == 0){
		//presence is correct
		//Skip ROM
		Ds18b20_WriteCommand(DS18B20_SKIP_ROM);
		//Convert Temperature
		Ds18b20_WriteCommand(DS18B20_CONVERT_T);
		HAL_Delay(750);
		/*Sequence 2: Read scratchpad*/
		//Init
		flag = Ds18b20_Init();
		if(flag == 0){
			//presence is correct
			//Skip ROM
			Ds18b20_WriteCommand(DS18B20_SKIP_ROM);
			//Read scratchpad
			Ds18b20_WriteCommand(DS18B20_READ_SCRATCHPAD);
			
			//Read Temperature LSB 
			temp1 = Ds18b20_ReadData();
			
			//Read Temperature MSB
			temp2 = Ds18b20_ReadData();
			return (temp2 << 4) + ((float)temp1)/16;		// >>4 tuong duong voi chia cho 2^4 = chia cho 16
		}
		else 
			//presence is wrong
			return flag;
	}
	else 
		//presence is wrong
		return flag;
}

