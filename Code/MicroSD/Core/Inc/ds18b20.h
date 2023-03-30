/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DS18B20_H
#define __DS18B20_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*Define Ports and Pins*/
#define DS18B20_Port  GPIOA
#define DS18B20_Data_Pin  GPIO_PIN_1

/*Declare Function*/
uint8_t Ds18b20_Init (void);
void Ds18b20_Write1Bit (uint8_t bit);
uint8_t Ds18b20_Read1Bit (void);
void Ds18b20_WriteCommand (uint8_t command);
uint8_t Ds18b20_ReadData (void);
uint32_t Ds18b20_NhietDo(void);


/*Define DS18B20 Command*/
#define DS18B20_SKIP_ROM					0xCC
#define DS18B20_CONVERT_T					0x44
#define DS18B20_READ_SCRATCHPAD		0xBE



#endif /* __DS18B20_H */
