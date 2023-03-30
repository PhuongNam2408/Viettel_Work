
#include "main.h"

uint8_t FLASH_Read1Byte(uint32_t addr);
void FLASH_ErasePage(uint32_t page_address);
void FLASH_WriteHalfWord(uint32_t addr, uint16_t value);
void FLASH_WriteWord(uint32_t addr, uint32_t value);
void FLASH_WriteArray(uint32_t addr, uint8_t *arr, uint32_t len);
uint32_t FLASH_ReadWorld(uint32_t addr);



