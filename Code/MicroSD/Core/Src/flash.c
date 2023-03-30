
#include "flash.h"

uint8_t FLASH_Read1Byte(uint32_t addr)
{
	uint8_t data;
	//Castype bien addr ve dang con tro + __IO (volatile)
	data = * ( (__IO uint32_t*) addr);		//doc gia tri 1 byte tai dia chi addr cua FLASH
	return data;
}
	
void FLASH_WriteHalfWord(uint32_t addr, uint16_t value)
{
	//Unlocking the Flash memory
	HAL_FLASH_Unlock();
	//Write value in to addr use halfword
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, value);
	//Locking the Flash memory
	HAL_FLASH_Lock();
}

void FLASH_WriteWord(uint32_t addr, uint32_t value)
{
	//Unlocking the Flash memory
	HAL_FLASH_Unlock();
	//Write value in to addr use halfword
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, value);
	//Locking the Flash memory
	HAL_FLASH_Lock();
}

void FLASH_WriteArray(uint32_t addr, uint8_t *arr, uint32_t len)
{
	//Gia tri la u8, khi castype uint16 se lay 2 gia tri lien tiep gop vao nhau	
	uint16_t *temp = (uint16_t *)arr;
	/* Viet 2 byte 1 lan: (len+1)/2 */
	for(int i=0; i < (len+1)/2; i++)
	{
		FLASH_WriteHalfWord(addr + 2*i, *temp);
		temp++;
	}
}

void FLASH_ErasePage(uint32_t page_address)
{
	//Declare flash erase typedef
	FLASH_EraseInitTypeDef erase_init;
	erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
	erase_init.Banks = 1;
	erase_init.NbPages = 1;
	erase_init.PageAddress = page_address;
	//Declare address get error
	uint32_t page_err;
	//Unlocking the Flash memory
	HAL_FLASH_Unlock();
	//Write value in to addr use halfword
	HAL_FLASHEx_Erase(&erase_init, &page_err);
	//Locking the Flash memory
	HAL_FLASH_Lock();	
}

uint32_t FLASH_ReadWorld(uint32_t addr)
{
	uint32_t data = 0;
	for(int i = 0; i < 4; i++)
	{
		//dich trai 1 byte moi lan lap lai
		data |= FLASH_Read1Byte(addr + i) << 8*i;
	}
	return data;
}



