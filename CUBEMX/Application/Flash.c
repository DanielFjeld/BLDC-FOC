/*
 * Flash.c
 *
 *  Created on: Dec 11, 2023
 *      Author: Daniel
 */
#include "Flash.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g431xx.h"

#define  PAGE_NUMBER 63 //max = 64
#define PAGE_SIZE 2048 //2k

Flash *ptr;

Flash RAM = {
		.Angle_kp = 4.0f,
		.Angle_ki = 0.0f,
		.Angle_kd = 0.0f,

		.Velocity_kp = 0.1f, //0.2
		.Velocity_ki = 4.0f, //0.01
		.Velocity_kd = 0.0f, //0.00001

		.Current_kp = 0.02f,
		.Current_ki = 20.0f,
		.Current_kd = 0.0f,

		.Current_offset_kp = 0.001f,
		.Current_offset_ki = 30.1f,
		.Current_offset_kd = 0.0f,

		.Velocity_limit = 4000.0f, //rpm
		.Current_limit = 10.0f //ampere
};

uint64_t test_data[2] = {0};

uint32_t flash_read(uint32_t address){
    return (uint32_t*)address;
}

void flash_write(uint32_t page, uint64_t data[], uint32_t size){
	uint32_t PageError;
	FLASH_EraseInitTypeDef pEraseInit = {
			.Banks = FLASH_BANK_1,
			.NbPages = 1,
			.Page = page,
			.TypeErase = FLASH_TYPEERASE_PAGES
	};

	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&pEraseInit, &PageError);

    for(int i = 0; i < size/sizeof(uint64_t); i++){
    	uint32_t address = FLASH_BASE + PAGE_SIZE*page + i*sizeof(uint64_t);
    	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,address, data[i]);
    }
    HAL_FLASH_Lock();
}

void Flash_init(){
	ptr = &RAM;
}

void Flash_save(Flash *data){
	test_data[1] = 0x0a0a;
	flash_write(PAGE_NUMBER, test_data, sizeof(test_data));
}

Flash *Flash_get_values(){
	return flash_read(FLASH_BASE + PAGE_SIZE*PAGE_NUMBER);
}


