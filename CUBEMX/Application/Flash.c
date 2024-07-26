/*
 * Flash.c
 *
 *  Created on: Dec 11, 2023
 *      Author: Daniel
 */
#include "Flash.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g431xx.h"
#include <stdlib.h>
#include "string.h"

#define  PAGE_NUMBER 63 //max = 64
#define PAGE_SIZE 2048 //2k

#define ID_STRING "FLASH STORAGE ID V0.0.1\0"

const char ID[] =  {ID_STRING};

Flash *ptr = (Flash*)(FLASH_BASE + PAGE_SIZE*PAGE_NUMBER);
Flash Stored_in_RAM = {0};

#define RAM_COMPARE 94+100
Flash RAM = {
		.ID = ID_STRING,

		.Angle_kp = 3.0f,
		.Angle_ki = 0.0f,
		.Angle_kd = 0.0f,

		.Velocity_kp = 0.02f, //0.18,// 0.1f, //0.2
		.Velocity_ki = 10.0,//4.0f, //0.01
		.Velocity_kd = 0.0,//0.0f, //0.00001

		.Current_kp = 0.01f,//0.02
		.Current_ki = 1000.0f,//20.0f,
		.Current_kd = 0.0f,

		.Current_offset_kp = 0.01f,
		.Current_offset_ki = 1000.0f,
		.Current_offset_kd = 0.0f,

		.Velocity_limit = 4000.0f, //rpm
		.Current_limit = 10.0f, //ampere

		.Encoder1_offset = 5.0f,

		.VBAT = 22.0f,           //volt
		.MAX_VOLTAGE = 22.0f,       //volt
		.MAX_CURRENT = 5.0f, //16.0f      //amp
		.MAX_VELOCITY = 6000.0f,   //RPM
		.MIN_POSITION = 0.0f,      //degrees
		.MAX_POSITION = 360.0*16.5f,    //degrees
		.MAX_RAMP_RPM = 6000,

		.calibrate_on_start = 0,

		.CAN_ID = 0x100
};

uint64_t test_data[2] = {0};

Flash* flash_read(uint32_t address){
    return (Flash*)address;
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

void Flash_init(uint8_t use_flash){
	if (!use_flash || memcmp(RAM.ID, ptr->ID, sizeof(ID_STRING) != 0)){
		//memcpy(&Stored_in_RAM, ptr, sizeof(Flash));
		memcpy(&Stored_in_RAM, &RAM, sizeof(Flash));
		Flash_save();
	}
	else memcpy(&Stored_in_RAM, ptr, sizeof(Flash));
}

void Flash_save(){
	flash_write(PAGE_NUMBER, (uint64_t*)&Stored_in_RAM, sizeof(Flash));
}

Flash *Flash_get_values(){
	return &Stored_in_RAM; //*ptr;// = flash_read(FLASH_BASE + PAGE_SIZE*PAGE_NUMBER);
}
void flash_check(){
	if(memcmp(&Stored_in_RAM, ptr, sizeof(Flash)) == 0) HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 1);
	else HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 0);
}
