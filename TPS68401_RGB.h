/**
	******************
	*@file				TPS68401_RGB.h
	*@author			Sami Emre Yagli
	*@created			30.01.2020
	*@UC					STM32F4xx
	*@brief				Stripe RGB Led Control with using TPS68401.

	******************
**/
#ifndef RGB_Strp_Driver
#define RGB_Strp_Driver

#include "stm32f4xx_hal.h"
#include "TPS68401_RGB_Config.h"

void rgb_init(uint8_t ADDR_SEL);
void rgb_drive(uint8_t ADDR_SEL, uint32_t RGB_hex);

typedef struct mode{
	uint8_t op_mode;	
	uint8_t en_mode;
	uint8_t config_reg;
}Mode_t;

extern I2C_HandleTypeDef TPS68401_I2C;

/*Program memory register functions*/
void set_pwm(uint8_t mem_addr,uint8_t pwm_val);
void wait(uint8_t mem_addr,uint16_t time_ms);
void ramp(uint8_t mem_addr,uint16_t time_ms, uint8_t sign,uint8_t inc_num);
void branch(uint8_t mem_addr, uint8_t loop_cntr, uint8_t step_number);
void go_to_start(uint8_t mem_addr);
void end(uint8_t mem_addr, uint8_t interr, uint8_t def);
void pm_example(uint8_t ADDR_SEL);

#endif