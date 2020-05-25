/**
	******************
	*@file				TPS68401_RGB.c
	*@author			Sami Emre Yagli
	*@created			30.01.2020
	*@UC					STM32F4xx
	*@brief				TPS68401 RGB Led Driver

	================================================================================================
	MODES
	================================================================================================
	***Direct Mode: 
			#In DIRECT mode the LED channels can be controlled independently through the I2C interface. For each channel
			there is a PWM control register (R_PWM, G_PWM, B_PWM) which contains the PWM duty cycle.
	
	***Program Memory Modes: 
			#The device can store 16 16-bit commands for each channel (R, G, B). To use in program memory mode, 
			LOAD and RUN mode will be used. Firstly the LOAD mode should be selected, after the writing is finished,
			RUN mode should be selected. 
	
			+++Load Mode		: Read / write access to program memory is allowed only in LOAD mode and only to the channel in LOAD mode.
			+++Run Mode			: In RUN mode the LED controller executes instructions stored in program memory.
			Note:GPO and Trig pins were not needed in the project. 

	================================================================================================
	
	******************
**/

#include "TPS68401_RGB.h"

Mode_t m;

/******************* Private address defining part *****************************/
uint8_t  TPS68401_I2C_ADDR;
uint8_t  TPS68401_ENABLE 			= 0x00;
uint8_t  TPS68401_OP_MODE 		=	0x01;
uint8_t  TPS68401_R_PWM				= 0x02;
uint8_t  TPS68401_G_PWM				=	0x03;
uint8_t  TPS68401_B_PWM				=	0x04;
uint8_t  TPS68401_R_CURRENT		= 0x05;
uint8_t  TPS68401_G_CURRENT 	= 0x06;
uint8_t  TPS68401_B_CURRENT		= 0x07;
uint8_t  TPS68401_CON					= 0x08;
uint8_t  TPS68401_R_PC		 		= 0x09;
uint8_t  TPS68401_G_PC		 		= 0x0A;
uint8_t  TPS68401_B_PC		 		= 0x0B;
uint8_t  TPS68401_STATUS	 		= 0x0C;
uint8_t  TPS68401_RESET		 		= 0x0D;
uint8_t  TPS68401_GPO					= 0x0E;
/*********************** End Of addressing ***********************/

/**@brief: Address selection function for using multiple I2C devices. 
	*@param:  uint8_t ADDR_SEL : Address selection(i.e. 0,1,2,3) 
*/
static void addr_select(uint8_t ADDR_SEL)
{	
	switch(ADDR_SEL)
	{
		case 0:
			TPS68401_I2C_ADDR = TPS68401_ADDR_SEL_0;
			break;
		case 1:
			TPS68401_I2C_ADDR = TPS68401_ADDR_SEL_1;
			break;
		case 2:
			TPS68401_I2C_ADDR = TPS68401_ADDR_SEL_2;
			break;
		case 3:
			TPS68401_I2C_ADDR = TPS68401_ADDR_SEL_3;
			break;
		default: 
			TPS68401_I2C_ADDR = HAL_ERROR;
			break;
	}
}

/**@brief: 	Checks the device and reset
	*@param:	uint8_t ADDR_SEL : Address selection(i.e. 0,1,2,3)
*/
static void RGB_Ready_Reset(uint8_t ADDR_SEL)
{
	addr_select(ADDR_SEL);

	uint8_t reset_value = 0xFF; //Reset
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, TPS68401_RESET, 1, &reset_value, 1, TPS68401_TIMEOUT);
	
	reset_value = 0;						//Set again
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, TPS68401_RESET, 1, &reset_value, 1, TPS68401_TIMEOUT);
}

/**@brief:  Initialization in DIRECT mode with setting the ENABLE, OP_MODE and CONFIG registers.
	*@param:  uint8_t ADDR_SEL : Address selection(i.e. 0,1,2,3) 
*/
void rgb_init(uint8_t ADDR_SEL)
{
	m.op_mode = TPS68401_OP_Direct;
	m.en_mode = TPS68401_EN_Direct;
	m.config_reg = TPS68401_Config_Reg;
	
	uint8_t InitData[3] = {m.en_mode, m.config_reg, m.op_mode};
	
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, TPS68401_ENABLE, 1, &InitData[0], 1, TPS68401_TIMEOUT); //ENABLE REG
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, TPS68401_CON,    1, &InitData[1], 1, TPS68401_TIMEOUT); //CONFIG REG
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, TPS68401_OP_MODE,1, &InitData[2], 1, TPS68401_TIMEOUT); //OP_MODE 
}

/**@brief: 	Drive RGB through PWM control register in DIRECT mode.
	*@param:	uint8_t ADDR_SEL : Address selection(i.e. 0,1,2,3)	. 
						uint32_t RGB_hex : Hex color code (ie 0xFE0245)
*/
void rgb_drive(uint8_t ADDR_SEL,uint32_t RGB_hex)
{
	addr_select(ADDR_SEL);
	
	uint8_t red_hex   = 0xFF - ((uint8_t) ((RGB_hex & 0x00FF0000) >> 16));
	uint8_t green_hex = 0xFF - ((uint8_t) ((RGB_hex & 0x0000FF00)>> 8));
	uint8_t blue_hex  = 0xFF - ((uint8_t) ( RGB_hex & 0x000000FF));

	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, TPS68401_R_PWM, 1, &red_hex, 1, TPS68401_TIMEOUT);	
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, TPS68401_G_PWM, 1, &green_hex, 1, TPS68401_TIMEOUT);
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, TPS68401_B_PWM, 1, &blue_hex, 1, TPS68401_TIMEOUT);
}

/**@brief:  Restriction the current to max.
	*@param:  uint8_t red_c, uint8_t green_c, uint8_t blue_c
**/
static void rgb_current(uint8_t red_c, uint8_t green_c, uint8_t blue_c)//default current = (175,175,175) = 17.5mA
{
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, TPS68401_R_CURRENT, 1, &red_c,   1, TPS68401_TIMEOUT);	
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, TPS68401_G_CURRENT, 1, &green_c, 1, TPS68401_TIMEOUT);
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, TPS68401_B_CURRENT, 1, &blue_c,  1, TPS68401_TIMEOUT);
}


/*
 *PROGRAM MEMORY REGISTER FUNCTIONS
 */

/**@brief:  Drive RGB through Program Memory register.
	*@param:  uint8_t mem_addr : Memory address of device (i.e. 0x10) .
					  uint8_t pwm_val	: Pwm value for that channel (0-255)*/
void set_pwm(uint8_t mem_addr,uint8_t pwm_val){		//Pwm setting for program memory registers.
	uint8_t pwm_inst = 0x40;
	pwm_val = 255 - pwm_val;
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, mem_addr, 1, &pwm_inst, 1, TPS68401_TIMEOUT);	
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, mem_addr+1, 1, &pwm_val, 1, TPS68401_TIMEOUT);	
}

/**@brief:  Waits for a while. (Tmax = 1000 ms)
	*@param:  uint8_t mem_addr : Memory address of device (i.e. 0x12) . 
						uint16_t time_ms : Necessary time in ms.*/
void wait(uint8_t mem_addr,uint16_t time_ms){ 							//time_ms is 1000ms max.
	uint8_t step_time = (32767 * time_ms/1000) / (512+1); 		//psc=512 	//step_time = 63;
	uint8_t wait_val = (1 << 6) | step_time;
	uint8_t zero = 0;
	
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, mem_addr, 1, &wait_val, 1, TPS68401_TIMEOUT);
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, mem_addr+1, 1, &zero, 1, TPS68401_TIMEOUT);
}

/**@brief:  The ramp command generates a PWM ramp starting from the current value. At each ramp step the PWM value is 
					incremented or decremented by one with sign. Time for one step is defined by time_ms.
	*@param:  uint8_t mem_addr : Memory address of device (i.e. 0x12)
						uint16_t time_ms : Necessary time in ms.
						uint8_t sign		 : 0 - incerement; 1 - decrement
						uint8_t step		 : step number
**/
void ramp(uint8_t mem_addr,uint16_t time_ms, uint8_t sign,uint8_t inc_num){
	if(time_ms > 1000.0)
		time_ms = 999;
	
	if(time_ms < 0)
		time_ms = 1;
	
	uint8_t step_time = (32768.0 * time_ms)/ (513.0 * 1000.0); 	//psc=512 //step_time = 63; 
	uint8_t ramp_inst = 0x40 | step_time; 
	uint8_t ramp_val  = (sign << 7) | inc_num; //sign=0 : increment to inc_num; sign=1 : decrement from inc_num
	
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, mem_addr, 1, &ramp_inst, 1, TPS68401_TIMEOUT);
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, mem_addr+1, 1, &ramp_val, 1, TPS68401_TIMEOUT);
}

/**@brief:  Loop instruction. 
	*@param:  uint8_t mem_addr : Memory address of device (i.e. 0x12)
						uint8_t loop_cntr: Code between (step number) and BRANCH command will be executed (loop count + 1) times. 
						uint8_t step_number: Set (loop count) = 0 for infinite looping
*/
void branch(uint8_t mem_addr, uint8_t loop_cntr, uint8_t step_number){
	uint16_t branch_val= (5 << 13) | (loop_cntr << 7) | step_number;
	uint8_t msb = ((branch_val >> 8) & 0xff);
	uint8_t lsb = ((branch_val >> 0) & 0xff);
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, mem_addr, 1, &msb, 1, TPS68401_TIMEOUT);	
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, mem_addr+1, 1, &lsb, 1, TPS68401_TIMEOUT);	
}


/**@brief:  Command resets program counter register and continues executing program from the 00H location.
	*@param:  uint8_t mem_addr : Memory address of device (i.e. 0x12)
*/
void go_to_start(uint8_t mem_addr){
	uint8_t res=0;
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, mem_addr, 1, &res, 1, TPS68401_TIMEOUT);	
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, mem_addr+1, 1, &res, 1, TPS68401_TIMEOUT);
}

/**@brief: End of the instructions.
	*@param: uint8_t mem_addr : Memory address of device (i.e. 0x12). 
					 uint8_t interr		: After the 'end' is executed, interrupt pin enable : 1 ; disable : 0 .
					 uint8_t def 			: 
*/
void end(uint8_t mem_addr, uint8_t interr, uint8_t def){
	uint8_t end_val = (6 << 5) | (interr<<4) | (def<<3);
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, mem_addr, 1, &end_val, 1, TPS68401_TIMEOUT);	
}

/**@brief:  Load operation.
	*@param:  uint8_t mem_addr : Memory address of device.
*/
void RGB_Program_Mem_Load(uint8_t ADDR_SEL)
{
	m.op_mode = TPS68401_OP_Load;
	m.en_mode = TPS68401_EN_Load;
	
	uint8_t InitData[2] = {m.en_mode, m.op_mode};
	
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, TPS68401_ENABLE, 1, &InitData[0], 1, TPS68401_TIMEOUT); //ENABLE REG
	HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, TPS68401_OP_MODE,1, &InitData[1], 1, TPS68401_TIMEOUT); //OP_MODE 
}

/**@brief:  Run operation.
	*@param:  uint8_t mem_addr : Memory address of device.
*/
void RGB_Program_Mem_Run(void)
{
		m.op_mode = TPS68401_OP_Direct;
		m.en_mode = TPS68401_EN_Direct;
		m.config_reg = TPS68401_Config_Reg;
		
		uint8_t InitData[3] = {m.en_mode, m.config_reg, m.op_mode};
		
		HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, TPS68401_CON,    1, &InitData[1], 1, TPS68401_TIMEOUT); //CONFIG REG
		HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, TPS68401_OP_MODE,1, &InitData[2], 1, TPS68401_TIMEOUT); //OP_MODE 
		HAL_I2C_Mem_Write(&TPS68401_I2C, TPS68401_I2C_ADDR, TPS68401_ENABLE, 1, &InitData[0], 1, TPS68401_TIMEOUT); //ENABLE REG
}

/**@brief:	Example code to use program memory.
	*@param:  uint8_t mem_addr : Memory address of device.
*/
void pm_example(uint8_t ADDR_SEL){
	
	RGB_Program_Mem_Load(ADDR_SEL);
	
	//////////
	set_pwm(0x10,50);
	wait(0x12,200);
	set_pwm(0x14,250);
	wait(0x16,500);
	branch(0x18,4,0);
	end(0x20,1,0);
	/////////////
	set_pwm(0x30,250);
	wait(0x32,200);
	set_pwm(0x34,5);
	wait(0x36,500);
	branch(0x38,5,0);
	end(0x40,1,0);
	////////////
	set_pwm(0x50,250);
	wait(0x52,200);
	set_pwm(0x54,5);
	wait(0x56,500);
	branch(0x58,6,0);
	end(0x60,1,0);
	//////////
	RGB_Program_Mem_Run();
}
