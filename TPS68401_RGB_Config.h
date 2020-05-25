/**
******************
	*@file				TPS68401_RGB_Config.h
	*@author			Sami Emre Yagli
	*@created			30.01.2020
	*@UC					STM32F4xx
	*@brief				RGB Led Control with using TPS68401.
	
																						REGISTERS
	================================================================================================================
																			  RW7   |  RW6 	 	 |  RW5	 |  RW4		|  RW3	 |  	RW2	 |   RW1	 |  RW0 
	
	#ENABLE register in DIRECT MODE 	:		LOG_EN | CHIP_EN | 0		 |	0			|	 0		 |	  0	   |	  0		 |	  0
										in LOAD MODE	 	:		LOG_EN | CHIP_EN | 0		 |	0			|	 0		 |	  0	   |	  0		 |	  0 		(hold)
										in RUN MODE		 	:		LOG_EN | CHIP_EN | 1		 |	0			|	 1		 |	  0 	 |	  1		 |	  0			(continue)
										in RUN MODE		 	:		LOG_EN | CHIP_EN | 1		 |	1			|	 1		 |	  1 	 |	  1		 |	  1			(execute)
			--LOG_EN: 0- Enable linear current adjustment mode
								1- Enable logarithmic current adjustment mode
			--CHIP_EN:0- Forcing EN pin low resets CHIP_EN to zero
	----------------------------------------------------------------------------------------------------------------
	#OP_MODE register in DIRECT MODE	:		 N/A	 |   N/A	 | 1		 |	1			|	 1		 |	  1 	 |	  1		 |	  1			(direct)
										 in LOAD MODE		:		 N/A	 |   N/A	 | 0		 |	1			|	 0		 |	  1 	 |	  0		 |	  1			(load)
										 in RUN MODE		:		 N/A	 |   N/A	 | 1		 |	0			|	 1		 |	  0 	 |	  1		 |	  0			(run)
	----------------------------------------------------------------------------------------------------------------
	#CONFIG register 									:		 N/A	 | PWM_HF  |PWRSAVE|  CP_MODE[1:0]   |R_TO_BAT |	  CLK_DET[1:0] 	
			--PWM_HF: 0- 256Hz
								1- 558Hz
			--PWRSAVE:0- Power save mode disabled
								0- Power save mode enabled
			--CP_MODE[1:0]: 00- Charge pump operation is off
											01- Charge pump forced 1x mode
											10- Charge pump forced 1.5x mode
											11- Charge pump automatic mode
			--R_TO_BAT: 1- Red channel connected to VDD
									0- Red channel connected to charge pump output.
									Note: Green and blue channels are always connected to charge pump output.
			--CLK_DET[1:0]: 00- External clk source (CLK_32)
											01- Internal clock source, clock detection disabled
											10- Automatically select clock source
											11- Internal clock source, clock detection enabled
	
******************
**/

#ifndef RGB_Strp_Driver_Config
#define RGB_Strp_Driver_Config

#define  TPS68401_I2C					hi2c1

#define  TPS68401_ADDR_SEL_0	0x64
#define  TPS68401_ADDR_SEL_1	0x66
#define  TPS68401_ADDR_SEL_2	0x68
#define  TPS68401_ADDR_SEL_3	0x6A

#define  TPS68401_EN_PIN			0
#define  TPS68401_TIMEOUT 		100

#define  TPS68401_Config_Reg  0x6F

#define	 TPS68401_OP_Direct   0x3F
#define  TPS68401_EN_Direct		0x40

#define	 TPS68401_OP_Load			0x15
#define	 TPS68401_EN_Load			0x40

#define	 TPS68401_OP_Run			0x2A
#define	 TPS68401_EN_Run			0xE8

#define	 R_Current_Limit			0x0A //restricted current max to 1 mA
#define	 G_Current_Limit			0x0A
#define	 B_Current_Limit			0x0A



#if (TPS68401_EN_PIN == 1)
	#define TPS68401_EN_GPIO		GPIOB
	#define TPS68401_EN_PIN			GPIO_PIN_13
#endif

#endif