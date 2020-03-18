/*
 * Board.h
 *
 *  Created on: Nov 23, 2019
 *      Author: FireSourcery
 */

#ifndef BOARD_H_
#define BOARD_H_

/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "GPIOA.h"
#include "GPIOB.h"
//#include "FTM1.h"
//#include "FTM0.h"
#include "FTM2.h"
//#include "WDOG.h"

/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"

#include "GPIO_PDD.h"
#include "WDOG_PDD.h"
#include "ADC_PDD.h"
#include "FTM_PDD.h"

#define MASK_LSB_1 0x01
#define MASK_LSB_2 0x03
#define MASK_LSB_3 0x07
#define MASK_LSB_4 0x0F
#define MASK_LSB_16 0xFFFF

//	I/O Init
//  GPIOA PTA0 - ADC_SE0 	- B+_AD
//  GPIOA PTA1 - ADC_SE1 	- V_1A
//  GPIOA PTA2 - UART0_RX 	- RxD0
//  GPIOA PTA3 - UART0_TX 	- TxD0
//  GPIOA PTA4 - SWDIO		- SWDIO
//  GPIOA PTA5 - RST		- RST
//  GPIOA PTA6 - ADC_SE2 	- V_1B
//  GPIOA PTA7 - 
//  
//  GPIOA PTB0 - ADC_SE4 - I_AD_1
//  GPIOA PTB1 - ADC_SE5 - I_AD_2
//  GPIOA PTB2 - 
//  GPIOA PTB3 - 
//  GPIOA PTB4 - FTM2_CH4 - PWM_1B
//  GPIOA PTB5 - FTM2_CH5 - PWM_1A
//  GPIOA PTB6 -			- Button/LED
//  GPIOA PTB7 - 
//  
//  GPIOA PTC0 - FTM2_CH0 	- PWM_2C
//  GPIOA PTC1 - FTM2_CH1 	- PWM_2B
//  GPIOA PTC2 - FTM2_CH2 	- PWM_2A
//  GPIOA PTC3 - FTM2_CH3 	- PWM_1C
//  GPIOA PTC4 - SWD_CLK    
//  GPIOA PTC5 - FTM1_CH1	- S1A
//  GPIOA PTC6 - GPIO		- S1B
//  GPIOA PTC7 - GPIO		- S1C
//
//  GPIOA PTD0 -  
//  GPIOA PTD1 -  
//  GPIOA PTD2 - EN_1A
//  GPIOA PTD3 - EN_1B
//  GPIOA PTD4 - EN_1C
//  GPIOA PTD5 - SE_2A
//  GPIOA PTD6 - SE_2B
//  GPIOA PTD7 - SE_2C
//
//  GPIOB PTE0 - SCK0
//  GPIOB PTE1 - MOSI0
//  GPIOB PTE2 - MISO0
//  GPIOB PTE3 - CSN0
//  GPIOB PTE4 - 
//  GPIOB PTE5 - S2A
//  GPIOB PTE6 - S2B
//  GPIOB PTE7 - S2C
//
//  GPIOB PTF0 - 
//  GPIOB PTF1 - EN_2A
//  GPIOB PTF2 - EN_2B
//  GPIOB PTF3 - EN_2C
//  GPIOB PTF4 - 
//  GPIOB PTF5 - ADC_SE13 - V_2A
//  GPIOB PTF6 - ADC_SE14 - V_2B
//  GPIOB PTF7 - ADC_SE15 - LS_TEMP_AD
//
//  GPIOB PTG0 - SE_1A
//  GPIOB PTG1 - SE_1B
//  GPIOB PTG2 - SE_1C
//  GPIOB PTG3 - CE0
//  GPIOB PTG4 - NC
//  GPIOB PTG5 - NC
//  GPIOB PTG6 - NC
//  GPIOB PTG7 - NC
//
//  GPIOB PTH0 - Buzzer_Out
//  GPIOB PTH1 - LED1_OUT
//  GPIOB PTH2 - IRQ0/BUSOUT
//  GPIOB PTH3 - NC
//  GPIOB PTH4 - NC
//  GPIOB PTH5 - NC
//  GPIOB PTH6 - NC
//  GPIOB PTH7 - NC

#define LED_PIN_OFF()		GPIO_PDD_ClearPortDataOutputMask(GPIOB_DEVICE,	GPIO_PDD_PIN_25) //PTH1
#define LED_PIN_ON()		GPIO_PDD_SetPortDataOutputMask(GPIOB_DEVICE,	GPIO_PDD_PIN_25) //PTH1
#define ALARM_PIN_OFF()		GPIO_PDD_ClearPortDataOutputMask(GPIOB_DEVICE,	GPIO_PDD_PIN_24) //PTH0
#define ALARM_PIN_ON()		GPIO_PDD_SetPortDataOutputMask(GPIOB_DEVICE,	GPIO_PDD_PIN_24) //PTH0

#define SE_1A_PIN_ON()			GPIO_PDD_SetPortDataOutputMask(GPIOB_DEVICE, GPIO_PDD_PIN_16) //PTG0
#define SE_1B_PIN_ON()			GPIO_PDD_SetPortDataOutputMask(GPIOB_DEVICE, GPIO_PDD_PIN_17) //PTG1
#define SE_1C_PIN_ON()			GPIO_PDD_SetPortDataOutputMask(GPIOB_DEVICE, GPIO_PDD_PIN_18) //PTG2
#define SE_1A_PIN_OFF()			GPIO_PDD_ClearPortDataOutputMask(GPIOB_DEVICE, GPIO_PDD_PIN_16) //PTG0
#define SE_1B_PIN_OFF() 		GPIO_PDD_ClearPortDataOutputMask(GPIOB_DEVICE, GPIO_PDD_PIN_17) //PTG1
#define SE_1C_PIN_OFF()			GPIO_PDD_ClearPortDataOutputMask(GPIOB_DEVICE, GPIO_PDD_PIN_18) //PTG2
#define SE_1_ALL_PIN_ON() 		GPIO_PDD_SetPortDataOutput(GPIOB_DEVICE, GPIO_PDD_GetPortDataOutput(GPIOB_DEVICE) | (MASK_LSB_3<<16))
#define SE_1_ALL_PIN_OFF() 		GPIO_PDD_SetPortDataOutput(GPIOB_DEVICE, GPIO_PDD_GetPortDataOutput(GPIOB_DEVICE) & ~(MASK_LSB_3<<16))
#define SE_1_ALL_PIN_SET(val) 	GPIO_PDD_SetPortDataOutput(GPIOB_DEVICE, (GPIO_PDD_GetPortDataOutput(GPIOB_DEVICE) & ~((val&MASK_LSB_3)<<16)) | ((val&MASK_LSB_3)<<16))

#define SE_2A_PIN_ON()			GPIO_PDD_SetPortDataOutputMask(GPIOA_DEVICE, GPIO_PDD_PIN_29) //PTD5
#define SE_2B_PIN_ON()			GPIO_PDD_SetPortDataOutputMask(GPIOA_DEVICE, GPIO_PDD_PIN_30) //PTD6
#define SE_2C_PIN_ON()			GPIO_PDD_SetPortDataOutputMask(GPIOA_DEVICE, GPIO_PDD_PIN_31) //PTD7
#define SE_2A_PIN_OFF()			GPIO_PDD_ClearPortDataOutputMask(GPIOA_DEVICE, GPIO_PDD_PIN_29) //PTD5
#define SE_2B_PIN_OFF() 		GPIO_PDD_ClearPortDataOutputMask(GPIOA_DEVICE, GPIO_PDD_PIN_30) //PTD6
#define SE_2C_PIN_OFF()			GPIO_PDD_ClearPortDataOutputMask(GPIOA_DEVICE, GPIO_PDD_PIN_31) //PTD7
#define SE_2_ALL_PIN_ON() 		GPIO_PDD_SetPortDataOutput(GPIOA_DEVICE, GPIO_PDD_GetPortDataOutput(GPIOA_DEVICE) | (MASK_LSB_3<<29))
#define SE_2_ALL_PIN_OFF() 		GPIO_PDD_SetPortDataOutput(GPIOA_DEVICE, GPIO_PDD_GetPortDataOutput(GPIOA_DEVICE) & ~(MASK_LSB_3<<29))

#define EN_1A_PIN_ON()		GPIO_PDD_SetPortDataOutputMask(GPIOA_DEVICE, GPIO_PDD_PIN_26) 	//PTD2
#define EN_1B_PIN_ON() 		GPIO_PDD_SetPortDataOutputMask(GPIOA_DEVICE, GPIO_PDD_PIN_27) 	//PTD3
#define EN_1C_PIN_ON()		GPIO_PDD_SetPortDataOutputMask(GPIOA_DEVICE, GPIO_PDD_PIN_28) 	//PTD4
#define EN_1A_PIN_OFF()		GPIO_PDD_ClearPortDataOutputMask(GPIOA_DEVICE, GPIO_PDD_PIN_26) //PTD2
#define EN_1B_PIN_OFF() 	GPIO_PDD_ClearPortDataOutputMask(GPIOA_DEVICE, GPIO_PDD_PIN_27) //PTD3
#define EN_1C_PIN_OFF()		GPIO_PDD_ClearPortDataOutputMask(GPIOA_DEVICE, GPIO_PDD_PIN_28) //PTD4
#define EN_1A_PIN_SET(en)	((en) ? EN_1A_PIN_ON() : EN_1A_PIN_OFF())
#define EN_1B_PIN_SET(en)	((en) ? EN_1B_PIN_ON() : EN_1B_PIN_OFF())
#define EN_1C_PIN_SET(en)	((en) ? EN_1C_PIN_ON() : EN_1C_PIN_OFF())
#define EN_1_ALL_PIN_ON() 		GPIO_PDD_SetPortDataOutput(GPIOA_DEVICE, GPIO_PDD_GetPortDataOutput(GPIOA_DEVICE) | (MASK_LSB_3<<26))
#define EN_1_ALL_PIN_OFF() 		GPIO_PDD_SetPortDataOutput(GPIOA_DEVICE, GPIO_PDD_GetPortDataOutput(GPIOA_DEVICE) & ~(MASK_LSB_3<<26))
#define EN_1_ALL_PIN_SET(val)	GPIO_PDD_SetPortDataOutput(GPIOA_DEVICE, (GPIO_PDD_GetPortDataOutput(GPIOA_DEVICE) & ~((val&MASK_LSB_3)<<26)) | ((val&MASK_LSB_3)<<26))

#define EN_2A_PIN_ON()		GPIO_PDD_SetPortDataOutputMask(GPIOB_DEVICE, GPIO_PDD_PIN_9)	//PTF1
#define EN_2B_PIN_ON() 		GPIO_PDD_SetPortDataOutputMask(GPIOB_DEVICE, GPIO_PDD_PIN_10) 	//PTF2
#define EN_2C_PIN_ON()		GPIO_PDD_SetPortDataOutputMask(GPIOB_DEVICE, GPIO_PDD_PIN_11) 	//PTF3
#define EN_2A_PIN_OFF()		GPIO_PDD_ClearPortDataOutputMask(GPIOB_DEVICE, GPIO_PDD_PIN_9) 	//PTF1
#define EN_2B_PIN_OFF() 	GPIO_PDD_ClearPortDataOutputMask(GPIOB_DEVICE, GPIO_PDD_PIN_10) //PTF2
#define EN_2C_PIN_OFF()		GPIO_PDD_ClearPortDataOutputMask(GPIOB_DEVICE, GPIO_PDD_PIN_11) //PTF3
#define EN_2_ALL_PIN_ON() 	GPIO_PDD_SetPortDataOutput(GPIOB_DEVICE, GPIO_PDD_GetPortDataOutput(GPIOB_DEVICE) | (MASK_LSB_3<<9))
#define EN_2_ALL_PIN_OFF() 	GPIO_PDD_SetPortDataOutput(GPIOB_DEVICE, GPIO_PDD_GetPortDataOutput(GPIOB_DEVICE) & ~(MASK_LSB_3<<9))


//  GPIOA PTC5 - S1A
//  GPIOA PTC6 - S1B 
//  GPIOA PTC7 - S1C

 //bit order 00000CAB
#define S1_ALL_PIN_READ() ((GPIO_PDD_GetPortDataInput(GPIOA_BASE_PTR) >> 21) & MASK_LSB_3) //bit order 00000CBA in schematic, physically CAB?
#define S2_ALL_PIN_READ() ((GPIO_PDD_GetPortDataInput(GPIOB_BASE_PTR) >> 5)	& MASK_LSB_3) //bit order 00000CBA


//#define PWM_CV_PHASE_A 	FTM2_C5V
//#define PWM_CV_PHASE_B 	FTM2_C4V
//#define PWM_CV_PHASE_C 	FTM2_C3V
//#define PWM_CV_PHASE_A_SET(val) (*FTM2_C5V = val)
//#define PWM_CV_PHASE_B_SET(val) (*FTM2_C4V = val)
//#define PWM_CV_PHASE_C_SET(val) (*FTM2_C3V = val)

#define PWM_1A_PIN_SET_CV(reg)	FTM_PDD_WriteChannelValueReg(FTM2_DEVICE, FTM_PDD_CHANNEL_5, reg) //PTB5 FTM2_C5V
#define PWM_1B_PIN_SET_CV(reg)	FTM_PDD_WriteChannelValueReg(FTM2_DEVICE, FTM_PDD_CHANNEL_4, reg) //PTB4 FTM2_C4V
#define PWM_1C_PIN_SET_CV(reg)	FTM_PDD_WriteChannelValueReg(FTM2_DEVICE, FTM_PDD_CHANNEL_3, reg) //PTC3 FTM2_C3V

#define PWM_2A_PIN_SET_CV(reg)	FTM_PDD_WriteChannelValueReg(FTM2_DEVICE, FTM_PDD_CHANNEL_2, reg) //PTC2 FTM2_C2V
#define PWM_2B_PIN_SET_CV(reg)	FTM_PDD_WriteChannelValueReg(FTM2_DEVICE, FTM_PDD_CHANNEL_1, reg) //PTC1 FTM2_C1V
#define PWM_2C_PIN_SET_CV(reg)	FTM_PDD_WriteChannelValueReg(FTM2_DEVICE, FTM_PDD_CHANNEL_0, reg) //PTC0 FTM2_C0V

//CLEARS PREVIOUS SETTING
//1 sets top MOSFET on, bottom off, ignores PWM
//0 sets bottom MOSFET on, top off, ignores PWM
#define PWM_1A_PIN_SET_SWO(bit)		(*FTM2_SWOCTRL = FTM_SWOCTRL_CH5OC_MASK | ((bit<<FTM_SWOCTRL_CH5OCV_SHIFT)&FTM_SWOCTRL_CH5OCV_MASK))
#define PWM_1B_PIN_SET_SWO(bit)		(*FTM2_SWOCTRL = FTM_SWOCTRL_CH4OC_MASK | ((bit<<FTM_SWOCTRL_CH4OCV_SHIFT)&FTM_SWOCTRL_CH4OCV_MASK))
#define PWM_1C_PIN_SET_SWO(bit)		(*FTM2_SWOCTRL = FTM_SWOCTRL_CH3OC_MASK | ((bit<<FTM_SWOCTRL_CH3OCV_SHIFT)&FTM_SWOCTRL_CH3OCV_MASK))
#define PWM_1_ALL_PIN_CLEAR_SWO()	(*FTM2_SWOCTRL = 0)

//(*FTM2_SWOCTRL & ~((reg&MASK_LSB_3)<<FTM_SWOCTRL_CH3OC_MASK)) | ((reg&MASK_LSB_3)<<FTM_SWOCTRL_CH3OC_MASK) |
//~((reg&MASK_LSB_3)<<FTM_SWOCTRL_CH3OC_MASK)) | ((reg&MASK_LSB_3)<<FTM_SWOCTRL_CH3OC_MASK)

#define PWM_1_ALL_PIN_SET_SWO(reg)		(*FTM2_SWOCTRL = FTM_SWOCTRL_CH5OC_MASK | ((bit<<FTM_SWOCTRL_CH5OCV_SHIFT)&FTM_SWOCTRL_CH5OCV_MASK))


#define CAPTURE_MOTOR2_HALL_C() (SIM_PINSEL |= SIM_PINSEL_FTM1PS1_MASK)
#define CAPTURE_MOTOR1_HALL_A() (SIM_PINSEL &= ~SIM_PINSEL_FTM1PS1_MASK)

#endif /* BOARD_H_ */
