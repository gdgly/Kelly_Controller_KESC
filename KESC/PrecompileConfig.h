//Precompile Config

#ifndef _PRECOMPILE_CONFIG_H_
#define _PRECOMPILE_CONFIG_H_


#define SHELL_OPTION_USE_LIST
#define CMDLINE_MAX_ARGS 	3
#define CMDLINE_BUF_SIZE	32

#define MENU_FUNCTIONS_COUNT 10
#define STATE_POINTERS_COUNT 2

#define F_CPU 40000000

// ISR Map - Maps to MCUXpresso generated code. Not used for CW

#define ADC_ISR		ADC_IRQHandler
//#define	UARTO_ISR	UART0_IRQHandler
#define	FMSTR_Isr	UART0_IRQHandler

#define SysTick_ISR 			SysTick_Handler
#define FTM2_ISR 				FTM2_IRQHandler
#define FTM1_ISR 				FTM1_IRQHandler

#endif /* KESC_CONFIG_H_ */
