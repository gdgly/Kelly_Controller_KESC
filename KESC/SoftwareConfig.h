//Precompile Config

#ifndef _PRECOMPILE_CONFIG_H_
#define _PRECOMPILE_CONFIG_H_


#define SHELL_OPTION_USE_LIST
#define CMDLINE_MAX_ARGS 	3
#define CMDLINE_BUF_SIZE	32
#define Shell_ReadChar		Terminal_ReadChar
#define Shell_SendChar		Terminal_SendChar
#define Shell_SendStr		Terminal_SendStr
#define Shell_KeyPressed	Terminal_KeyPressed

#define MENU_FUNCTIONS_COUNT 10
#define STATE_POINTERS_COUNT 2

#define F_CPU 40000000

#endif /* KESC_CONFIG_H_ */
