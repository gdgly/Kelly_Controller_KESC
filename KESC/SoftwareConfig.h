//Precompile Config

#ifndef _PRECOMPILE_CONFIG_H_
#define _PRECOMPILE_CONFIG_H_

#define SHELL_OPTION_USE_LIST
#define CMDLINE_MAX_ARGS 	3
#define CMDLINE_BUF_SIZE	32
#define Shell_ReadChar		Term1_ReadChar
#define Shell_SendChar		Term1_SendChar
#define Shell_SendStr		Term1_SendStr
#define Shell_KeyPressed	Term1_KeyPressed

#define MENU_FUNCTIONS_COUNT 10
#define STATE_POINTERS_COUNT 2

#define SYSTICK_MILLIS

#endif /* KESC_CONFIG_H_ */
