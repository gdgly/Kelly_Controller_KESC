#include <KEA64_CPU.h>
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
//#include "MKE02Z4.h"
/* TODO: insert other include files here. */
#include "Cpu.h"
#include "KEA64/KEA64_CPU.h"

/* TODO: insert other definitions and declarations here. */

void SystemInitHook (void) {
	CPU_InitClock40MHz();
}

/*
 * @brief   Application entry point.
 */
int main(void) {
  	/* Init board hardware. */
//    BOARD_InitBootPins();
//    BOARD_InitBootClocks();
//    BOARD_InitBootPeripherals();
//
    //printf("Hello World\n");
	//SystemCoreClockUpdate();

	PE_low_level_init();
	KESC_Init();
	KESC_Loop();

    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}
