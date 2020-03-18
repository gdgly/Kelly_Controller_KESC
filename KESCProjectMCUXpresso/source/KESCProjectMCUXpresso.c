/*
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    KESCProjectMCUXpresso.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
//#include "MKE02Z4.h"
/* TODO: insert other include files here. */
#include "Cpu.h"

/* TODO: insert other definitions and declarations here. */
void PEClockInit(void) {
	/* System clock initialization */
	if ( *((uint8_t*) 0x03FFU) != 0xFFU) {
	  ICS_C3 = *((uint8_t*) 0x03FFU);
	  ICS_C4 = (ICS_C4 & 0xFEU) | ((*((uint8_t*) 0x03FEU)) & 0x01U);
	}
	/* ICS_C2: BDIV|=1 */
	ICS_C2 |= ICS_C2_BDIV(0x01);         /* Update system prescalers */
	/* SIM_BUSDIV: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,BUSDIV=0 */
	SIM_BUSDIV = 0x00U;                  /* Update system prescalers */
	/* Switch to FEI Mode */
	/* ICS_C1: CLKS=0,RDIV=0,IREFS=1,IRCLKEN=1,IREFSTEN=1 */
	ICS_C1 = ICS_C1_CLKS(0x00) |
			 ICS_C1_RDIV(0x00) |
			 ICS_C1_IREFS_MASK |
			 ICS_C1_IRCLKEN_MASK |
			 ICS_C1_IREFSTEN_MASK;
	/* ICS_C2: BDIV=1,LP=0 */
	ICS_C2 = (uint8_t)((ICS_C2 & (uint8_t)~(uint8_t)(
			  ICS_C2_BDIV(0x06) |
			  ICS_C2_LP_MASK
			 )) | (uint8_t)(
			  ICS_C2_BDIV(0x01)
			 ));
	/* OSC_CR: OSCEN=0,??=0,OSCSTEN=1,OSCOS=0,??=0,RANGE=0,HGO=0,OSCINIT=0 */
	OSC_CR = OSC_CR_OSCSTEN_MASK;
	while((ICS_S & ICS_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
	}
	while((ICS_S & 0x0CU) != 0x00U) {    /* Wait until output of the FLL is selected */
	}
	/*** End of PE initialization code after reset ***/

	/*** User code after PE initialization and before C initialization ***/
	// Change ICS to 40MHz
	// If user wants to use 40 MHz core clock , this bit must be set to 1 before changing ICS_C2[BDIV]=0x000.
	/* SIM_BUSDIV: BUSDIV=1 */
	SIM_BUSDIV = 0x01U;
	// NOTE: If user want to use 40 MHz core clock, this bit need clear to 000b after SIM_BUSDIV[BUSDIV]=1.
	/* ICS_C2: BDIV=0 */
	ICS_C2 = (uint8_t)(ICS_C2 & (uint8_t)~(uint8_t)(ICS_C2_BDIV_MASK));
	/*** End of user code after PE initialization and before C initialization ***/
}

void SystemInitHook (void) {
	PEClockInit();
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
	//PEClockInit();
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
