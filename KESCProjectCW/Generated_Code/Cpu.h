/* ###################################################################
**     THIS COMPONENT MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename    : Cpu.h
**     Project     : KESCProjectCW
**     Processor   : SKEAZN64MLH2
**     Component   : MKE02Z64QH2
**     Version     : Component 01.044, Driver 01.04, CPU db: 3.00.000
**     Datasheet   : MKE02Z64M20SF0RM, Rev.2.1, Apr-23 2013; KEAZ64RM, Rev.1, Sep 2013
**     Compiler    : GNU C Compiler
**     Date/Time   : 2019-12-08, 19:02, # CodeGen: 0
**     Abstract    :
**
**     Comment     :
**         ICS scaler changed to 1
**     Settings    :
**
**     Contents    :
**         SetClockConfiguration - LDD_TError Cpu_SetClockConfiguration(LDD_TClockConfiguration ModeID);
**         GetClockConfiguration - LDD_TClockConfiguration Cpu_GetClockConfiguration(void);
**         EnableInt             - void Cpu_EnableInt(void);
**         DisableInt            - void Cpu_DisableInt(void);
**         SystemReset           - void Cpu_SystemReset(void);
**
**     (c) Freescale Semiconductor, Inc.
**     2004 All Rights Reserved
**
**     Copyright : 1997 - 2014 Freescale Semiconductor, Inc. 
**     All Rights Reserved.
**     
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**     
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**     
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**     
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**     
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**     
**     http: www.freescale.com
**     mail: support@freescale.com
** ###################################################################*/
/*!
** @file Cpu.h
** @version 01.04
** @brief
**
*/         
/*!
**  @addtogroup Cpu_module Cpu module documentation
**  @{
*/         

#ifndef __Cpu_H
#define __Cpu_H

/* MODULE Cpu. */
/*Include shared modules, which are used for whole project*/
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Active configuration define symbol */
#define PEcfg_FLASH 1U

/* Methods configuration constants - generated for all enabled component's methods */
#define Cpu_SetClockConfiguration_METHOD_ENABLED
#define Cpu_GetClockConfiguration_METHOD_ENABLED
#define Cpu_EnableInt_METHOD_ENABLED
#define Cpu_DisableInt_METHOD_ENABLED
#define Cpu_SystemReset_METHOD_ENABLED

/* Events configuration constants - generated for all enabled component's events */
#define Cpu_OnReset_EVENT_ENABLED
#define Cpu_OnNMIINT_EVENT_ENABLED

#define CPU_BUS_CLK_HZ                  20000000U /* Initial value of the bus clock frequency in Hz */
#define CPU_CORE_CLK_HZ                 20000000U /* Initial value of the core/system clock frequency in Hz.  */

#define CPU_CLOCK_CONFIG_NUMBER         0x01U /* Specifies number of defined clock configurations. */

#define CPU_BUS_CLK_HZ_CLOCK_CONFIG0    20000000U /* Value of the bus clock frequency in the clock configuration 0 in Hz. */
#define CPU_CORE_CLK_HZ_CLOCK_CONFIG0   20000000U /* Value of the core/system clock frequency in the clock configuration 0 in Hz. */


#define CPU_INT_SLOW_CLK_HZ             39062U /* Value of the slow internal oscillator clock frequency in Hz  */

#define CPU_FAMILY_Kinetis             /* Specification of the core type of the selected cpu */
#define CPU_DERIVATIVE_MKE02Z64QH2     /* Name of the selected cpu derivative */
#define CPU_PARTNUM_SKEAZN64MLH2       /* Part number of the selected cpu */
#define CPU_LITTLE_ENDIAN              /* The selected cpu uses little endian */

/* CPU frequencies in clock configuration 0 */
#define CPU_CLOCK_CONFIG_0              0x00U /* Clock configuration 0 identifier */
#define CPU_CORE_CLK_HZ_CONFIG_0        20000000UL /* Core clock frequency in clock configuration 0 */
#define CPU_BUS_CLK_HZ_CONFIG_0         20000000UL /* Bus clock frequency in clock configuration 0 */
#define CPU_FLEXBUS_CLK_HZ_CONFIG_0     0UL /* Flexbus clock frequency in clock configuration 0 */
#define CPU_FLASH_CLK_HZ_CONFIG_0       20000000UL /* FLASH clock frequency in clock configuration 0 */
#define CPU_USB_CLK_HZ_CONFIG_0         0UL /* USB clock frequency in clock configuration 0 */
#define CPU_PLL_FLL_CLK_HZ_CONFIG_0     40000000UL /* PLL/FLL clock frequency in clock configuration 0 */
#define CPU_MCGIR_CLK_HZ_CONFIG_0       39062UL /* MCG internal reference clock frequency in clock configuration 0 */
#define CPU_OSCER_CLK_HZ_CONFIG_0       0UL /* System OSC external reference clock frequency in clock configuration 0 */
#define CPU_ERCLK32K_CLK_HZ_CONFIG_0    0UL /* External reference clock 32k frequency in clock configuration 0 */
#define CPU_MCGFF_CLK_HZ_CONFIG_0       39062UL /* MCG fixed frequency clock */


typedef struct  {
  uint32_t cpu_core_clk_hz;            /* Core clock frequency in clock configuration */
  uint32_t cpu_bus_clk_hz;             /* Bus clock frequency in clock configuration */
  uint32_t cpu_flexbus_clk_hz;         /* Flexbus clock frequency in clock configuration */
  uint32_t cpu_flash_clk_hz;           /* FLASH clock frequency in clock configuration */
  uint32_t cpu_usb_clk_hz;             /* USB clock frequency in clock configuration */
  uint32_t cpu_pll_fll_clk_hz;         /* PLL/FLL clock frequency in clock configuration */
  uint32_t cpu_mcgir_clk_hz;           /* MCG internal reference clock frequency in clock configuration */
  uint32_t cpu_oscer_clk_hz;           /* System OSC external reference clock frequency in clock configuration */
  uint32_t cpu_erclk32k_clk_hz;        /* External reference clock 32k frequency in clock configuration */
  uint32_t cpu_mcgff_clk_hz;           /* MCG fixed frequency clock */
} TCpuClockConfiguration;

/* The array of clock frequencies in configured clock configurations */
extern const TCpuClockConfiguration PE_CpuClockConfigurations[CPU_CLOCK_CONFIG_NUMBER];

  /* Interrupt vector table type definition */
  typedef void (*const tIsrFunc)(void);
  typedef struct {
    void * __ptr;
    tIsrFunc __fun[0x2F];
  } tVectorTable;
  
  extern const tVectorTable __vect_table;

/* Global variables */
/*lint -esym(765,SR_reg) Disable MISRA rule (8.10) checking for symbols (SR_reg). The SR_reg is used in inline assembler. */
extern volatile uint8_t SR_reg;        /* Current FAULTMASK register */
/*lint -esym(765,SR_lock) Disable MISRA rule (8.10) checking for symbols (SR_lock). The SR_reg is used in inline assembler. */
extern volatile uint8_t SR_lock;


/*
** ===================================================================
**     Method      :  Cpu_SystemReset (component MKE02Z64QH2)
*/
/*!
**     @brief
**         This method initiates a system reset request to reset the
**         CPU.
*/
/* ===================================================================*/
void Cpu_SystemReset(void);

/*
** ===================================================================
**     Method      :  Cpu_SetClockConfiguration (component MKE02Z64QH2)
*/
/*!
**     @brief
**         Calling of this method will cause the clock configuration
**         change and reconfiguration of all components according to
**         the requested clock configuration setting.
**     @param
**         ModeID          - Clock configuration identifier
**     @return
**                         - ERR_OK - OK.
**                           ERR_RANGE - Mode parameter out of range
*/
/* ===================================================================*/
LDD_TError Cpu_SetClockConfiguration(LDD_TClockConfiguration ModeID);

/*
** ===================================================================
**     Method      :  Cpu_GetClockConfiguration (component MKE02Z64QH2)
*/
/*!
**     @brief
**         Returns the active clock configuration identifier. The
**         method is automatically enabled if more than one clock
**         configuration is enabled in the component.
**     @return
**                         - Active clock configuration identifier
*/
/* ===================================================================*/
LDD_TClockConfiguration Cpu_GetClockConfiguration(void);

/*
** ===================================================================
**     Method      :  Cpu_EnableInt (component MKE02Z64QH2)
*/
/*!
**     @brief
**         Enables all maskable interrupts.
*/
/* ===================================================================*/
void Cpu_EnableInt(void);

/*
** ===================================================================
**     Method      :  Cpu_DisableInt (component MKE02Z64QH2)
*/
/*!
**     @brief
**         Disables all maskable interrupts.
*/
/* ===================================================================*/
void Cpu_DisableInt(void);

/*
** ===================================================================
**     Method      :  PE_low_level_init (component MKE02Z64QH2)
**
**     Description :
**         Initializes beans and provides common register initialization. 
**         The method is called automatically as a part of the 
**         application initialization code.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
void PE_low_level_init(void);

/* {Default RTOS Adapter} ISR function prototype */
PE_ISR(Cpu_INT_NMIInterrupt);
/*
** ===================================================================
**     Method      :  Cpu_INT_NMIInterrupt (component MKE02Z64QH2)
**
**     Description :
**         This ISR services the Non Maskable Interrupt interrupt.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

PE_ISR(Cpu_Interrupt);
/*
** ===================================================================
**     Method      :  Cpu_Cpu_Interrupt (component MKE02Z64QH2)
**
**     Description :
**         This ISR services an unused interrupt/exception vector.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

void __init_hardware(void);
/*
** ===================================================================
**     Method      :  __init_hardware (component MKE02Z64QH2)
**
**     Description :
**         Initializes the whole system like timing, external bus, etc.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

/* END Cpu. */

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif
/* __Cpu_H */

/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.3 [05.09]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
