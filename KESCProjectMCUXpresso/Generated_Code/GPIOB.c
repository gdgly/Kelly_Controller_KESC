/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : GPIOB.c
**     Project     : KESCProjectMCUXpresso
**     Processor   : SKEAZN64MLH2
**     Component   : Init_GPIO
**     Version     : Component 01.006, Driver 01.06, CPU db: 3.00.000
**     Repository  : Kinetis
**     Compiler    : GNU C Compiler
**     Date/Time   : 2019-12-12, 18:30, # CodeGen: 0
**     Abstract    :
**          This file implements the GPIO (GPIOB) module initialization
**          according to the Peripheral Initialization settings, and
**          defines interrupt service routines prototypes.
**     Settings    :
**          Component name                                 : GPIOB
**          Device                                         : GPIOB
**          Settings                                       : 
**            Pin 0                                        : Do not initialize
**            Pin 1                                        : Do not initialize
**            Pin 2                                        : Do not initialize
**            Pin 3                                        : Do not initialize
**            Pin 4                                        : Do not initialize
**            Pin 5                                        : Do not initialize
**            Pin 6                                        : Do not initialize
**            Pin 7                                        : Do not initialize
**            Pin 8                                        : Do not initialize
**            Pin 9                                        : Do not initialize
**            Pin 10                                       : Do not initialize
**            Pin 11                                       : Do not initialize
**            Pin 12                                       : Do not initialize
**            Pin 13                                       : Do not initialize
**            Pin 14                                       : Do not initialize
**            Pin 15                                       : Do not initialize
**            Pin 16                                       : Do not initialize
**            Pin 17                                       : Do not initialize
**            Pin 18                                       : Do not initialize
**            Pin 19                                       : Do not initialize
**            Pin 24                                       : Do not initialize
**            Pin 25                                       : Initialize
**              Pin direction                              : Output
**              Pin input                                  : Disabled
**              Output value                               : No initialization
**              Pull resistor                              : No initialization
**            Pin 26                                       : Do not initialize
**            Pin 30                                       : Do not initialize
**            Pin 31                                       : Do not initialize
**          Pin selection/routing                          : 
**            Pin 0                                        : Disabled
**            Pin 1                                        : Disabled
**            Pin 2                                        : Disabled
**            Pin 3                                        : Disabled
**            Pin 4                                        : Disabled
**            Pin 5                                        : Disabled
**            Pin 6                                        : Disabled
**            Pin 7                                        : Disabled
**            Pin 8                                        : Disabled
**            Pin 9                                        : Disabled
**            Pin 10                                       : Disabled
**            Pin 11                                       : Disabled
**            Pin 12                                       : Disabled
**            Pin 13                                       : Disabled
**            Pin 14                                       : Disabled
**            Pin 15                                       : Disabled
**            Pin 16                                       : Disabled
**            Pin 17                                       : Disabled
**            Pin 18                                       : Disabled
**            Pin 19                                       : Disabled
**            Pin 24                                       : Disabled
**            Pin 25                                       : Enabled
**              Pin                                        : PTH1/FTM2_CH1
**              Pin signal                                 : LED1_OUT
**            Pin 26                                       : Disabled
**            Pin 30                                       : Disabled
**            Pin 31                                       : Disabled
**          Initialization                                 : 
**            Call Init method                             : yes
**     Contents    :
**         Init - void GPIOB_Init(void);
**
**     Copyright : 1997 - 2015 Freescale Semiconductor, Inc. 
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
** @file GPIOB.c
** @version 01.06
** @brief
**          This file implements the GPIO (GPIOB) module initialization
**          according to the Peripheral Initialization settings, and
**          defines interrupt service routines prototypes.
*/         
/*!
**  @addtogroup GPIOB_module GPIOB module documentation
**  @{
*/         

/* MODULE GPIOB. */

#include "GPIOB.h"
  /* Including shared modules, which are used in the whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "Cpu.h"


/*
** ===================================================================
**     Method      :  GPIOB_Init (component Init_GPIO)
**     Description :
**         This method initializes registers of the GPIO module
**         according to the Peripheral Initialization settings.
**         Call this method in user code to initialize the module. By
**         default, the method is called by PE automatically; see "Call
**         Init method" property of the component for more details.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void GPIOB_Init(void)
{
  /* GPIOB_PDDR: PDD|=0x02000000 */
  GPIOB_PDDR |= GPIO_PDDR_PDD(0x02000000);
  /* GPIOB_PIDR: PID|=0x02000000 */
  GPIOB_PIDR |= GPIO_PIDR_PID(0x02000000);
}


/* END GPIOB. */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
