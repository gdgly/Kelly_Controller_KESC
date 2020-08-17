#include "Platform/KEA64/ADC.h"

#include "IO_Map.h"
#include "ADC_PDD.h"

#include <stdint.h>
#include <stdbool.h>

void ADC_Init(void)
{
	/* Interrupt vector(s) priority setting */
	// priority level 2 (normal)
	/* NVIC_IPR3: PRI_15=0x80 */
	NVIC_IPR3 = (uint32_t) ((NVIC_IPR3 & (uint32_t) ~(uint32_t) (NVIC_IP_PRI_15(0xFF))) | (uint32_t) (NVIC_IP_PRI_15(0x80)));
	/* NVIC_ISER: SETENA|=0x8000 */
	NVIC_ISER |= NVIC_ISER_SETENA(0x8000);

	SIM_SCGC |= SIM_SCGC_ADC_MASK;

	// Set HW trigger FTM2 counter init
	/* SIM_SOPT: ADHWT=0x02 */
	SIM_SOPT = (uint32_t) ((SIM_SOPT & (uint32_t) ~(uint32_t) (SIM_SOPT_ADHWT(0x03))) | (uint32_t) (SIM_SOPT_ADHWT(0x02)));
	/* ADC_APCTL1: ADPC=0xE037 */
	ADC_APCTL1 = ADC_APCTL1_ADPC(0xE037);
	/* ADC_SC4: ASCANE=0,ACFSEL=0,AFDEP=0 */
	ADC_SC4 = ADC_SC4_AFDEP(0x00) | 0x0100U;
	/* ADC_SC3: ADIV=2,ADLSMP=0,MODE=0,ADICLK=0 */
	ADC_SC3 = (ADC_SC3_ADIV(0x02) | ADC_SC3_MODE(0x00) | ADC_SC3_ADICLK(0x00));
	/* ADC_SC2: ADACT=0,ADTRG=0,ACFE=0,ACFGT=0,FEMPTY=0,FFULL=0,REFSEL=1 */
	ADC_SC2 = ADC_SC2_REFSEL(0x01);
	/* ADC_SC1: COCO=0,AIEN=1,ADCO=1,ADCH=0x1F */
	ADC_SC1 = (ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(0x1F));
}
