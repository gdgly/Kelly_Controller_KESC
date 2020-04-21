/*
 * Parameters.h
 *
 *  Created on: Dec 5, 2019
 *      Author: FireSourcery
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <stdint.h>

#define MOTOR1_POLE_PAIRS	10 // read from eeprom
#define MAX_RPM 5000


//#define ETS_MAP_HALL_PHASE_AB		47
//#define ETS_MAP_HALL_PHASE_AB		47
//#define ETS_MAP_HALL_PHASE_AB		47
//#define ETS_MAP_HALL_PHASE_AB		47
//#define ETS_MAP_HALL_PHASE_AB		47
//#define ETS_MAP_HALL_PHASE_AB		47

typedef struct Parameters_MemMap 
{
	union
	{
		uint8_t Index[256];
		struct
		{
			uint8_t CommuntationIndexAB;
			uint8_t CommuntationIndexAC;
			uint8_t CommuntationIndexBC;
			uint8_t CommuntationIndexBA;
			uint8_t CommuntationIndexCA;
			uint8_t CommuntationIndexCB;
		};
	};
} 
volatile *Parameters_MemMapPtr;

#define PARAMETERS_BASE_PTR		((Parameters_MemMapPtr)0x10000000u) //eeprom base

void dd (void)
{
	PARAMETERS_BASE_PTR[20];
}

#endif /* PARAMETERS_H_ */
