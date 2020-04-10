/*
 * KESC_V0.c
 *
 *  Created on: Oct 11, 2019
 *      Author: SLi
 */

#include "KESC.h"

#include "Board.h"
#include "Parameters.h"
#include "Shell.h"

#include "BLDC.h"
#include "Commutation.h"
#include "Measure.h"

#include "VoltageDivider.h"
#include "PID.h"
#include "Millis.h"
#include "OS.h"
#include "Blinky.h"

#include "ADC.h"
//#include "ADC0.h"

#include "Cpu.h"
#include "GPIOA.h"
#include "GPIOB.h"
#include "FTM1.h"
//#include "FTM0.h"
#include "FTM2.h"
//#include "WDOG.h"
#include "Cpu.h"
#include "Events.h"
#include "GPIOA.h"
#include "GPIOB.h"
#include "FTM2.h"
#include "SysTick.h"
#include "ADC.h"
#include "Term1.h"
#include "Inhr1.h"
#include "ASerialLdd1.h"

#include "FTM_PDD.h"

/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef enum
{
	KESC_STATE_RUN,
	KESC_STATE_ETS,
} KESC_STATE_ID_T;

//typedef struct
//{
//	KESC_STATE_ID_T State;
//	void (*Function)(void);
//} KESC_STATE_T;

typedef enum
{
	KESC_DIRECTION_FORWARD,
	KESC_DIRECTION_REVERSE,
} KESC_DIRECTION_T;

volatile static KESC_DIRECTION_T Direction; //direction of travel

//volatile union
//{
//	uint16_t Reg;
//	struct
//	{
//		uint16_t E0				:1;
//		uint16_t OverVolt		:1;
//		uint16_t LowVolt		:1;
//		uint16_t TempWarn		:1;
//		uint16_t Startup		:1;
//		uint16_t E5				:1;
//		uint16_t OverTemp		:1;
//		uint16_t StartupTPS		:1;
//		uint16_t Reverse		:1;
//		uint16_t ILOP			:1;
//		uint16_t Identify1		:1;
//		uint16_t Identify2		:1;
//		uint16_t RegenOverVolt	:1;
//		uint16_t HallSensor1	:1;
//		uint16_t HallSensor2	:1;
//		uint16_t Com			:1;
//	};
//} ErrorFlags = {0};

BLDC_CONTROLLER_T	Motor1;
BLINKY_T 			LEDPowerButton;

LITE_FX_OS_THREAD_T ThreadLED;
LITE_FX_OS_THREAD_T ThreadSerial;
LITE_FX_OS_THREAD_T ThreadTask1Second;
LITE_FX_OS_THREAD_T ThreadComTx;	//20ms period task for communications
//LITE_FX_OS_THREAD_T ThreadSample;	//20ms period task for communications

//R1 = 47.5k, R2 = 5.62k, DIV = 5.62/(47.5 + 5.62) = 281/2656
//VREF = 5V, VDIV_PER_ADC = VREF/255 = 5/255
//ADC = VOUT/VDIV_PER_ADC, VBAT = VOUT*DIV
//VBAT_PER_ADC = VDIV_PER_ADC/DIV = 5/256/(5.62/(47.5 + 5.62)) = 415/2248

//Max Voltage 42V:
//      VOUT = 42*DIV = 4.444V
//      ADC = VOUT/VDIV_PER_ADC = 227
//      VBAT_PER_ADC = VBAT/ADC = 42/227 ~= 10/54
//Min Voltage 30V:
//      VOUT = 30V*DIV = 3.174V
//      ADC = VOUT/VDIV_PER_ADC = 162
#define R1_RATIO 	4750
#define R2_RATIO 	562
#define VREF		5
#define ADC_MAX		255

VOLTAGE_DIVIDER_T 		DividerBattery;


// 1 POLE_PAIR = 6 STEPS = ELECTRIC_R
// 60 Steps = 1 MECH_ROTATION:
// 60 Steps / 6 Steps = 10 PolePairs

#define CPU_FREQ 			40000000
#define BUS_FREQ 			20000000

#define FTM1_BUS_PEROID		(64)
#define FTM1_FREQ 			(BUS_FREQ/FTM1_BUS_PEROID)
#define FTM1_CV_MAX			(0xFFFF)

static bool MotorSelect;

//Motor1 Hall A rising edge trigger
void FTM1_ISR(void)
{
	FTM1_C1SC &= ~FTM_CnSC_CHF_MASK;	//FTM_PDD_ClearChannelInterruptFlag(FTM1_DEVICE, FTM_PDD_CHANNEL_1);

//	BLDC_Commutation_ISR(&Motor1.Commutation, Motor1.PWM); //commutate 1/3 times here?

	Speed_CaptureDeltaISR(&Motor1.Speed);

//	if (motorSel)	Speed_Capture(&Motor1.Speed);
//	else			Speed_Capture(&Motor2.Speed);
//	if (count < 1)
//	{
//		count++;
//	}
//	else
//	{
//		count = 0;
//		motorSel = !motorSel;
//		reset timer
//	}
}

#define FTM2_BUS_PEROID	 	(1)
#define FTM2_FREQ 			(BUS_FREQ/FTM2_BUS_PEROID)
#define FTM2_MODULO 		(512)
#define PWM_MAX 			(FTM2_MODULO)
#define PWM_PEROID			(FTM2_MODULO*2) // FTM2 ticks, Modulo * 2 for center aligned PWM
#define PWM_FREQ			(FTM2_FREQ/PWM_PEROID)

//event period to pwm percentage
//PWM_COUNTS per hall cycle =  FTM1_CV * 64 / 1024

//FTM2 PWM Cycle
void FTM2_ISR(void)
{
	FTM2_SC &= ~FTM_SC_TOF_MASK;	//FTM_PDD_ClearOverflowInterruptFlag(FTM2_DEVICE);

	if (Motor1.State == MOTOR_STATE_RUN) BLDC_Commutation_Poll(&Motor1.Commutation, Motor1.PWM);


	//Optionally measure Back EMF at beginning of PWM cycle. (Output is low)
	//Triggered measure will occur again at center of pulse
	//StartADCPhaseA();
	//StartADCPhaseB();
	//StartADCPhaseC();

	//PID Process
	//speed 0-256

	//	Output = Motor1.PWM * something;
	//
	//	DesiredEventPeroid = Speed * something;

	//PIDOutput is in event period time
	//DesiredEventPeroid = Speed * something;
	//Motor1*PWM_MAX*speed>>8

	//PIDOutput*PWM_MAX*speed>>8
	//if(PID_ComputeTimerISR(&PIDMotor1)) BLDC_SetPWM(&Motor1, );

//	if(PID_ProcessTimerISR(&PIDMotor2)) BLDC_SetPWM(&Motor2, 12);
}


/******************************************************************************/
/*!
 * @name  	MeasureADCSection
 * @brief 	ADC Config
 *
 *
 */
/******************************************************************************/
/*! @{ */
#define ADC_CHANNEL_COUNT			8u

#define ADC_CHANNEL_PIN_BAT_AD		ADC_PDD_SINGLE_ENDED_DAD0
#define ADC_CHANNEL_PIN_I1_AD		ADC_PDD_SINGLE_ENDED_AD4
#define ADC_CHANNEL_PIN_I2_AD		ADC_PDD_SINGLE_ENDED_AD5
#define ADC_CHANNEL_PIN_V1A_AD 		ADC_PDD_SINGLE_ENDED_DAD1
#define ADC_CHANNEL_PIN_V1B_AD 		ADC_PDD_SINGLE_ENDED_DAD2
#define ADC_CHANNEL_PIN_V2A_AD 		ADC_PDD_SINGLE_ENDED_AD13
#define ADC_CHANNEL_PIN_V2B_AD 		ADC_PDD_SINGLE_ENDED_AD14
#define ADC_CHANNEL_PIN_LSTEMP_AD 	ADC_PDD_SINGLE_ENDED_AD15

//	Virtual/Measure Channel, index into arrays
#define ADC_BAT_AD		0U
#define ADC_I1_AD		1U
#define ADC_I2_AD		2U
#define ADC_V1A			3U
#define ADC_V1B			4U
#define ADC_V2A			5U
#define ADC_V2B			6U
#define ADC_LSTEMP_AD	7U

const uint8_t CHANNEL_TO_PIN[ADC_CHANNEL_COUNT] =
{ 	/* Channel to pin conversion table */	/* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=0 */
	ADC_CHANNEL_PIN_BAT_AD, /* Status and control register value */	/* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=4 */
	ADC_CHANNEL_PIN_I1_AD, /* Status and control register value */	/* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=5 */
	ADC_CHANNEL_PIN_I2_AD, /* Status and control register value */	/* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=1 */
	ADC_CHANNEL_PIN_V1A_AD, /* Status and control register value */	/* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=2 */
	ADC_CHANNEL_PIN_V1B_AD, /* Status and control register value */	/* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=0x0D */
	ADC_CHANNEL_PIN_V2A_AD, /* Status and control register value */	/* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=0x0E */
	ADC_CHANNEL_PIN_V2B_AD, /* Status and control register value */	/* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=0x0F */
	ADC_CHANNEL_PIN_LSTEMP_AD /* Status and control register value */
};

uint8_t MeasureChannelResult[ADC_CHANNEL_COUNT];

uint8_t ADC_SampleGroupChannelPinBuffer[ADC_MAX_HW_SAMPLE_COUNT];

//Sensored Routine
//Store value
//If sensorless, check commutate
//Start measure another

void MonitorGroup2(void)
{
 //Monitor() if too high
}

void MonitorBEMF(void)
{

}

static const uint8_t SampleGroupChannels[] = {ADC_BAT_AD, ADC_I1_AD};

static void OnEndTriggerADCMotor1BEMF(void)
{
//	static const uint8_t MeasureNext[] = {ADC_BAT_AD, ADC_I1_AD, ADC_LSTEMP_AD};
//	static uint8_t i = 0;
//	Measure_StartMeasureChannel(MeasureNext[i], 0);
//	if (i<sizeof(MeasureNext)/sizeof(uint8_t)) i++;
	MonitorBEMF();
	Measure_StartMeasureSampleGroup(&SampleGroupChannels, sizeof(SampleGroupChannels)/sizeof(uint8_t), MonitorGroup2);
}

static inline void TriggerADCMotor1PhaseA(void) { Measure_TriggerMeasureChannelOverwrite(ADC_V1A,		OnEndTriggerADCMotor1BEMF); }
static inline void TriggerADCMotor1PhaseB(void) { Measure_TriggerMeasureChannelOverwrite(ADC_V1B,		OnEndTriggerADCMotor1BEMF); }
static inline void TriggerADCMotor1PhaseC(void) { Measure_TriggerMeasureChannelOverwrite(ADC_I1_AD,		OnEndTriggerADCMotor1BEMF); }

//static inline void TriggerADCMotor1PhaseC(void) {
//	ADC0_CreateSampleGroup(ADC0_DeviceData, SampleGroupChannels, 2);
//	ADC0_StartLoopTriggeredMeasurement(ADC0_DeviceData);
//}

//PWM must be approximately 35 for 4us ADC sample



//I_Max = 50;
//
//if(Stall_Flag1)
//	I_Max=(uint16_t)(I_Max*7)/10;

/*! @} */


/******************************************************************************/
/*!
 *  @name CommutationSection
 *  Commutation functions
 */
/******************************************************************************/
/*! @{ */
uint8_t ReadHallSensorMotor1(void)		{return S1_ALL_PIN_READ();}
//force zero with FTM2_SWOCTRL? //void PwmTimerDisableAll(void) //FTM2_SWOCTRL
void SetPWMMotor1PhaseA(uint8_t pwm)	{PWM_1A_PIN_SET_CV(pwm);}
void SetPWMMotor1PhaseB(uint8_t pwm) 	{PWM_1B_PIN_SET_CV(pwm);}
void SetPWMMotor1PhaseC(uint8_t pwm)	{PWM_1C_PIN_SET_CV(pwm);}

#define SetPWMMotor1PhaseAB(pwm) SetPWMMotor1PhaseA(pwm)
#define SetPWMMotor1PhaseAC(pwm) SetPWMMotor1PhaseA(pwm)
#define SetPWMMotor1PhaseBC(pwm) SetPWMMotor1PhaseB(pwm)
#define SetPWMMotor1PhaseBA(pwm) SetPWMMotor1PhaseB(pwm)
#define SetPWMMotor1PhaseCA(pwm) SetPWMMotor1PhaseC(pwm)
#define SetPWMMotor1PhaseCB(pwm) SetPWMMotor1PhaseC(pwm)

void CommutateMotor1PhaseAB(uint8_t pwm) 	{PWM_1A_PIN_SET_CV(pwm); PWM_1B_PIN_SET_CV(0); EN_1_ALL_PIN_SET(0b011); SE_1_ALL_PIN_SET(0b010); TriggerADCMotor1PhaseC();}
void CommutateMotor1PhaseAC(uint8_t pwm)	{PWM_1A_PIN_SET_CV(pwm); PWM_1C_PIN_SET_CV(0); EN_1_ALL_PIN_SET(0b101); SE_1_ALL_PIN_SET(0b100); TriggerADCMotor1PhaseB();}
void CommutateMotor1PhaseBC(uint8_t pwm)	{PWM_1B_PIN_SET_CV(pwm); PWM_1C_PIN_SET_CV(0); EN_1_ALL_PIN_SET(0b110); SE_1_ALL_PIN_SET(0b100); TriggerADCMotor1PhaseA();}
void CommutateMotor1PhaseBA(uint8_t pwm)	{PWM_1B_PIN_SET_CV(pwm); PWM_1A_PIN_SET_CV(0); EN_1_ALL_PIN_SET(0b011); SE_1_ALL_PIN_SET(0b001); TriggerADCMotor1PhaseC();}
void CommutateMotor1PhaseCA(uint8_t pwm)	{PWM_1C_PIN_SET_CV(pwm); PWM_1A_PIN_SET_CV(0); EN_1_ALL_PIN_SET(0b101); SE_1_ALL_PIN_SET(0b001); TriggerADCMotor1PhaseB();}
void CommutateMotor1PhaseCB(uint8_t pwm)	{PWM_1C_PIN_SET_CV(pwm); PWM_1B_PIN_SET_CV(0); EN_1_ALL_PIN_SET(0b110); SE_1_ALL_PIN_SET(0b010); TriggerADCMotor1PhaseA();}
void EnablePWMMotor1All(void) 				{EN_1_ALL_PIN_SET(0b111); SE_1_ALL_PIN_SET(0b000);}
void DisablePWMMotor1All(void) 				{EN_1_ALL_PIN_SET(0b000); SE_1_ALL_PIN_SET(0b000);}

void SaveParametersCommutationIndex(void)
{
	PARAMETERS_BASE_PTR->CommuntationIndexAB = 0;
	PARAMETERS_BASE_PTR->CommuntationIndexAC = 0;
	PARAMETERS_BASE_PTR->CommuntationIndexBC = 0;
	PARAMETERS_BASE_PTR->CommuntationIndexBA = 0;
	PARAMETERS_BASE_PTR->CommuntationIndexCA = 0;
	PARAMETERS_BASE_PTR->CommuntationIndexCB = 0;
}
/*! @} */

void LEDBlinkOn(void)  	{ LED_PIN_ON();  }
void LEDBlinkOff(void) 	{ LED_PIN_OFF(); }
void LEDBlink(void) 	{ Blinky_Toggle(&LEDPowerButton); }

void Serial(void)
{
	Shell_ProcessNonBlocking();
}

void Task1Second(void)
{
	Measure_StartMeasureChannel(ADC_LSTEMP_AD, 0); //any issues if adc interrupt from previous measurement??
}

void ComTx(void)
{
//	Test_TxPacketValue
//		(
//			Speed_GetRotarySpeedDegreesPerSecond(&Motor1.Speed),
//			Motor1.BackEMF,
//			Motor1.PID.OutMax,
//			20000 - looptime
//			);
	Test_TxPacketValue
		(
			1,
			2,
			3,
			4
		);
//
//	for (i = 0; i < 18; i++)
//		SendChar(TxPacketBuffer[i]);

}









void ConfigLoadDefault(void)
{

}

void ConfigLoadEEPROM(void)
{

}

void Boot(void)
{
//	if (eeprom.setting)
//	{
//		ConfigLoadEEPROM();
//	}
//	else
//	{
//		ConfigDefault();
//	}
}



/******************************************************************************/
/*!
 *  @name	ShellSection
 *  Shell functions
 */
/******************************************************************************/
/*! @{ */

#define SHELL_OUTTER_LOOP_FREQ 100

typedef enum
{
	RETURN_CODE_OK = 0,
	RETURN_CODE_ERROR_1 = 1,
} RETURN_CODE_T;

extern int Cmd_pwm 		(int argc, char ** argv);
extern int Cmd_rpm 		(int argc, char ** argv);
extern int Cmd_jog 		(int argc, char ** argv);
extern int Cmd_run 		(int argc, char ** argv);
extern int Cmd_stop		(int argc, char ** argv);
extern int Cmd_release 	(int argc, char ** argv);
extern int Cmd_vbat		(int argc, char ** argv);

CMDLINE_ENTRY_T CmdEntry_pwm 		=	{ "pwm", 		"Sets pwm value", 			Cmd_pwm		};
CMDLINE_ENTRY_T CmdEntry_rpm 		=	{ "rpm", 		"Display rpm", 				Cmd_rpm		};
CMDLINE_ENTRY_T CmdEntry_jog		=	{ "jog", 		"Jog motor", 				Cmd_jog		};
CMDLINE_ENTRY_T CmdEntry_run 		=	{ "run", 		"Set motor to run mode", 	Cmd_run		};
CMDLINE_ENTRY_T CmdEntry_stop 		=	{ "stop", 		"Set motor to idle mode", 	Cmd_stop	};
CMDLINE_ENTRY_T CmdEntry_release 	=	{ "release", 	"Release hold", 			Cmd_release	};
CMDLINE_ENTRY_T CmdEntry_vbat 		=	{ "vbat", 		"Display battery voltage", 	Cmd_vbat	};

int Cmd_pwm(int argc, char ** argv)
{
	(void)argv;
    if(argc == 2) BLDC_SetPWM(&Motor1, strtoul(argv[1], 0, 10));
    return RESERVED_SHELL_RETURN_CODE_SUCCESS;
}

int Cmd_rpm(int argc, char ** argv)
{
	(void)argv;
    if(argc == 1)
    {
    	Term1_SendStr("\r\nRPM = ");
    	Term1_SendNum(Speed_GetRPM(&Motor1.Speed));
    	Term1_SendStr("\r\n");
    }
    return RESERVED_SHELL_RETURN_CODE_SUCCESS;
}

int Cmd_jog(int argc, char ** argv)
{
	(void)argv;
    if(argc == 2) BLDC_SetJogSteps(&Motor1, strtoul(argv[1], 0, 10));
    return RESERVED_SHELL_RETURN_CODE_SUCCESS;
}

int Cmd_run(int argc, char ** argv)
{
	(void)argv;
    if(argc == 1) Motor1.State = MOTOR_STATE_RUN;
    return RESERVED_SHELL_RETURN_CODE_SUCCESS;
}

int Cmd_stop(int argc, char ** argv)
{
	(void)argv;
    if(argc == 1) Motor1.State = MOTOR_STATE_STANDBY;
    return RESERVED_SHELL_RETURN_CODE_SUCCESS;
}

int Cmd_hold(int argc, char ** argv)
{
	(void)argv;
    if(argc == 1) Motor1.State = MOTOR_STATE_STANDBY;
    return RESERVED_SHELL_RETURN_CODE_SUCCESS;
}

int Cmd_release(int argc, char ** argv)
{
	(void)argv;
    if(argc == 1) BLDC_Release(&Motor1);
    return RESERVED_SHELL_RETURN_CODE_SUCCESS;
}

int Cmd_vbat(int argc, char ** argv)
{
	(void)argv;
    if(argc == 1)
    {
    	Term1_SendStr("\r\nVBat = ");
    	Term1_SendNum(VoltageDivider_GetVoltage(&DividerBattery, *Motor1.VBat_ADCU));
    	Term1_SendStr("\r\n");
    }
    return RESERVED_SHELL_RETURN_CODE_SUCCESS;
}
/*! @} */



















static uint8_t RxPacketBufferArray[20];
static uint8_t TxPacketBufferArray[20];




void KESC_Init(void)
{
	Cpu_DisableInt();
	//Hardware init
	Inhr1_SetBaudRateMode(Inhr1_BM_9600BAUD);
	//Inhr1_SetBaudRateMode(Inhr1_BM_38400BAUD);
	//Inhr1_SetBaudRateMode(Inhr1_BM_115200BAUD);
	ADC_Init();
	//Millis_Init(40000000, 200); //set in generated systick file.



	//Load eeprom

	//Init software modules
	VoltageDivider_Init(&DividerBattery, R1_RATIO, R2_RATIO, VREF, ADC_MAX);
	Blinky_Init(&LEDPowerButton, LEDBlinkOn, LEDBlinkOff);

	Motor1.State = MOTOR_STATE_STANDBY;

	BLDC_Commutation_Init
	(
		&Motor1.Commutation,
		DIRECTION_CW,
		ReadHallSensorMotor1,
		SetPWMMotor1PhaseA,
		SetPWMMotor1PhaseA,
		SetPWMMotor1PhaseB,
		SetPWMMotor1PhaseB,
		SetPWMMotor1PhaseC,
		SetPWMMotor1PhaseC,
		CommutateMotor1PhaseAB,
		CommutateMotor1PhaseAC,
		CommutateMotor1PhaseBC,
		CommutateMotor1PhaseBA,
		CommutateMotor1PhaseCA,
		CommutateMotor1PhaseCB,
		0,
		0,
		0,
		0,
		0,
		0,
		DisablePWMMotor1All
	);

	Measure_Init
	(
		ADC_CHANNEL_COUNT,
		ADC_MAX_HW_SAMPLE_COUNT,
		CHANNEL_TO_PIN,
		MeasureChannelResult,
		ADC_SampleGroupChannelPinBuffer,
		ADC_SetConversion,
		(ADC_DATA_T (*)(uint8_t))ADC_GetResult,
		ADC_AbortConversion,
		ADC_DisableInterrupt
	);

	Speed_InitBLDC(&Motor1.Speed, &FTM1_C1V, FTM1_FREQ, FTM1_CV_MAX, MOTOR1_POLE_PAIRS, PWM_FREQ);

//	PID_Init
//	(
//		&PIDMotor1, Speed_GetPtrEventPeriod(), &Motor1.PIDOutputHallPeriod, &Motor1.PIDSetPointHallPeriod,
//		500, 1, 0,
//		PWM_PEROID, FTM2_FREQ, 10,
//		10, FTM1_CV_MAX
//	);

	Measure_MapAddress(&Motor1.VBat_ADCU, 			ADC_BAT_AD);
	Measure_MapAddress(&Motor1.BackEMFPhaseA_ADCU, 	ADC_V1A);
	Measure_MapAddress(&Motor1.BackEMFPhaseB_ADCU, 	ADC_V1B);
	//Measure_MapAddress(&Motor1.BackEMFPhaseC_ADCU, ADC_V1B);
	Measure_MapAddress(&Motor1.I_ADCU, 				ADC_I1_AD);
	Measure_MapAddress(&Motor1.LSTemp_ADCU, 		ADC_LSTEMP_AD);
//	Measure_MapAddress(&Motor2.BackEMFPhaseA_ADCU, ADC_V2A);
//	Measure_MapAddress(&Motor2.BackEMFPhaseB_ADCU, ADC_V2B);
//	Measure_MapAddress(&Motor2.BackEMFPhaseC_ADCU, ADC_V2B);
//	Measure_MapAddress(&Motor2.I_ADCU, 			ADC_I2_AD);

	Shell_InitNonBlocking(10, SHELL_OUTTER_LOOP_FREQ);
	Shell_RegisterCmdLineEntry(&CmdEntry_pwm);
	Shell_RegisterCmdLineEntry(&CmdEntry_rpm);
	Shell_RegisterCmdLineEntry(&CmdEntry_jog);
	Shell_RegisterCmdLineEntry(&CmdEntry_run);
	Shell_RegisterCmdLineEntry(&CmdEntry_stop);
	Shell_RegisterCmdLineEntry(&CmdEntry_release);
	Shell_RegisterCmdLineEntry(&CmdEntry_vbat);

	LiteFXOS_InitMillis(Millis_GetTickCounter());
	LiteFXOS_InitThreadPeriodicArgPeriod(&ThreadLED, 		 	LEDBlink, 		1000);
	LiteFXOS_InitThreadPeriodicArgPeriod(&ThreadSerial, 		Serial, 		SHELL_OUTTER_LOOP_FREQ);
	LiteFXOS_InitThreadPeriodicArgPeriod(&ThreadTask1Second, 	Task1Second, 	1000);
	LiteFXOS_InitThreadPeriodicArgPeriod(&ThreadComTx, 			ComTx, 			20);

	LiteFXOS_SetThreadStart(&ThreadLED);
	LiteFXOS_SetThreadStart(&ThreadSerial);
	LiteFXOS_SetThreadStart(&ThreadTask1Second);
	LiteFXOS_SetThreadStart(&ThreadComTx);
	Cpu_EnableInt();

	BLDC_Commutation_MapCommuntationTableRunCalibration
	(	
		&Motor1.Commutation,
		0,
		0,
		0,
		0,
		0,
		0,
		SetPWMMotor1PhaseA,
		SetPWMMotor1PhaseA,
		SetPWMMotor1PhaseB,
		SetPWMMotor1PhaseB,
		SetPWMMotor1PhaseC,
		SetPWMMotor1PhaseC,
		12,
		CommutateMotor1PhaseAB, 
		CommutateMotor1PhaseAC,
		CommutateMotor1PhaseBC,
		CommutateMotor1PhaseBA,
		CommutateMotor1PhaseCA,
		CommutateMotor1PhaseCB,
		EnablePWMMotor1All,
		Delay,
		1000
	);

	BLDC_SetPWM(&Motor1, 12);
	ADC_Poll();

	Test_Init
	(
		TxPacketBufferArray,
		RxPacketBufferArray,
		Inhr1_GetCharsInRxBuf,
		Inhr1_RecvChar,
		Inhr1_SendChar
	);

	//SendStartMessage();
}



void KESC_Loop(void)
{
	static volatile uint32_t t1, t2, delta;

	while(1)
	{
		t1 = Micros();

	//20ms send thread


		//__DI();
		//WDOG_Feed();
		//__EI();

		LiteFXOS_ProcThread(&ThreadLED);
		//LiteFXOS_ProcThread(&ThreadSerial);
		LiteFXOS_ProcThread(&ThreadTask1Second);

		BLDC_Process(&Motor1);



		Test_StartUp();
		Test_RxPacket(); ///always receive

		//LiteFXOS_ProcThread(&ThreadComTx); //tx every 20ms

		//		if (Test_RxPacket())
		//		{
		//			Motor1.PID.Kp = Test_GetKp();
		//		}

		Motor1.PID.Kp = Test_GetKp();
		Motor1.PID.Ki = Test_GetKi();
		Motor1.PID.Kd = Test_GetKd();


//		//BLDC Fault checking
//
//		if
//		(
//			ErrorFlags.HallSensor1 ||
//			ErrorFlags.Identify1 ||
//			ErrorFlags.HallSensor2 ||
//			ErrorFlags.Identify2 ||
//			ErrorFlags.OverVolt /*|| Motor_Temp_Err*/ ||
//			ErrorFlags.LowVolt ||
//			ErrorFlags.StartupTPS||
//			ErrorFlags.Reverse ||
//			ErrorFlags.RegenOverVolt ||
//			ErrorFlags.Com /*||Hall_Sensor_Err||Identify_Err*/  ||
//			ErrorFlags.OverTemp
//		)
//		{
//			SE_1_ALL_PIN_OFF();
//			EN_1_ALL_PIN_OFF();
//			SE_2_ALL_PIN_OFF();
//			EN_2_ALL_PIN_OFF();
//			//M_En=0;
//			//Global_En=0;
//			//Target_TPSx64 = 0;
//		}
//		else
//		{
//			//Global_En = 1;
//		}

		//Debugging

		t2 = Micros();
//		if (LiteFXOS_ProcThread(&ThreadTask1Second))
//		{
//	    	Term1_SendStr("\r\nLoop time t2 ");
//	    	Term1_SendNum(t2);
//	    	Term1_SendStr("\r\n");
//	    	Term1_SendStr("\r\nLoop time t1 ");
//	    	Term1_SendNum(t1);
//	    	Term1_SendStr("\r\n");
//		}

	}
}

