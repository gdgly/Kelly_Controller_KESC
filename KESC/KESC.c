#include "KESC.h"
#include "Board.h"
#include "Parameters.h"

// From Kelly Library
#include "BLDC/BLDC.h"
#include "BLDC/Commutation.h"
#include "BLDC/Waveform.h"
#include "BLDC/Monitor.h"

#include "VoltageDivider/VoltageDivider.h"
#include "PID/PID.h"
#include "Millis/Millis.h"
#include "OS/OS.h"
#include "Blinky/Blinky.h"
#include "Measure/Measure.h"

#include "Shell/Terminal.h"
#include "Shell/Shell.h"

#include "KEA64/SysTick.h"
#include "KEA64/ADC.h"
#include "KEA64/UART0.h"
	#include "KEA64/Serial0.h"
#include "KEA64/FTM1.h"

#include "freemaster.h"

// PE Drivers
#include "Cpu.h"
#include "Events.h"
#include "GPIOA.h"
#include "GPIOB.h"
#include "FTM1.h"
#include "FTM2.h"
//#include "WDOG.h"

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

volatile static KESC_DIRECTION_T Direction; // Direction of travel

BLDC_CONTROLLER_T	Motor1;
PID_T 				Motor1PID;
MONITOR_T			Motor1Monitor;

static uint32_t LoopTimer, LoopDelta;
BLINKY_T 			LEDPowerButton;

LITE_FX_OS_THREAD_T ThreadLED;
LITE_FX_OS_THREAD_T ThreadSerial;
LITE_FX_OS_THREAD_T ThreadTask1Second;
LITE_FX_OS_THREAD_T ThreadComTx;			//20ms period task for communications




/******************************************************************************/
/*!
 * @name  	VoltageDividers
 * @brief 	ADCU to voltage conversion
 */
/******************************************************************************/
/*! @{ */
VOLTAGE_DIVIDER_T	DividerCommon; // Battery, BackEMF,
VOLTAGE_DIVIDER_T	DividerTemp; // LSTemp

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
/*! @} */

/******************************************************************************/
/*!
 * @name  	MeasureADCSection
 * @brief 	ADC Config
 *
 * PWM must be approximately 35 for 4us ADC sample
 */
/******************************************************************************/
/*! @{ */
MEASURE_T MeasureADC0;

#define MEASURE_ADC_CHANNEL_COUNT			8U

//	Virtual/Measure Channel, index into arrays
#define MEASURE_CHANNEL_BAT_AD		0U
#define MEASURE_CHANNEL_I1_AD		1U
#define MEASURE_CHANNEL_I2_AD		2U
#define MEASURE_CHANNEL_V1A			3U
#define MEASURE_CHANNEL_V1B			4U
#define MEASURE_CHANNEL_V2A			5U
#define MEASURE_CHANNEL_V2B			6U
#define MEASURE_CHANNEL_LSTEMP_AD	7U

uint8_t Measure_ChannelResultBuffer[MEASURE_ADC_CHANNEL_COUNT];
uint8_t Measure_ChannelSumBuffer[0]; //[MEASURE_ADC_CHANNEL_COUNT];

const uint8_t DummyPhaseCBuffer = 0;

// ADC Pins
#define ADC_CHANNEL_PIN_BAT_AD		ADC_PDD_SINGLE_ENDED_DAD0
#define ADC_CHANNEL_PIN_I1_AD		ADC_PDD_SINGLE_ENDED_AD4
#define ADC_CHANNEL_PIN_I2_AD		ADC_PDD_SINGLE_ENDED_AD5
#define ADC_CHANNEL_PIN_V1A_AD 		ADC_PDD_SINGLE_ENDED_DAD1
#define ADC_CHANNEL_PIN_V1B_AD 		ADC_PDD_SINGLE_ENDED_DAD2
#define ADC_CHANNEL_PIN_V2A_AD 		ADC_PDD_SINGLE_ENDED_AD13
#define ADC_CHANNEL_PIN_V2B_AD 		ADC_PDD_SINGLE_ENDED_AD14
#define ADC_CHANNEL_PIN_LSTEMP_AD 	ADC_PDD_SINGLE_ENDED_AD15

const uint8_t MEASURE_CHANNEL_TO_ADC_PIN[MEASURE_ADC_CHANNEL_COUNT] =
{
	ADC_CHANNEL_PIN_BAT_AD,
	ADC_CHANNEL_PIN_I1_AD,
	ADC_CHANNEL_PIN_I2_AD,
	ADC_CHANNEL_PIN_V1A_AD,
	ADC_CHANNEL_PIN_V1B_AD,
	ADC_CHANNEL_PIN_V2A_AD,
	ADC_CHANNEL_PIN_V2B_AD,
	ADC_CHANNEL_PIN_LSTEMP_AD
};

uint8_t ADC_SampleChannelsBuffer[ADC_MAX_HW_SAMPLE_COUNT]; // array of pin channels pushed to adc fifo

// runs after all items in fifo are processed.
// only 1 channel in fifo is processed every hw trigger.
// entire fifo is processed using software trigger.
void ADC_ISR(void)
{
	Measure_CompleteISR(&MeasureADC0);
}

extern MEASURE_SAMPLE_T MeasureSampleBEMFMotor1A;
extern MEASURE_SAMPLE_T MeasureSampleBEMFMotor1B;
extern MEASURE_SAMPLE_T MeasureSampleBEMFMotor1C;
static const uint8_t ChannelsMotor1A2nd[] = {MEASURE_CHANNEL_I1_AD, MEASURE_CHANNEL_BAT_AD, MEASURE_CHANNEL_LSTEMP_AD};
static const uint8_t ChannelsMotor1B2nd[] = {MEASURE_CHANNEL_I1_AD, MEASURE_CHANNEL_BAT_AD, MEASURE_CHANNEL_LSTEMP_AD};
static const uint8_t ChannelsMotor1C2nd[] = {MEASURE_CHANNEL_I1_AD, MEASURE_CHANNEL_BAT_AD, MEASURE_CHANNEL_LSTEMP_AD};
static void OnEndADCMotor1A2nd(void) {Measure_Start(&MeasureADC0, &MeasureSampleBEMFMotor1A);}
static void OnEndADCMotor1B2nd(void) {Measure_Start(&MeasureADC0, &MeasureSampleBEMFMotor1B);}
static void OnEndADCMotor1C2nd(void) {Measure_Start(&MeasureADC0, &MeasureSampleBEMFMotor1C);}

MEASURE_SAMPLE_T MeasureSampleMotor1A2nd =
{
	.Channels.ChannelGroup = ChannelsMotor1A2nd,
	.ChannelCount = sizeof(ChannelsMotor1A2nd),
	.HWTrigger = true,
	.OnEndISR = OnEndADCMotor1A2nd,
	.Overwrite = false,
	.RepeatCount = 1,
	.RepeatCounter = 0,
};

MEASURE_SAMPLE_T MeasureSampleMotor1B2nd =
{
	.Channels.ChannelGroup = ChannelsMotor1B2nd,
	.ChannelCount = sizeof(ChannelsMotor1B2nd),
	.HWTrigger = true,
	.OnEndISR = OnEndADCMotor1B2nd,
	.Overwrite = false,
	.RepeatCount = 1,
	.RepeatCounter = 0,
};

MEASURE_SAMPLE_T MeasureSampleMotor1C2nd =
{
	.Channels.ChannelGroup = ChannelsMotor1C2nd,
	.ChannelCount = sizeof(ChannelsMotor1C2nd),
	.HWTrigger = true,
	.OnEndISR = OnEndADCMotor1C2nd,
	.Overwrite = false,
	.RepeatCount = 1,
	.RepeatCounter = 0,
};

static void OnEndADCMotor1PhaseA(void)
{
	//Monitor_Process(); // here or main?
	Monitor_CaptureFilterBEMFPhaseA(&Motor1Monitor);
	Measure_Start(&MeasureADC0, &MeasureSampleMotor1A2nd);
}

static void OnEndADCMotor1PhaseB(void)
{
	Monitor_CaptureFilterBEMFPhaseB(&Motor1Monitor);
	Measure_Start(&MeasureADC0, &MeasureSampleMotor1B2nd);
}

static void OnEndADCMotor1PhaseC(void)
{
	//Monitor_CaptureBEMFPhaseC(&Motor1Monitor);
	Measure_Start(&MeasureADC0, &MeasureSampleMotor1C2nd);
}

MEASURE_SAMPLE_T MeasureSampleBEMFMotor1A =
{
	.Channels.ChannelSingle = MEASURE_CHANNEL_V1A,
	.ChannelCount = 1,
	.HWTrigger = true,
	.OnEndISR = OnEndADCMotor1PhaseA,
	.Overwrite = true,
	.RepeatCount = 1,
	.RepeatCounter = 0,
};

MEASURE_SAMPLE_T MeasureSampleBEMFMotor1B =
{
	.Channels.ChannelSingle = MEASURE_CHANNEL_V1B,
	.ChannelCount = 1,
	.HWTrigger = true,
	.OnEndISR = OnEndADCMotor1PhaseB,
	.Overwrite = true,
	.RepeatCount = 1,
	.RepeatCounter = 0,
};

MEASURE_SAMPLE_T MeasureSampleBEMFMotor1C =
{
	.Channels.ChannelSingle = 0, //will end up measuring battery.
	.ChannelCount = 1,
	.HWTrigger = true,
	.OnEndISR = OnEndADCMotor1PhaseC,
	.Overwrite = true,
	.RepeatCount = 1,
	.RepeatCounter = 0,
};

//arm hw trigger on commutation
static inline void TriggerADCMotor1PhaseA(void) { Measure_Start(&MeasureADC0, &MeasureSampleBEMFMotor1A); }//Monitor_ZeroCaptureFilterBEMF(&Motor1Monitor);}
static inline void TriggerADCMotor1PhaseB(void) { Measure_Start(&MeasureADC0, &MeasureSampleBEMFMotor1B); }//Monitor_ZeroCaptureFilterBEMF(&Motor1Monitor);}
static inline void TriggerADCMotor1PhaseC(void) { Measure_Start(&MeasureADC0, &MeasureSampleBEMFMotor1C); }//Monitor_ZeroCaptureFilterBEMF(&Motor1Monitor);}
/*! @} */


#define CPU_FREQ 			40000000
#define BUS_FREQ 			20000000

/******************************************************************************/
/*!
 * @name  	Hall Timer
 * @brief	Freq = 312,500 Hz, Peroid = 3.2 uS, Overflow 209,712 us, 209 ms
 *
 * Set to capture Motor 1 Hall A
 * Min rpm when delta = 0xFFFF:
 * 312,500/10*60/0xFFFF = 28 rpm min for 10 pole pairs
 *
 * 10,000 rpm -> delta = 187
 *
 * // 1 POLE_PAIR = 6 STEPS = ELECTRIC_R
// 60 Steps = 1 MECH_ROTATION:
// 60 Steps / 6 Steps = 10 PolePairs
 */
/******************************************************************************/
/*! @{ */
SPEED_T 			Motor1Speed;

#define FTM1_BUS_PEROID		(64)
#define FTM1_FREQ 			(BUS_FREQ/FTM1_BUS_PEROID)
#define FTM1_CV_MAX			(0xFFFF)

inline static bool FTM1_Filter(void)
{
	volatile static uint32_t td, ts;

	td = Micros() - ts;
	ts = Micros();

	if (td < 100) return true;

	return false;
}

//Motor1 Hall A rising edge trigger
void FTM1_ISR(void)
{
	//volatile uint32_t reg = FTM1_C1SC;
	FTM1_C1SC &= ~FTM_CnSC_CHF_MASK;	//FTM_PDD_ClearChannelInterruptFlag(FTM1_DEVICE, FTM_PDD_CHANNEL_1);

	//if (FTM1_Filter()) return;

//	BLDC_Commutation_ISR(&Motor1.Commutation, Motor1.PWM); //commutate 1/6 times here?
	//Speed_CaptureDeltaISR(&Motor1Speed);
	//Speed_DeltaOverflowDetectionISR(&Motor1Speed);

	//FTM1_C1SC = reg & ~FTM_CnSC_CHF_MASK;
	// If another event occurs between the read and write operations, the write operation has no effect; therefore, CHF remains set indicating an event has occurred.
	FTM1_C1SC &= ~FTM_CnSC_CHF_MASK;	//FTM_PDD_ClearChannelInterruptFlag(FTM1_DEVICE, FTM_PDD_CHANNEL_1);
}
/*! @} */



/******************************************************************************/
/*!
 * @name  	PWM Timer
 * @brief	51.2 us duty cycle
 *
 */
/******************************************************************************/
/*! @{ */
COMMUTATION_T  Motor1Commutation;

#define FTM2_BUS_PEROID	 	(1)
#define FTM2_FREQ 			(BUS_FREQ/FTM2_BUS_PEROID)
#define FTM2_MODULO 		(512)
#define PWM_MAX 			(FTM2_MODULO)
#define PWM_PEROID			(FTM2_MODULO*2) // FTM2 ticks, Modulo * 2 for center aligned PWM
#define PWM_FREQ			(FTM2_FREQ/PWM_PEROID)
#define FTM2_ISR_FREQ 		(FTM2_FREQ/PWM_PEROID)

//PWM counts per hall cycle = delta_FTM1_CV * 64 / 1024

extern uint8_t ReadHallSensorMotor1(void);

//FTM2 PWM Cycle trigger, beginning of PWM cycle. (Output is low)
void FTM2_ISR(void)
{
	//(void)FTM2_SC;
	FTM2_SC &= ~FTM_SC_TOF_MASK;	//FTM_PDD_ClearOverflowInterruptFlag(FTM2_DEVICE);

	//Speed_CaptureDeltaPoll(&Motor1Speed, ReadHallSensorMotor1()&0x01);
	Speed_CaptureLongDeltaPoll(&Motor1Speed, ReadHallSensorMotor1()&0x01);

	Waveform_ModulateAngleISR(Motor1.PWM, 1);

	BLDC_ProcessRunPoll(&Motor1); //if stops at same sensor state?

	Motor1.RPM 	= Speed_GetRPM(&Motor1Speed);
//if (Motor1.MotorMode == MOTOR_MODE_RUN) PID_ComputeTimerISR(&Motor1PID);

}
/*! @} */

/******************************************************************************/
/*!
 *  @name CommutationSection
 *  Commutation functions
 */
/******************************************************************************/
/*! @{ */
uint8_t ReadHallSensorMotor1(void) {return S1_ALL_PIN_READ();}
//force zero with FTM2_SWOCTRL? //void PwmTimerDisableAll(void) //FTM2_SWOCTRL
void EnableMotor1PhaseABC(void) 			{EN_1CBA_PIN_SET_OUT(0b111); SE_1CBA_PIN_SET_OUT(0b111);}
void DisableMotor1PhaseABC(void) 			{EN_1CBA_PIN_SET_OUT(0b000); SE_1CBA_PIN_SET_OUT(0b000);} // all mosfet off, motor coasts
void Set0PWMMotor1PhaseABC(void)		 	{PWM_1A_PIN_SET_CV(0); PWM_1B_PIN_SET_CV(0); PWM_1C_PIN_SET_CV(0); EN_1CBA_PIN_SET_OUT(0b111); SE_1CBA_PIN_SET_OUT(0b000);} // all mosfet top side off, low side on, short motor terminals for dynamic brake
void RestPWMPolarityMotor1PhaseABC(void) 	{PWM_1ABC_PIN_SET_POL(0b000);}

//void SetPWMMotor1_8(uint8_t pwmA, uint8_t pwmB, uint8_t pwmC) 	{PWM_1A_PIN_SET_CV((uint16_t)pwmA<<1); PWM_1B_PIN_SET_CV((uint16_t)pwmB<<1); PWM_1C_PIN_SET_CV((uint16_t)pwmC<<1);}
void SetPWMMotor1(uint16_t pwmA, uint16_t pwmB, uint16_t pwmC) 	{PWM_1A_PIN_SET_CV(pwmA); PWM_1B_PIN_SET_CV(pwmB); PWM_1C_PIN_SET_CV(pwmC);}
void EnablePWMMotor1(bool enA, bool enB, bool enC) 				{EN_1CBA_PIN_SET_OUT(enC<<2|enB<<1|enA);}
void InversePWMMotor1(bool invA, bool invB, bool invC)		 	{PWM_1ABC_PIN_SET_POL(invA<<2|invB<<1|invC);}

void OnMotor1PhaseAB(void) {SE_1CBA_PIN_SET_OUT(0b010); TriggerADCMotor1PhaseC();};
void OnMotor1PhaseAC(void) {SE_1CBA_PIN_SET_OUT(0b100); TriggerADCMotor1PhaseB();};
void OnMotor1PhaseBC(void) {SE_1CBA_PIN_SET_OUT(0b100); TriggerADCMotor1PhaseA();};
void OnMotor1PhaseBA(void) {SE_1CBA_PIN_SET_OUT(0b001); TriggerADCMotor1PhaseC();};
void OnMotor1PhaseCA(void) {SE_1CBA_PIN_SET_OUT(0b001); TriggerADCMotor1PhaseB();};
void OnMotor1PhaseCB(void) {SE_1CBA_PIN_SET_OUT(0b010); TriggerADCMotor1PhaseA();};
/*! @} */

void LEDBlinkOn(void)  	{ LED_PIN_ON();  }
void LEDBlinkOff(void) 	{ LED_PIN_OFF(); }
void LEDBlink(void) 	{ Blinky_Toggle(&LEDPowerButton); }


void Serial(void)
{
	//Shell_ProcessNonBlocking();
}

void ComTx(void)
{


}

void Task1Second(void)
{
	//PrintDebug();
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

LITE_FX_OS_THREAD_T ThreadPIDTuning;

void PIDTuning(void) //2s
{
	static bool toggle = 0;

	toggle ^= 1;

	if (toggle)
		Motor1.SetPointRPM = 100;
	else
		Motor1.SetPointRPM = 300;
}


uint8_t IndexAB;
uint8_t IndexAC;
uint8_t IndexBC;
uint8_t IndexBA;
uint8_t IndexCA;
uint8_t IndexCB;

void KESC_Init(void)
{
	Cpu_DisableInt();
	//Hardware init
	Serial0_Init();
	Serial0_SetBaudRateMode(Serial0_BM_9600BAUD);
	//Inhr1_SetBaudRateMode(Inhr1_BM_38400BAUD);
	//Inhr1_SetBaudRateMode(Inhr1_BM_115200BAUD);

	/* ADC_APCTL1: ADPC=0xE037 */
	ADC_Init(0xE037);
	SysTick_Init();
	FTM1_Init();

	//Millis_Init(40000000, 200); // set in generated systick file.

	Measure_Init
	(
		&MeasureADC0,
		ADC_SetConversion,
		ADC_SampleChannelsBuffer,
		ADC_MAX_HW_SAMPLE_COUNT,
		(ADC_DATA_T (*)(uint8_t))ADC_GetResult,
		ADC_DisableInterrupt,
		ADC_GetConversionActiveFlag,
		ADC_GetConversionCompleteFlag,
		ADC_AbortConversion
	);

	Measure_InitModule
	(
		Measure_ChannelResultBuffer,
		Measure_ChannelSumBuffer,
		MEASURE_CHANNEL_TO_ADC_PIN,
		MEASURE_ADC_CHANNEL_COUNT,
		Cpu_DisableInt,
		Cpu_EnableInt
	);

	//Init software modules
	VoltageDivider_Init(&DividerCommon, R1_RATIO, R2_RATIO, VREF, ADC_MAX);
	Blinky_Init(&LEDPowerButton, LEDBlinkOn, LEDBlinkOff);

	//Load eeprom
	Commutation_Init
	(
		&Motor1Commutation,
		ReadHallSensorMotor1,
		Waveform_CommutatePhaseAB,
		Waveform_CommutatePhaseAC,
		Waveform_CommutatePhaseBC,
		Waveform_CommutatePhaseBA,
		Waveform_CommutatePhaseCA,
		Waveform_CommutatePhaseCB,
		0,
		0,
		0,
		0,
		0,
		0,
		DisableMotor1PhaseABC,
		DisableMotor1PhaseABC
	);

	Speed_InitHallEncoder(&Motor1Speed, &FTM1_C1V, FTM1_CV_MAX, FTM1_FREQ, MOTOR1_POLE_PAIRS, 0, PWM_FREQ);
	Speed_InitDeltaOverflowDetection(&Motor1Speed, Millis_GetTickCounter());

	Waveform_InitSinusoidalModulation
	(
		SetPWMMotor1,
		EnablePWMMotor1,
		InversePWMMotor1,
		OnMotor1PhaseAB,
		OnMotor1PhaseAC,
		OnMotor1PhaseBC,
		OnMotor1PhaseBA,
		OnMotor1PhaseCA,
		OnMotor1PhaseCB,
		Speed_GetPtrDelta(&Motor1Speed),
		FTM1_FREQ,
		PWM_FREQ
	);

	PID_Init
	(
		&Motor1PID,
		&Motor1.RPM, &Motor1.PWM, &Motor1.SetPointRPM,
		20, 100, 0, 1, 0, 1,
		FTM2_FREQ, PWM_PEROID, 20,
		0, PWM_MAX/4
	);

	Monitor_Init
	(
		&Motor1Monitor,
		&Measure_ChannelResultBuffer[MEASURE_CHANNEL_BAT_AD],
		&Measure_ChannelResultBuffer[MEASURE_CHANNEL_V1A],
		&Measure_ChannelResultBuffer[MEASURE_CHANNEL_V1B],
		&DummyPhaseCBuffer, //no phase c
		&Measure_ChannelResultBuffer[MEASURE_CHANNEL_I1_AD],
		&Measure_ChannelResultBuffer[MEASURE_CHANNEL_LSTEMP_AD],
		&DividerCommon,
		&DividerCommon,
		&DividerCommon
	);

	//Map Motor Structs
	BLDC_Init
	(
		&Motor1,
		&Motor1Commutation,
		&Motor1Speed,
		&Motor1PID,
		&Motor1Monitor,
		DisableMotor1PhaseABC,
		Set0PWMMotor1PhaseABC,
		PWM_MAX
	);

	//KellyMotorShell_Init();
		//Shell_InitNonBlocking(10, SHELL_OUTTER_LOOP_FREQ);
		//RegisterShellCmds();

	FMSTR_Init();

	LiteFXOS_InitMillis(Millis_GetTickCounter());
	LiteFXOS_InitThreadPeriodicArgPeriod(&ThreadLED, 		 	LEDBlink, 		1000);
	//LiteFXOS_InitThreadPeriodicArgPeriod(&ThreadSerial, 		Serial, 		SHELL_OUTTER_LOOP_FREQ);
	LiteFXOS_InitThreadPeriodicArgPeriod(&ThreadTask1Second, 	Task1Second, 	1000);
	LiteFXOS_InitThreadPeriodicArgPeriod(&ThreadComTx, 			ComTx, 			20);
	LiteFXOS_InitThreadPeriodicArgPeriod(&ThreadPIDTuning, 		PIDTuning, 		2000);

	LiteFXOS_SetThreadStart(&ThreadLED);
	//LiteFXOS_SetThreadStart(&ThreadSerial);
	LiteFXOS_SetThreadStart(&ThreadTask1Second);
	LiteFXOS_SetThreadStart(&ThreadComTx);
	LiteFXOS_SetThreadStart(&ThreadPIDTuning);
	Cpu_EnableInt();


}

bool GlobalPIDTuning = 0;

void KESC_Loop(void)
{
	// Temp run
	Commutation_MapCommuntationTableRunCalibration
	(
		&Motor1Commutation,
		&IndexAB,
		&IndexAC,
		&IndexBC,
		&IndexBA,
		&IndexCA,
		&IndexCB,
		Waveform_CommutatePhaseAB,
		Waveform_CommutatePhaseAC,
		Waveform_CommutatePhaseBC,
		Waveform_CommutatePhaseBA,
		Waveform_CommutatePhaseCA,
		Waveform_CommutatePhaseCB,
		12,
		EnablePWMMotor1,
		Delay,
		1000
	);

	Waveform_SetMode(0);
	//Waveform_EnableSinusoidalModulation();
	//Waveform_DisableSinusoidalModulation();


	while(1)
	{
		LoopDelta = Micros() - LoopTimer;
		LoopTimer = Micros();

		//__DI();
		//WDOG_Feed();
		//__EI();
		//Speed_CaptureDeltaPoll(&Motor1Speed, ReadHallSensorMotor1()&0x01);

		LiteFXOS_ProcThread(&ThreadLED);
		LiteFXOS_ProcThread(&ThreadTask1Second);

		BLDC_Process(&Motor1);


		if (GlobalPIDTuning) LiteFXOS_ProcThread(&ThreadPIDTuning);

		//		Monitor_Process(&Motor1Monitor);
		if(LiteFXOS_ProcThread(&ThreadComTx))
		{
			Monitor_Convert(&Motor1Monitor);

			Motor1.PWMVoltage 	= BLDC_GetPWMVoltage(&Motor1);
		}

		FMSTR_Poll();
		FMSTR_Recorder();

		//LiteFXOS_ProcThread(&ThreadSerial);

		//Test_StartUp();
		//Test_RxPacket(); ///always receive

		//Debugging

	}
}

