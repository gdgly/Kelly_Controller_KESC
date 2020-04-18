#include "KESC.h"
#include "Board.h"
#include "Parameters.h"

// From Kelly Library
#include "BLDC/BLDC.h"
#include "BLDC/Commutation.h"

#include "Shell/Terminal.h"
#include "Shell/Shell.h"

#include "VoltageDivider/VoltageDivider.h"
#include "PID/PID.h"
#include "Millis/Millis.h"
#include "OS/OS.h"
#include "Blinky/Blinky.h"
#include "Measure/Measure.h"

#include "KEA64/SysTick.h"
#include "KEA64/ADC.h"
#include "KEA64/UART0.h"
#include "KEA64/Serial0.h"

#include "freemaster.h"

// PE Drivers
#include "Cpu.h"
#include "Events.h"
#include "GPIOA.h"
#include "GPIOB.h"
#include "FTM1.h"
#include "FTM2.h"
//#include "WDOG.h"

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


void LEDBlinkOn(void)  	{ LED_PIN_ON();  }
void LEDBlinkOff(void) 	{ LED_PIN_OFF(); }
void LEDBlink(void) 	{ Blinky_Toggle(&LEDPowerButton); }

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
#define ADC_BAT_AD		0U
#define ADC_I1_AD		1U
#define ADC_I2_AD		2U
#define ADC_V1A			3U
#define ADC_V1B			4U
#define ADC_V2A			5U
#define ADC_V2B			6U
#define ADC_LSTEMP_AD	7U

uint8_t Measure_ChannelResultBuffer[MEASURE_ADC_CHANNEL_COUNT];
//uint8_t Measure_ChannelSumBuffer[MEASURE_ADC_CHANNEL_COUNT];
uint8_t Measure_ChannelSumBuffer[0];

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

uint8_t ADC_SampleChannelsBuffer[ADC_MAX_HW_SAMPLE_COUNT]; // array of pin channels pushed to adc fifo

void ADC_ISR(void)
{
	LEDBlink();
	LEDBlink();
	LEDBlink();
	Measure_ISR(&MeasureADC0);
}

static const uint8_t ChannelsMotor1A[] = {ADC_V1A, ADC_I1_AD, ADC_BAT_AD, ADC_LSTEMP_AD};
static const uint8_t ChannelsMotor1B[] = {ADC_V1B, ADC_I1_AD, ADC_BAT_AD, ADC_LSTEMP_AD};
static const uint8_t ChannelsMotor1C[] = {ADC_I1_AD, ADC_BAT_AD, ADC_LSTEMP_AD};

static void OnEndADCMotor1PhaseA(void)
{
	//BLDC_CaptureBEMF(&Motor1, Measure_GetAddr(ADC_V1A));
	//BLDC_CaptureBEMF(&Motor1, Motor1.BackEMFPhaseA_ADCU);
	BLDC_SetBEMF(&Motor1, Measure_ChannelResultBuffer[ADC_V1A]);
	//Monitor_Process(); // here or main?
}

static void OnEndADCMotor1PhaseB(void)
{
	//BLDC_CaptureBEMF(&Motor1, Motor1.BackEMFPhaseB_ADCU);
	BLDC_SetBEMF(&Motor1, Measure_ChannelResultBuffer[ADC_V1B]);
}

static void OnEndADCMotor1PhaseC(void)
{

}

MEASURE_SAMPLE_T MeasureSampleMotor1A =
{
	.Channels.ChannelGroup = ChannelsMotor1A,
	.ChannelCount = sizeof(ChannelsMotor1A),
	.HWTrigger = true,
	.OnEndISR = OnEndADCMotor1PhaseA,
	.Overwrite = true,
	.RepeatCount = 1,
	.RepeatCounter = 0,
};

MEASURE_SAMPLE_T MeasureSampleMotor1B =
{
		.Channels.ChannelGroup = ChannelsMotor1B,
		.ChannelCount = sizeof(ChannelsMotor1B),
		.HWTrigger = true,
		.OnEndISR = OnEndADCMotor1PhaseB,
		.Overwrite = true,
		.RepeatCount = 1,
		.RepeatCounter = 0,
};

MEASURE_SAMPLE_T MeasureSampleMotor1C =
{
		.Channels.ChannelGroup = ChannelsMotor1C,
		.ChannelCount = sizeof(ChannelsMotor1C),
		.HWTrigger = true,
		.OnEndISR = OnEndADCMotor1PhaseC,
		.Overwrite = true,
		.RepeatCount = 1,
		.RepeatCounter = 0,
};

static inline void TriggerADCMotor1PhaseA(void) { Measure_Start(&MeasureADC0, &MeasureSampleMotor1A); }
static inline void TriggerADCMotor1PhaseB(void) { Measure_Start(&MeasureADC0, &MeasureSampleMotor1B); }
static inline void TriggerADCMotor1PhaseC(void) { Measure_Start(&MeasureADC0, &MeasureSampleMotor1C); }
/*! @} */

/******************************************************************************/
/*!
 * @name  	Hall Timer
 * @brief	Freq = 312,500 Hz, Peroid = 3.2 uS, Overflow 209,712 us, 209 ms
 *
 * Hall triggered ISR
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

#define CPU_FREQ 			40000000
#define BUS_FREQ 			20000000

#define FTM1_BUS_PEROID		(64)
#define FTM1_FREQ 			(BUS_FREQ/FTM1_BUS_PEROID)
#define FTM1_CV_MAX			(0xFFFF)

//static bool MotorSelect;

//Motor1 Hall A rising edge trigger
void FTM1_ISR(void)
{
	FTM1_C1SC &= ~FTM_CnSC_CHF_MASK;	//FTM_PDD_ClearChannelInterruptFlag(FTM1_DEVICE, FTM_PDD_CHANNEL_1);

//	BLDC_Commutation_ISR(&Motor1.Commutation, Motor1.PWM); //commutate 1/3 times here?

	Speed_CaptureDeltaISR(&Motor1Speed);

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

//PWM counts per hall cycle = delta_FTM1_CV * 64 / 1024

//FTM2 PWM Cycle trigger
void FTM2_ISR(void)
{
	FTM2_SC &= ~FTM_SC_TOF_MASK;	//FTM_PDD_ClearOverflowInterruptFlag(FTM2_DEVICE);

	if (Motor1.State == MOTOR_STATE_RUN) Commutation_Poll(&Motor1Commutation, Motor1.PWM);


	//Optionally measure Back EMF at beginning of PWM cycle. (Output is low)
	//Triggered measure will occur again at center of pulse, use for motor current.
	//StartADCPhaseA();
	//StartADCPhaseB();
	//StartADCPhaseC();


	// calculate using pid,


	// if set pid this loop, direct change pwm, dont need to check if hall state changed.
//	BLDC_Commutation_ISR(&Motor1.Commutation, Motor1.PWM);
//	BLDC_Commutation_SetPhasePWM(&Motor1.Commutation, Motor1.PWM);



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
void SetPWMMotor1PhaseA(uint16_t pwm) {PWM_1A_PIN_SET_CV(pwm);}
void SetPWMMotor1PhaseB(uint16_t pwm) {PWM_1B_PIN_SET_CV(pwm);}
void SetPWMMotor1PhaseC(uint16_t pwm) {PWM_1C_PIN_SET_CV(pwm);}

#define SetPWMMotor1PhaseAB(pwm) SetPWMMotor1PhaseA(pwm)
#define SetPWMMotor1PhaseAC(pwm) SetPWMMotor1PhaseA(pwm)
#define SetPWMMotor1PhaseBC(pwm) SetPWMMotor1PhaseB(pwm)
#define SetPWMMotor1PhaseBA(pwm) SetPWMMotor1PhaseB(pwm)
#define SetPWMMotor1PhaseCA(pwm) SetPWMMotor1PhaseC(pwm)
#define SetPWMMotor1PhaseCB(pwm) SetPWMMotor1PhaseC(pwm)

void CommutateMotor1PhaseAB(uint16_t pwm) {PWM_1A_PIN_SET_CV(pwm); PWM_1B_PIN_SET_CV(0); EN_1_ALL_PIN_SET(0b011); SE_1_ALL_PIN_SET(0b010); TriggerADCMotor1PhaseC();}
void CommutateMotor1PhaseAC(uint16_t pwm) {PWM_1A_PIN_SET_CV(pwm); PWM_1C_PIN_SET_CV(0); EN_1_ALL_PIN_SET(0b101); SE_1_ALL_PIN_SET(0b100); TriggerADCMotor1PhaseB();}
void CommutateMotor1PhaseBC(uint16_t pwm) {PWM_1B_PIN_SET_CV(pwm); PWM_1C_PIN_SET_CV(0); EN_1_ALL_PIN_SET(0b110); SE_1_ALL_PIN_SET(0b100); TriggerADCMotor1PhaseA();}
void CommutateMotor1PhaseBA(uint16_t pwm) {PWM_1B_PIN_SET_CV(pwm); PWM_1A_PIN_SET_CV(0); EN_1_ALL_PIN_SET(0b011); SE_1_ALL_PIN_SET(0b001); TriggerADCMotor1PhaseC();}
void CommutateMotor1PhaseCA(uint16_t pwm) {PWM_1C_PIN_SET_CV(pwm); PWM_1A_PIN_SET_CV(0); EN_1_ALL_PIN_SET(0b101); SE_1_ALL_PIN_SET(0b001); TriggerADCMotor1PhaseB();}
void CommutateMotor1PhaseCB(uint16_t pwm) {PWM_1C_PIN_SET_CV(pwm); PWM_1B_PIN_SET_CV(0); EN_1_ALL_PIN_SET(0b110); SE_1_ALL_PIN_SET(0b010); TriggerADCMotor1PhaseA();}
void EnableMotor1PhaseABC(void) 			{EN_1_ALL_PIN_SET(0b111); SE_1_ALL_PIN_SET(0b000);}

void FloatMotor1(void) 	{EN_1_ALL_PIN_SET(0b000); SE_1_ALL_PIN_SET(0b000);} // all mosfet off, motor coasts
void ShortMotor1(void) 	{PWM_1C_PIN_SET_CV(0); PWM_1C_PIN_SET_CV(0); PWM_1C_PIN_SET_CV(0); EN_1_ALL_PIN_SET(0b111); SE_1_ALL_PIN_SET(0b000);} // all mosfet top side off, low side on, short motor terminals for dynamic brake

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




void Serial(void)
{
	//Shell_ProcessNonBlocking();
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
//	Test_TxPacketValue
//		(
//			1,
//			2,
//			3,
//			4
//		);
//
//	for (i = 0; i < 18; i++)
//		SendChar(TxPacketBuffer[i]);

}

void PrintDebug(void)
{
	//printf("Loop Time is %d \n", LoopDelta);
	//printf("PWM %d \n", Motor1.PWM);
	printf("BackEMF A ADCU %d \n", *Motor1.BackEMFPhaseA_ADCU);
	printf("BackEMF B ADCU %d \n", *Motor1.BackEMFPhaseB_ADCU);
	//printf("Battery Voltage %d \n", VoltageDivider_GetVoltage(&DividerCommon, *Motor1.VBat_ADCU));
	printf("BackEMF A %d \n", VoltageDivider_GetVoltage(&DividerCommon, *Motor1.BackEMFPhaseA_ADCU));
	printf("BackEMF B %d \n", VoltageDivider_GetVoltage(&DividerCommon, *Motor1.BackEMFPhaseB_ADCU));
	printf("BackEMF Select %d \n", VoltageDivider_GetVoltage(&DividerCommon, *Motor1.BackEMFSelect_ADCU));
	printf("Hall Delta %d \n", Speed_GetDeltaTicks(&Motor1Speed));
	//printf("RPM %d \n", Speed_GetRPM(&Motor1Speed));
	printf("I ADCU %d \n", *Motor1.I_ADCU);

//	printf("Motor Desired %d \n", *Motor1PID.SetPoint);
//	printf("Motor Input %d \n", *Motor1PID.Input);
//	printf("Motor Error (Measured - Input) %d \n", *Motor1PID.SetPoint - *Motor1PID.Input);

	printf("\n");
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

//static uint8_t RxPacketBufferArray[20];
//static uint8_t TxPacketBufferArray[20];

void KESC_Init(void)
{
	Cpu_DisableInt();
	//Hardware init
	//Serial0_Init();
	//Serial0_SetBaudRateMode(Serial0_BM_9600BAUD);
	//Inhr1_SetBaudRateMode(Inhr1_BM_38400BAUD);
	//Inhr1_SetBaudRateMode(Inhr1_BM_115200BAUD);
	ADC_Init();
	SysTick_Init();
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
		FloatMotor1,
		FloatMotor1
	);

	Speed_InitHallEncoder(&Motor1Speed, &FTM1_C1V, FTM1_CV_MAX, FTM1_FREQ, MOTOR1_POLE_PAIRS, 0, PWM_FREQ);

//	PID_Init
//	(
//		&PIDMotor1, Speed_GetPtrEventPeriod(), &Motor1.PIDOutputHallPeriod, &Motor1.PIDSetPointHallPeriod,
//		500, 1, 0,
//		PWM_PEROID, FTM2_FREQ, 10,
//		10, FTM1_CV_MAX
//	);


	//Monitor_Init();
//	Measure_MapAddress(&Motor1.VBat_ADCU, 			ADC_BAT_AD);
//	Measure_MapAddress(&Motor1.BackEMFPhaseA_ADCU, 	ADC_V1A);
//	Measure_MapAddress(&Motor1.BackEMFPhaseB_ADCU, 	ADC_V1B);
//	Measure_MapAddress(&Motor1.BackEMFPhaseC_ADCU, ADC_V1B);
//	Measure_MapAddress(&Motor1.I_ADCU, 				ADC_I1_AD);
//	Measure_MapAddress(&Motor1.LSTemp_ADCU, 		ADC_LSTEMP_AD);
//	Measure_MapAddress(&Motor2.BackEMFPhaseA_ADCU, ADC_V2A);
//	Measure_MapAddress(&Motor2.BackEMFPhaseB_ADCU, ADC_V2B);
//	Measure_MapAddress(&Motor2.BackEMFPhaseC_ADCU, ADC_V2B);
//	Measure_MapAddress(&Motor2.I_ADCU, 			ADC_I2_AD);

	//Map Motor Structs
	BLDC_Init
	(
		&Motor1,
		&Motor1Commutation,
		&Motor1Speed,
		&Motor1PID,
		&DividerCommon,
		&DividerCommon,
		&DividerCommon,
		&DividerCommon,
		FloatMotor1,
		ShortMotor1,
		PWM_MAX
	);

	//Shell_InitNonBlocking(10, SHELL_OUTTER_LOOP_FREQ);
	//RegisterShellCmds();

	FMSTR_Init();

	LiteFXOS_InitMillis(Millis_GetTickCounter());
	LiteFXOS_InitThreadPeriodicArgPeriod(&ThreadLED, 		 	LEDBlink, 		1000);
	//LiteFXOS_InitThreadPeriodicArgPeriod(&ThreadSerial, 		Serial, 		SHELL_OUTTER_LOOP_FREQ);
	LiteFXOS_InitThreadPeriodicArgPeriod(&ThreadTask1Second, 	Task1Second, 	1000);
	LiteFXOS_InitThreadPeriodicArgPeriod(&ThreadComTx, 			ComTx, 			20);

	LiteFXOS_SetThreadStart(&ThreadLED);
	LiteFXOS_SetThreadStart(&ThreadSerial);
	LiteFXOS_SetThreadStart(&ThreadTask1Second);
	LiteFXOS_SetThreadStart(&ThreadComTx);
	Cpu_EnableInt();

	// Temp run
	Commutation_MapCommuntationTableRunCalibration
	(	
		&Motor1Commutation,
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
		CommutateMotor1PhaseAB, 
		CommutateMotor1PhaseAC,
		CommutateMotor1PhaseBC,
		CommutateMotor1PhaseBA,
		CommutateMotor1PhaseCA,
		CommutateMotor1PhaseCB,
		12,
		EnableMotor1PhaseABC,
		Delay,
		1000
	);

//	Test_Init
//	(
//		TxPacketBufferArray,
//		RxPacketBufferArray,
//		Inhr1_GetCharsInRxBuf,
//		Inhr1_RecvChar,
//		Inhr1_SendChar
//	);

	// SendStartMessage();
}

void KESC_Loop(void)
{
	while(1)
	{
		LoopDelta = Micros() - LoopTimer;
		LoopTimer = Micros();

		//__DI();
		//WDOG_Feed();
		//__EI();

		LiteFXOS_ProcThread(&ThreadLED);
		//LiteFXOS_ProcThread(&ThreadSerial);
		LiteFXOS_ProcThread(&ThreadTask1Second);

		BLDC_Process(&Motor1);


		FMSTR_Poll();
		FMSTR_Recorder();

		//Test_StartUp();
		//Test_RxPacket(); ///always receive
		//LiteFXOS_ProcThread(&ThreadComTx); //tx every 20ms
		//		if (Test_RxPacket())
		//		{
		//			Motor1.PID.Kp = Test_GetKp();
		//		}

//		Motor1.PID->Kp = Test_GetKp();
//		Motor1.PID->Ki = Test_GetKi();
//		Motor1.PID->Kd = Test_GetKd();

		//Debugging

	}
}

