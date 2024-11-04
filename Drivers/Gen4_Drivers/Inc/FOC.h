/*
 * FOC.h
 *
 *  Created on: 23-Feb-2024
 *      Author: harsha
 */

#ifndef FOC_H_
#define FOC_H_

#include <stdint.h>
#include "MathConstants.h"
#include "MathOps.h"
#include "Cordic.h"
#include "ADC.h"
#include "HWConfig.h"

extern CORDIC_HandleTypeDef hcordic;
extern TIM_HandleTypeDef htim1;

typedef struct FOC_Struct{
	float U_amps;
	float V_amps;
	float W_amps;
	float U_volts;
	float V_volts;
	float W_volts;
	float IqRef;
	float IdRef;
	float Iq;
	float Id;
	float Vd;
	float Vq;
	float phaseAngle;
	float m;
	float IpkMax;
	float Vpk_Ph_Max;
}FOC;

typedef struct SVPWM_Struct{
	long loopCounter;
	float voltagePercent;
	float encoderAngle;
	float voltageAngle;
	uint8_t sector ;
	float sectorAngle;
	uint16_t PV1,PV2,null,CCR1,CCR2,CCR3;
}SVPWM;

typedef struct HWstate{
	uint8_t tim1PwmHWOn;
}HW;

typedef struct PIDstruct{
	float Kp;
	float Ki;
	float sO;
	float FF;
	float error;
	float integralError;
	float KiTerm;
	float KpTerm;
	float FFTerm;
	float out;
}PID;

void Initialize_FOC_Terms(FOC *foc,float IpkMax);
void FOC_calcSVPWM(SVPWM *foc,float m, float encoderAngle,float deltaTovoltageAngle);
void FOC_applyPWM(SVPWM *foc,uint8_t DT_compensation,uint8_t reversePhases);
void ZeroAllCCRs(SVPWM *foc);

void HW_statesInit(HW *hw);
void StartAllPWM(HW *hw);
void StopAllPWM(HW *hw);

void FOC_updateSensorVals(FOC *foc,RAW_ADC_I_V *adc,RAW_ADC_Averages *adcAvg);

void Init_PID_Terms(PID *pid,float Kp,float Ki,float SO,float FF);
float ExecVoltagePID(PID *pid,float target, float actual,float min,float max);
void getTargetCurrentsDQ(PID *pid,FOC *foc,float target, float actual);
void Zero_PID_Terms(PID *pid);
void getVq(FOC *foc,PID *pid);
void getVd(FOC *foc,PID *pid);
#endif /* FOC_H_ */
