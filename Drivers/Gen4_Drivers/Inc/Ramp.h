/*
 * Ramp.h
 *
 *  Created on: 29-Feb-2024
 *      Author: harsha
 */

#ifndef GEN4_DRIVERS_INC_RAMP_H_
#define GEN4_DRIVERS_INC_RAMP_H_

#include "stdio.h"
#include "stdint.h"

#define RUN_FOREVER -1

#define RAMP_UP 0
#define RAMP_DOWN 1
#define RAMP_STEADY 2
#define RAMP_OVER 3
#define RAMP_WAIT 4
#define RAMP_CHANGE 8

typedef struct RampDutyStruct {
  uint16_t finalTargetDuty;
  uint16_t currentDuty;
  float currentDutyF;
  float dDuty_F_RU;
  float dDuty_F_RD;
  int16_t steadyRunTime_s;
  long rampUpTime_ms;
  long rampDownTime_ms;
  uint8_t rampPhase;
  uint16_t transitionTarget;
  uint16_t transitionTime_ms;
  float dDuty_F_transition;

  float ramp_callingTime_s;
  float rampTimer;
}RampDuty;


typedef struct RampRPMStruct {
  uint16_t finalTargetRPM ;
  float instTargetRPM_F;

  float dRPM_F_RU;
  float dRPM_F_RD;

  uint8_t rampPhase;
  uint16_t transitionTarget;
  float transitionTime_s;
  float dRPM_F_transition;

  float steadyRunTime_s; // runTime is -1 for Forever (normal mode)
  float rampUpTime_s;
  float rampDownTime_s;
  float ramp_callingTime_s;
  float rampTimer;

}RampRPM;

void InitRampDutyStruct(RampDuty *ramp,uint16_t targetDuty,long rampUpTime,long rampDownTime,int16_t rampSteadyTime);
void StartRampDuty(RampDuty *ramp);
void StartRampDownDuty(RampDuty *ramp);
void StopRampDuty(RampDuty *ramp);
void ResetRampDuty(RampDuty *ramp);
void ExecRampDuty(RampDuty *ramp);
void ChangeDuty(RampDuty *r);
void Recalculate_RampDuty_RampRates( RampDuty *ramp,uint16_t newTarget);

void InitRampRPMStruct(RampRPM *ramp,uint16_t targetRPM,float rampUpTime,float rampDownTime,float rampSteadyTime);
void StartRampRPM(RampRPM *ramp);
void StopRampRPM(RampRPM *ramp);
void StartRampDownRPM(RampRPM *ramp);
void ResetRampRPM(RampRPM *ramp);
void IdleRampRPM(RampRPM *ramp);
void ExecRampRPM(RampRPM *ramp);
void ChangeRPM(RampRPM *r,uint16_t transitionTarget,float transitionTime_s);
void Recalculate_RampRPM_RampRates(RampRPM *ramp,uint16_t newTarget);




#endif /* GEN4_DRIVERS_INC_RAMP_H_ */
