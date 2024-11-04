/*
 * PositionControl.h
 *
 *  Created on: 30-Apr-2024
 *      Author: harsha
 */

#ifndef POSITIONCONTROL_H_
#define POSITIONCONTROL_H_

#include "stdio.h"
#include "MathConstants.h"
#include "HWConfig.h"

#define MIN_VELOCITY 0.005f //in rads/sec

#define VELOCITY_RAMPUP 1
#define VELOCITY_CRUISE 2
#define VELOCITY_RAMPDOWN 3
#define VELOCITY_PAUSE 4
#define POS_LOOP_OVER 5
#define POS_LOOP_IDLE 6
#define POS_CYCLES_OVER 7

typedef struct PosRampStruct{

	float Revs;
	float targetRads;
	float targetRPM;
	float rampTime_ms;
	float controlLoopTime_ms;

	float timeForMove_s;
	float timeSteadyState_s;

	float dist_Ramps_rads;
	float dist_cruisePhase_rads;

	float pkVelocity_radSec;
	float dV_RU;
	float dS_cruise;

	float endDist_cruise;
	float endDist_RD;

	float currentRad;
	float currentVelocity_radSec;

	char  rampPhase;
}PosRamp;

void posRamp_Reset(PosRamp *p);
void posRamp_SetupMove(PosRamp *p,float Revs, float targetRPM,float rampTime_ms);
void posRamp_Start(PosRamp *p);
void posRamp_Stop(PosRamp *p);
void posRamp_Exec(PosRamp *p);

#endif /* POSITIONCONTROL_H_ */
