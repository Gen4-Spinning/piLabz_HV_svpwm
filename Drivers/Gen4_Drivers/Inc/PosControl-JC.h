/*
 * PosControl-JC.h
 *
 *  Created on: Oct 29, 2024
 *      Author: harsha
 */

#ifndef POSCONTROL_JC_H_
#define POSCONTROL_JC_H_

#include "stdint.h"
#include "MathConstants.h"

#define CALLING_TIME_MS 1


#define POS_RUNNING 1
#define POS_OVER 2
#define POS_IDLE 3


typedef struct posControlJCtype{
	float targetDistanceDeg;
	float targetTime_ms;
	float targetDistanceRad;

	float execCallingTime_ms;
	float execCallingTime_s;

	uint16_t subIntervalTime_ms;
	float jerkVal;

	uint16_t subIntervalTime_msArray[9];
	float maxVelRad_sec;
	float maxVel_RPM;

	uint8_t state;

	float instTime_ms;
	float instSubInterval;
	float instJerk;
	float instAccel;
	float instVel;
	float instVelRPM;
	float instTheta;


}posControlJC;

void Init_ConstJerk_PosControl(posControlJC *p);
void Reset_posControlJC(posControlJC *p);
void Setup_posControlJC(posControlJC *p,float targetThetaDeg,float targetTime_ms);
void ExecPosTrajectory(posControlJC *p);


#endif /* POSCONTROL_JC_H_ */
