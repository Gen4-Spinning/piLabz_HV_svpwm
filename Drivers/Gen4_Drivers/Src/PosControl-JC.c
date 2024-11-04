/*
 * PosControl-JC.c
 *
 *  Created on: Oct 29, 2024
 *      Author: harsha
 */


#include "PosControl-JC.h"

void Init_ConstJerk_PosControl(posControlJC *p){
	p->execCallingTime_ms = CALLING_TIME_MS;
	p->execCallingTime_s = p->execCallingTime_ms/1000.0f;
}


void Reset_posControlJC(posControlJC *p){
	p->targetDistanceDeg = 0;
	p->targetTime_ms = 0;

	p->instTime_ms = 0;
	p->instAccel = 0;
	p->instSubInterval=0;
	p->instJerk=0;
	p->instAccel=0;
	p->instVel=0;
	p->instVelRPM=0;
	p->instTheta=0;

	p->subIntervalTime_ms=0;
	for (int i=0;i<9;i++){
		p->subIntervalTime_msArray[i] = 0;
	}
	p->jerkVal=0;
	p->maxVelRad_sec=0;
	p->maxVel_RPM=0;

	p->state = POS_IDLE;
}

void Setup_posControlJC(posControlJC *p,float targetThetaDeg,float targetTime_ms){
	p->targetDistanceDeg = targetThetaDeg;
	p->targetTime_ms = targetTime_ms;

	p->subIntervalTime_ms = targetTime_ms/9.0f;
	p->targetDistanceRad = p->targetDistanceDeg*DEG_TO_RAD;
	float subInterval_s = (float)p->subIntervalTime_ms/1000.0f;
	p->jerkVal = p->targetDistanceRad/(12.0f*subInterval_s*subInterval_s*subInterval_s);

	p->maxVelRad_sec = p->jerkVal * 2.0f * subInterval_s * subInterval_s;
	p->maxVel_RPM = p->maxVelRad_sec * 60.0f / TWO_PI_F;

	//PUT CHECKS HERE

	p->subIntervalTime_msArray[0] = p->subIntervalTime_ms * 1;
	p->subIntervalTime_msArray[1] = p->subIntervalTime_ms * 2;
	p->subIntervalTime_msArray[2] = p->subIntervalTime_ms * 3;
	p->subIntervalTime_msArray[3] = p->subIntervalTime_ms * 4;
	p->subIntervalTime_msArray[4] = p->subIntervalTime_ms * 5;
	p->subIntervalTime_msArray[5] = p->subIntervalTime_ms * 6;
	p->subIntervalTime_msArray[6] = p->subIntervalTime_ms * 7;
	p->subIntervalTime_msArray[7] = p->subIntervalTime_ms * 8;
	p->subIntervalTime_msArray[8] = p->subIntervalTime_ms * 9;

}


void ExecPosTrajectory(posControlJC *p){

	if (p->instTime_ms >= p->targetTime_ms){
		p->state = POS_OVER;
	}

	if (p->state == POS_RUNNING){
		p->instTime_ms += p->execCallingTime_ms;

		if (p->instTime_ms < p->subIntervalTime_msArray[0]){
			p->instJerk = p->jerkVal;
			p->instSubInterval = 1;
		}
		else if (p->instTime_ms < p->subIntervalTime_msArray[1]){
			p->instJerk = 0;
			p->instSubInterval = 2;
		}
		else if (p->instTime_ms < p->subIntervalTime_msArray[2]){
			p->instJerk = -p->jerkVal;
			p->instSubInterval = 3;
		}
		else if (p->instTime_ms < p->subIntervalTime_msArray[3]){
			p->instJerk = 0;
			p->instSubInterval = 4;
		}
		else if (p->instTime_ms < p->subIntervalTime_msArray[4]){
			p->instJerk = 0;
			p->instSubInterval = 5;
		}
		else if (p->instTime_ms < p->subIntervalTime_msArray[5]){
			p->instJerk = 0;
			p->instSubInterval = 6;
		}
		else if (p->instTime_ms < p->subIntervalTime_msArray[6]){
			p->instJerk = -p->jerkVal;
			p->instSubInterval = 7;
		}
		else if (p->instTime_ms < p->subIntervalTime_msArray[7]){
			p->instJerk = 0;
			p->instSubInterval = 8;
		}
		else if (p->instTime_ms < p->subIntervalTime_msArray[8]){
			p->instJerk = p->jerkVal;
			p->instSubInterval = 9;
		}else{
			p->instJerk = 0;
		}
	    p->instAccel = p->instAccel  + p->instJerk*p->execCallingTime_s;
	    p->instVel = p->instVel + p->instAccel*p->execCallingTime_s;
	    p->instTheta = p->instTheta + p->instVel * p->execCallingTime_s;
	    p->instVelRPM = p->instVel * 60.0f/TWO_PI_F;

	}
}
