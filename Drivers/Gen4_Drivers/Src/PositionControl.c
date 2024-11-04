/*
 * PosCntrl.c
 *
 *  Created on: Mar 25, 2023
 *      Author: harsha
 */

#include "stdio.h"
#include "PositionControl.h"

void posRamp_Reset(PosRamp *p){

	p->Revs = 0;
	p->targetRads = 0;
	p->targetRPM = 0;
	p->rampTime_ms= 0;
	p->controlLoopTime_ms = 1;

	p->timeForMove_s = 0;
	p->timeSteadyState_s = 0;

	p->dist_Ramps_rads = 0;
	p->dist_cruisePhase_rads = 0;

	p->pkVelocity_radSec = 0;
	p->dV_RU = 0;
	p->dS_cruise = 0;

	p->endDist_cruise = 0;
	p->endDist_RD = 0;

	p->currentRad = 0;
	p->currentVelocity_radSec = 0;

	p->rampPhase = POS_LOOP_OVER;
}


//Trapezoidal Trajectory ( No Jerk Control - No S profile )
void posRamp_SetupMove(PosRamp *p,float Revs, float targetRPM,float rampTime_ms){

	p->Revs = Revs;
	p->targetRads = p->Revs * TWO_PI_F;
    p->targetRPM = targetRPM;
    p->rampTime_ms = rampTime_ms;
    p->controlLoopTime_ms = 1;//ms in a different loop

    p->timeForMove_s = p->Revs/(p->targetRPM /60.0f);
	p->timeSteadyState_s = p->timeForMove_s - (p->rampTime_ms * 2.0f/1000.0);

	float temp = (2.0f*0.5f*p->rampTime_ms/1000.0f) + p->timeSteadyState_s;
	p->pkVelocity_radSec = p->targetRads/temp;

	long rampSteps = p->rampTime_ms/p->controlLoopTime_ms;
	long cruise_steps = p->timeSteadyState_s*1000.0f/p->controlLoopTime_ms;

	p->dist_Ramps_rads = 0.5 * p->pkVelocity_radSec * p->rampTime_ms/1000.0f;
	p->dist_cruisePhase_rads = p->targetRads - 2.0f*p->dist_Ramps_rads  ;

	p->dV_RU = p->pkVelocity_radSec/rampSteps;
	p->dS_cruise = p->dist_cruisePhase_rads/cruise_steps;

	p->endDist_cruise = p->dist_cruisePhase_rads + p->dist_Ramps_rads;
	p->endDist_RD = p->targetRads;

}

void posRamp_Start(PosRamp *p){
	p->rampPhase = VELOCITY_RAMPUP;
}
void posRamp_Stop(PosRamp *p){
	p->rampPhase = POS_LOOP_OVER;
}

void posRamp_Exec(PosRamp *p){
	if (p->rampPhase == VELOCITY_RAMPUP){
		if(p->currentVelocity_radSec < p->pkVelocity_radSec){
			p->currentVelocity_radSec += p->dV_RU;
			if (p->currentVelocity_radSec >= p->pkVelocity_radSec){
				p->currentVelocity_radSec = p->pkVelocity_radSec;
				p->rampPhase =  VELOCITY_CRUISE;
			}
			float dS_RU =p->currentVelocity_radSec * p->controlLoopTime_ms/1000.0;
			p->currentRad += dS_RU;
		}
	}
	else if (p->rampPhase == VELOCITY_CRUISE){
		if (p->currentRad < p->endDist_cruise){
			p->currentRad += p->dS_cruise;
			if (p->currentRad >= p->endDist_cruise){
				p->currentRad = p->endDist_cruise;
				p->rampPhase =  VELOCITY_RAMPDOWN;
			}
		}
	}
	else if (p->rampPhase == VELOCITY_RAMPDOWN){
			p->currentVelocity_radSec -= p->dV_RU;
			if (p->currentVelocity_radSec <= MIN_VELOCITY){
				p->currentVelocity_radSec = MIN_VELOCITY;
			}
			float dS_RD =p->currentVelocity_radSec * p->controlLoopTime_ms/1000.0;
			p->currentRad += dS_RD;
			if (p->currentRad >= p->endDist_RD){
				p->rampPhase =  POS_LOOP_OVER;
			}
		}
	else{
	}
}
