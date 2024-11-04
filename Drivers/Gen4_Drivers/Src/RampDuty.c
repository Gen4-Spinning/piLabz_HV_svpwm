/*
 * Ramp.c
 *
 *  Created on: Feb 26, 2023
 *      Author: harsha
 */

#include "Ramp.h"


void InitRampDutyStruct(RampDuty *ramp,uint16_t targetDuty,long rampUpTime,long rampDownTime,int16_t rampSteadyTime){
	uint16_t totalSteps  = 0;
	ramp->ramp_callingTime_s = 0.020f;
	ramp->rampUpTime_ms = rampUpTime;
	ramp->rampDownTime_ms = rampDownTime;
	ramp->steadyRunTime_s = rampSteadyTime; // in Seconds!
	ramp->finalTargetDuty = targetDuty;
	ramp->currentDutyF = 0;
	ramp->currentDuty = 0;
	//For RampUp
	totalSteps = ramp->rampUpTime_ms/((uint16_t)(ramp->ramp_callingTime_s*1000));
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dDuty_F_RU = ((float)ramp->finalTargetDuty)/totalSteps;

	//For RampDown
	totalSteps = ramp->rampDownTime_ms/((uint16_t)(ramp->ramp_callingTime_s*1000));
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dDuty_F_RD = ((float)ramp->finalTargetDuty)/totalSteps;

	ramp->rampPhase = RAMP_WAIT;

	ramp-> transitionTarget = 0;
	ramp-> transitionTime_ms = 0;
	ramp-> dDuty_F_transition = 0;

	ramp->rampTimer = 0;
}


void StartRampDuty(RampDuty *ramp){
	ramp->rampPhase = RAMP_UP;
}

void StartRampDownDuty(RampDuty *ramp){
	ramp->rampPhase = RAMP_DOWN;
	ramp->rampTimer = 0;
}

void StopRampDuty(RampDuty *ramp){
	ramp->rampPhase = RAMP_WAIT;
	ramp->currentDutyF = 0;
	ramp->currentDuty = 0;
	ramp->rampTimer = 0;
}

void ResetRampDuty(RampDuty *ramp){
	uint16_t totalSteps  = 0;
	ramp->rampUpTime_ms = 10;
	ramp->rampDownTime_ms = 10;
	ramp->steadyRunTime_s = 30; // in Seconds!
	ramp->finalTargetDuty = 500;
	ramp->currentDutyF = 0;
	ramp->currentDuty = 0;
	ramp->ramp_callingTime_s = 0.020f;

	//For RampUp
	totalSteps = ramp->rampUpTime_ms/((uint16_t)(ramp->ramp_callingTime_s*1000));
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dDuty_F_RU = ((float)ramp->finalTargetDuty)/totalSteps;

	//For RampDown
	totalSteps = ramp->rampDownTime_ms/((uint16_t)(ramp->ramp_callingTime_s*1000));
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dDuty_F_RD = ((float)ramp->finalTargetDuty)/totalSteps;

	ramp->rampPhase = RAMP_WAIT;

	ramp->rampTimer = 0;
}


void ExecRampDuty(RampDuty *ramp){

	if(ramp->rampPhase != RAMP_WAIT){
		ramp->rampTimer += 	ramp->ramp_callingTime_s;
	}

	if (ramp->rampPhase == RAMP_UP){
		if(ramp->currentDuty <= ramp->finalTargetDuty){
			ramp->currentDutyF += ramp->dDuty_F_RU;
			ramp->currentDuty = (uint16_t)(ramp->currentDutyF);
			if(ramp->currentDuty >= ramp->finalTargetDuty){
				ramp->currentDuty  = ramp->finalTargetDuty;
				ramp->rampPhase = RAMP_STEADY;
				ramp->rampTimer = 0;
			}
		}
	}

	else if (ramp->rampPhase == RAMP_STEADY){
		if (ramp->steadyRunTime_s != RUN_FOREVER){
			if (ramp->rampTimer >= ramp->steadyRunTime_s){
				ramp->rampPhase = RAMP_DOWN;
				ramp->rampTimer = 0;
			}
		}
	}

	else if (ramp->rampPhase == RAMP_CHANGE){
		if(ramp->currentDuty < ramp->transitionTarget){
			ramp->currentDutyF += ramp->dDuty_F_transition;
			ramp->currentDuty = (uint16_t)(ramp->currentDutyF);
			if(ramp->currentDuty >= ramp->transitionTarget){
				ramp->currentDuty  = ramp->transitionTarget;
				ramp->rampPhase = RAMP_STEADY;
				ramp->rampTimer = 0;
			}
		}
		if(ramp->currentDuty > ramp->transitionTarget){
			ramp->currentDutyF -= ramp->dDuty_F_transition;
			ramp->currentDuty = (uint16_t)(ramp->currentDutyF);
			if(ramp->currentDuty <= ramp->transitionTarget){
				ramp->currentDuty  = ramp->transitionTarget;
				ramp->rampPhase = RAMP_STEADY;
				ramp->rampTimer = 0;
			}
		}
	}

	else if (ramp->rampPhase == RAMP_DOWN){
		if(ramp->currentDuty >= 0){
			ramp->currentDutyF -= ramp->dDuty_F_RD;
			ramp->currentDuty = (uint16_t)(ramp->currentDutyF);
			if(ramp->currentDuty <= 0){
				ramp->currentDuty  = 0;
				ramp->rampPhase = RAMP_WAIT;
				ramp->rampTimer = 0;
			}
		}
	}
	else{
		//Do Nothing
	}

}


void  ChangeDuty(RampDuty *r){
	uint16_t deltaDuty = 0;
	uint16_t totalSteps = 0;
	if (r->transitionTarget > r->currentDuty){
		deltaDuty = r->transitionTarget - r->currentDuty;
	}else if (r->transitionTarget < r->currentDuty){
		deltaDuty = r->currentDuty - r->transitionTarget;
	}else{
		deltaDuty = 0;
	}
	totalSteps = r->transitionTime_ms/((uint16_t)(r->ramp_callingTime_s*1000));
	if (totalSteps == 0){
		totalSteps = 1;
	}
	r->dDuty_F_transition = (float)deltaDuty/totalSteps;
}

// when we get a change Duty, we want to do ramp UP/ramp Down times with that
// new target.
void Recalculate_RampDuty_RampRates(RampDuty *ramp,uint16_t newTarget){
	uint16_t totalSteps = 0;
	//For RampUp
	ramp->finalTargetDuty = newTarget;
	totalSteps = ramp->rampUpTime_ms/((uint16_t)(ramp->ramp_callingTime_s*1000));
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dDuty_F_RU = ((float)ramp->finalTargetDuty)/totalSteps;

	//For RampDown
	totalSteps = ramp->rampDownTime_ms/((uint16_t)(ramp->ramp_callingTime_s*1000));
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dDuty_F_RD = ((float)ramp->finalTargetDuty)/totalSteps;
}

