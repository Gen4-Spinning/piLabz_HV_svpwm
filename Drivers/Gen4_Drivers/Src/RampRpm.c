/*
 * Ramp.c
 *
 *  Created on: Feb 26, 2023
 *      Author: harsha
 */

#include "Ramp.h"

void InitRampRPMStruct(RampRPM *ramp,uint16_t targetRPM,float rampUpTime,float rampDownTime,float rampSteadyTime){
	uint16_t totalSteps  = 0;
	ramp->ramp_callingTime_s = 0.020f;
	ramp->rampUpTime_s = rampUpTime;
	ramp->rampDownTime_s = rampDownTime;
	ramp->steadyRunTime_s = rampSteadyTime;

	ramp->finalTargetRPM = targetRPM;
	ramp->instTargetRPM_F = 0;
	//For RampUp
	totalSteps = (uint16_t)(ramp->rampUpTime_s/ramp->ramp_callingTime_s);
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dRPM_F_RU = ((float)ramp->finalTargetRPM)/totalSteps;

	//For RampDown
	totalSteps = (uint16_t)(ramp->rampDownTime_s/ramp->ramp_callingTime_s);
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dRPM_F_RD = ((float)ramp->finalTargetRPM)/totalSteps;

	ramp->rampPhase = RAMP_WAIT;
	ramp->rampTimer = 0;

	ramp-> transitionTarget = 0;
	ramp-> transitionTime_s = 0;
	ramp-> dRPM_F_transition = 0;

}
void StartRampRPM(RampRPM *ramp){
	ramp->rampPhase = RAMP_UP;
}

void StartRampDownRPM(RampRPM *ramp){
	ramp->rampPhase = RAMP_DOWN;
}

void StopRampRPM(RampRPM *ramp){
	ramp->rampPhase = RAMP_WAIT;
}

void IdleRampRPM(RampRPM *ramp){
	ramp->rampPhase = RAMP_WAIT;
	ramp->instTargetRPM_F = 0;
}

void ResetRampRPM(RampRPM *ramp){
	//default safe vales
	uint16_t totalSteps  = 0;
	ramp->ramp_callingTime_s = 0.020f;
	ramp->rampUpTime_s = 10;
	ramp->rampDownTime_s = 10;
	ramp->steadyRunTime_s = 30; // in Seconds!
	ramp->finalTargetRPM = 500;
	ramp->instTargetRPM_F = 0;
	//For RampUp
	totalSteps = (uint16_t)(ramp->rampUpTime_s/ramp->ramp_callingTime_s);
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dRPM_F_RU = ((float)ramp->finalTargetRPM)/totalSteps;

	//For RampDown
	totalSteps = (uint16_t)(ramp->rampDownTime_s/ramp->ramp_callingTime_s);
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dRPM_F_RD = ((float)ramp->finalTargetRPM)/totalSteps;

	ramp->rampPhase = RAMP_WAIT;
	ramp->rampTimer = 0;
}


void ExecRampRPM(RampRPM *ramp){
	if(ramp->rampPhase != RAMP_WAIT){
		ramp->rampTimer += 	ramp->ramp_callingTime_s;
	}

	if (ramp->rampPhase == RAMP_UP){
		if(ramp->instTargetRPM_F <= ramp->finalTargetRPM){
			ramp->instTargetRPM_F += ramp->dRPM_F_RU;

			if(ramp->instTargetRPM_F >= ramp->finalTargetRPM){
				ramp->instTargetRPM_F  = ramp->finalTargetRPM;
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
			if(ramp->instTargetRPM_F < ramp->transitionTarget){
				ramp->instTargetRPM_F += ramp->dRPM_F_transition;
				if(ramp->instTargetRPM_F >= ramp->transitionTarget){
					ramp->instTargetRPM_F  = ramp->transitionTarget;
					ramp->rampPhase = RAMP_STEADY;
					ramp->rampTimer = 0;
				}
			}
			if(ramp->instTargetRPM_F > ramp->transitionTarget){
				ramp->instTargetRPM_F -= ramp->dRPM_F_transition;
				if(ramp->instTargetRPM_F <= ramp->transitionTarget){
					ramp->instTargetRPM_F  = ramp->transitionTarget;
					ramp->rampPhase = RAMP_STEADY;
					ramp->rampTimer = 0;
				}
			}
		}

	else if (ramp->rampPhase == RAMP_DOWN){
		if(ramp->instTargetRPM_F >= 0){
			ramp->instTargetRPM_F -= ramp->dRPM_F_RD;
			if(ramp->instTargetRPM_F <= 0){
				ramp->instTargetRPM_F  = 0;
				ramp->rampPhase = RAMP_WAIT;
				ramp->rampTimer = 0;
			}
		}
	}

}

void  ChangeRPM(RampRPM *r,uint16_t transitionTarget,float transitionTime_s){
	uint16_t deltaRPM = 0;
	uint16_t totalSteps = 0;
	r->transitionTarget = transitionTarget;
	r->transitionTime_s = transitionTime_s;
	r->finalTargetRPM = r->transitionTarget;
	if (r->transitionTarget > r->instTargetRPM_F){
		deltaRPM = r->transitionTarget - r->instTargetRPM_F;
	}else if (r->transitionTarget < r->instTargetRPM_F){
		deltaRPM = r->instTargetRPM_F - r->transitionTarget;
	}else{
		deltaRPM = 0;
	}
	totalSteps = r->transitionTime_s/r->ramp_callingTime_s;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	r->dRPM_F_transition = ((float)deltaRPM)/totalSteps;
}

// when we get a change RPM for the bobbin, we want to do ramp UP/ramp Down times with that
// new target.(this is used to ramo down to zero, and ramp up from zero)
void Recalculate_RampRPM_RampRates(RampRPM *ramp,uint16_t newTarget){
	uint16_t totalSteps = 0;
	//For RampUp
	ramp->finalTargetRPM = newTarget;
	totalSteps = ramp->rampUpTime_s/ramp->ramp_callingTime_s;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dRPM_F_RU = ((float)ramp->finalTargetRPM)/totalSteps;

	//For RampDown
	totalSteps = ramp->rampDownTime_s/ramp->ramp_callingTime_s;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dRPM_F_RD = ((float)ramp->finalTargetRPM)/totalSteps;
}
