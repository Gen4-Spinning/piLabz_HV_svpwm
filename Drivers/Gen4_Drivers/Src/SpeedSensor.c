/*
 * SpeedSensor.c
 *
 *  Created on: 25-Apr-2024
 *      Author: harsha
 */
#include "SpeedSensor.h"

void updateSpeedCalc(Speed *s, PositionSensor *ps){
	//get one ms position change
	s->loopCounter ++;

	if (s->loopCounter == 9){
		s->totalDeltaRads = ps->multiturn_mech_radians - s->prevMechRads;
		s->prevMechRads = ps->multiturn_mech_radians;
		s->loopCounter = 0;

		// move the samples one step down
		for (int i=SPEEDSAMPLES-1;i>=1;i--){//i goes from 19 to 1
			s->deltaRadsArr[i]=s->deltaRadsArr[i-1];
			}
		// put in the new samples
		s->deltaRadsArr[0] = s->totalDeltaRads;

		//now calculate the RPM with the new data
		s->tenSampleSpeed = 0;
		s->twentySampleSpeed = 0;
		for (int i=0;i<SPEEDSAMPLES;i++){
			if (i<10){
				s->tenSampleSpeed += s->deltaRadsArr[i];
			}
			s->twentySampleSpeed += s->deltaRadsArr[i];
		}

		s->avgTenSamples = s->tenSampleSpeed/10.0f;
		s->tenSampleRPM = s->avgTenSamples * 1000.0f * 60.0f/TWO_PI_F;
		s->avgTwentySamples = s->twentySampleSpeed/20.0f;
		s->twentySampleRPM = s->avgTwentySamples * 1000.0f * 60.0f /TWO_PI_F;
		s->RPM = s->tenSampleRPM;
	}
}
