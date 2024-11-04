/*
 * SpeedSensor.h
 *
 *  Created on: 25-Apr-2024
 *      Author: harsha
 */

#ifndef SPEEDSENSOR_H_
#define SPEEDSENSOR_H_

#include "PositionSensor.h"

#define SPEEDSAMPLES 20
typedef struct SpeedSensor{
	float deltaRadsArr[SPEEDSAMPLES];

	long loopCounter;
	float prevMechRads;
	float deltaMechRads;
	float totalDeltaRads;

	float tenSampleSpeed;
	float twentySampleSpeed;
	float avgTenSamples;
	float avgTwentySamples;
	float tenSampleRPM;
	float twentySampleRPM;
	float RPM;
}Speed;

void updateSpeedCalc(Speed *s, PositionSensor *ps);

#endif /* GEN4_DRIVERS_INC_SPEEDSENSOR_H_ */
