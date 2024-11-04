/*
 * Calibration.h
 *
 *  Created on: Feb 26, 2024
 *      Author: harsha
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include <stdint.h>
#include "HWConfig.h"
#include "PositionSensor.h"
#include "MathConstants.h"
#include "MotorSetup.h"
#include "FOC.h"

extern SVPWM svpwm;
extern HW hw;

#define SAMPLES_PER_PPAIR 128
#define CALIB_SAMPLES (SAMPLES_PER_PPAIR* POLEPAIRS)

typedef struct Calibration_Struct{
	float setVoltage,mechSecPerRev,mechSpeedRadSec;
	float loopcounter,time;
	float virtualMechRadians,virtualElecRadians;
	int8_t noOfTurns,CWTurns,CCWTurns;
	int16_t encoderZeros_CW[5],encoderZeros_CCW[5];
	uint16_t finalCW,finalCCW,finalAverage;
	int16_t deltaBtwSides;

	float sampleTime,prevSampleTime;
	uint16_t sampleIndex;
}Calib;

void Calibration_init(Calib *c);
uint8_t Calibration_encoderZeroing(Calib *c,PositionSensor *ps);
void Calibration_saveSettings(Calib *c);

#endif /* CALIBRATION_H_ */
