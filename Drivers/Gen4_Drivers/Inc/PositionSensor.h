/*
 * PositionSensor.h
 *
 *  Created on: Feb 21, 2024
 *      Author: harsha
 */

#ifndef POSITIONSENSOR_H_
#define POSITIONSENSOR_H_

#include <stdint.h>
#include "MathConstants.h"


#define POS_SAMPLES 16
#define POLE_PAIRS 3.0f
#define CPR_ENCODER 2048;//16384;//2048.0f
#define EXP_FILTER_FACTOR 5

typedef struct PositionSensorStruct{
	float multiTurn_position[POS_SAMPLES];

	uint16_t encoder_raw;

	float mechRadians_singleTurn;
	float prev_mechRadians_singleTurn;
	float delta_mechRadians_dt;

	float multiturn_mech_radians;
	float elecRadians;

	float velocity_radsec;

	uint8_t avgIdx;
	uint8_t avgingStarted;
	float avg_velocity_radsec;


	float velocityRPM;
	float elec_velocity_radsec;

	int16_t turns;

}PositionSensor;

void resetPositionSensor(PositionSensor *ps);
void PositionSensor_update(PositionSensor* ps,float dt);
void getAveragedVelocityRadSec(PositionSensor *ps);
float getElecAngleFromMechAngle(float mechRadians);

#endif /* POSITIONSENSOR_H_ */
