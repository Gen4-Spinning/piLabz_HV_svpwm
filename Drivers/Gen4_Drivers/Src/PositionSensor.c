/*
 * PositionSensor.c
 *
 *  Created on: Feb 21, 2024
 *      Author: harsha
 */

#include "PositionSensor.h"
#include "EncoderFns.h"


void resetPositionSensor(PositionSensor *ps){
	for (int i=15;i>=0;i--){
			ps->multiTurn_position[i]=0;
		}
	ps->turns = 0;
	ps->prev_mechRadians_singleTurn =0 ;
	ps->mechRadians_singleTurn = 0;
	ps->multiturn_mech_radians = 0;

	ps->avgIdx = 0;
	ps->avgingStarted = 0;
	ps->avg_velocity_radsec = 0;
}


void PositionSensor_update(PositionSensor* ps,float dt){
	//relatively slow, 500 cycles
	for (int i=15;i>=1;i--){
		ps->multiTurn_position[i]=ps->multiTurn_position[i-1];
	}

	// this takes the most time -> 1800 cycles with continuous read, twice that for
	//discontinuous. speed this up with your own SPI code. If we want to check health
	//of the chip we need to restart the continuous read. for that this fn needs to
	//be modified.
	ps->encoder_raw = Encoder_get16BitMechAngle_Single(1);
	ps->encoder_raw = ps->encoder_raw>> 3;
	//linearize, calibrate if you need to
	//calculations are very fast, less than 500cycles.
	ps->mechRadians_singleTurn = ps->encoder_raw*TWO_PI_F/CPR_ENCODER;
	ps->elecRadians  = getElecAngleFromMechAngle(ps->mechRadians_singleTurn); 	//get electrical rads from mech rads

	ps->delta_mechRadians_dt = ps->mechRadians_singleTurn - ps->prev_mechRadians_singleTurn;
	int8_t rollover = 0;
	if(ps->delta_mechRadians_dt > PI_F){rollover = -1;}
	else if(ps->delta_mechRadians_dt < -PI_F){rollover = 1;}
	ps->turns += rollover;

	ps->multiturn_mech_radians = ps->turns * TWO_PI_F + ps->mechRadians_singleTurn;
	ps->prev_mechRadians_singleTurn = ps->mechRadians_singleTurn;

	ps->multiTurn_position[0] = ps->multiturn_mech_radians;

	ps->velocity_radsec = (ps->multiTurn_position[0] - ps->multiTurn_position[POS_SAMPLES-1])/(dt*(float)(POS_SAMPLES-1));


	ps->velocityRPM = ps->velocity_radsec * 60.0f/TWO_PI_F;
	ps->elec_velocity_radsec = POLE_PAIRS*ps->velocity_radsec;

}


void getAveragedVelocityRadSec(PositionSensor *ps){
	//averaging
	if (ps->avgingStarted==0){
		ps->avgIdx ++;
		ps->avg_velocity_radsec = 0 ;
		if (ps->avgIdx >= 30){
			ps->avgingStarted = 1;
		}
	}
	else{
		ps->avg_velocity_radsec = ps->avg_velocity_radsec + (ps->velocity_radsec - ps->avg_velocity_radsec)/EXP_FILTER_FACTOR;
	}
}

float getElecAngleFromMechAngle(float mechRadians){
	int multiplier = (int)(mechRadians/MECH_RADS_PER_ELECTRICAL_REV);
	float delta_mechRadians = mechRadians - (multiplier * MECH_RADS_PER_ELECTRICAL_REV);
	float elecRadians = (delta_mechRadians * TWO_PI_F)/MECH_RADS_PER_ELECTRICAL_REV;
	return elecRadians;
}
