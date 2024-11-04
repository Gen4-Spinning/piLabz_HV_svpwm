
/*
 * CoggingFrictionRemoval.h
 *
 *  Created on: Nov 1, 2024
 *      Author: harsha
 */

#ifndef COGGINGFRICTIONREMOVAL_H_
#define COGGINGFRICTIONREMOVAL_H_

#include "stdint.h"
#include "HWConfig.h"
#include "MathConstants.h"

#define DIVISION_RIGHT_SHIFT 9 //to make 16384=32, divide by 512 (2^9)
#define COGGING_ARR_SIZE 64

typedef struct friction_type{
	uint8_t frictionCompensationOn;
	uint16_t maxFrictionPWM;
	float maxFrictionPercentage;

	uint16_t inst_encoderVal;
	uint8_t  inst_idx;
	float 	inst_frictionNorm;
	float  inst_frictionAddition;
}friction;

typedef struct cogging_type{
	uint8_t coggingCompensationOn;
	uint16_t maxCoggingPWM;
	float maxCoggingPercentage;

	float inst_elecRadians;
	uint8_t  inst_idx;
	float 	inst_coggingNorm;
	float  inst_coggingAddition;
}cogging;

void setMaxFrictionPWM(friction *f,uint16_t pwm);
void lookupFrictionAddition(friction *f,uint16_t encoder_raw);

void setMaxCoggingPWM(cogging *c,uint16_t pwm);
void lookupCoggingAddition(cogging *c,float elecRadians);

#endif /* COGGINGFRICTIONREMOVAL_H_ */
