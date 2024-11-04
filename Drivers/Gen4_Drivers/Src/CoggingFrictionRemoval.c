/*
 * CoggingFrictionRemoval.c
 *
 *  Created on: Nov 1, 2024
 *      Author: harsha
 */

#include "CoggingFrictionRemoval.h"

float frictionNormalized[32] = {0.012f, 0.0f,0.008f,0.036f,0.084f,0.148f,
								0.227f, 0.316f,0.414f,0.515f,0.615f,0.711f,
								0.798f, 0.872f, 0.932f, 0.974f, 0.996f, 0.999f,
								0.98f, 0.942f, 0.886f, 0.815f, 0.73f, 0.636f,
								0.536f,0.435f,0.337f, 0.245f, 0.164f, 0.096f,
								0.045f, 0.012f};
float coggingNormalized[64] = {
		0.674f, 0.918f, 0.861f, 0.781f, 0.667f, 0.367f, 0.201f, 0.203f, 0.28f,
		 0.538f, 0.826f, 0.944f, 0.913f, 0.833f, 0.675f, 0.418f, 0.145f, 0.067f,
		 0.116f, 0.238f, 0.478f, 0.7f, 0.724f, 0.669f, 0.584f, 0.35f, 0.11f,
		 0.039f, 0.06f, 0.219f, 0.439f, 0.703f, 0.772f, 0.706f, 0.655f, 0.495f,
		 0.267f, 0.12f, 0.125f, 0.24f, 0.416f, 0.704f, 0.975f, 1.033f, 0.935f,
		 0.856f, 0.715f, 0.496f, 0.228f, 0.048f, 0.016f, 0.117f, 0.304f, 0.548f,
		 0.591f, 0.555f, 0.493f, 0.327f, 0.1f, 0.0f, 0.059f, 0.175f, 0.441f,
		 0.561f
};


void setMaxFrictionPWM(friction *f,uint16_t pwm){
	f->maxFrictionPWM = pwm;
	f->maxFrictionPercentage = f->maxFrictionPWM /TIMER1_ARR;
}

void lookupFrictionAddition(friction *f,uint16_t encoder_raw){
	//divide encoder raw
	f->inst_encoderVal = encoder_raw;
	f->inst_idx =f->inst_encoderVal>>DIVISION_RIGHT_SHIFT;
	f->inst_frictionNorm = frictionNormalized[f->inst_idx];
	f->inst_frictionAddition = f->inst_frictionNorm * f->maxFrictionPercentage;
}

void setMaxCoggingPWM(cogging *c,uint16_t pwm){
	c->maxCoggingPWM = pwm;
	c->maxCoggingPercentage = c->maxCoggingPWM /TIMER1_ARR;
}

void lookupCoggingAddition(cogging *c,float elecRadians){
	//divide encoder raw
	c->inst_elecRadians = elecRadians;
	c->inst_idx = c->inst_elecRadians/TWO_PI_F * COGGING_ARR_SIZE ;
	c->inst_coggingNorm = coggingNormalized[c->inst_idx];
	c->inst_coggingAddition = c->inst_coggingNorm * c->maxCoggingPercentage;
}
