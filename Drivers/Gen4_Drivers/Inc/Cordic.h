/*
 * Cordic.h
 *
 *  Created on: Feb 22, 2024
 *      Author: harsha
 */

#ifndef CORDIC_H_
#define CORDIC_H_

#include <stdint.h>
#include "main.h"

#define SINE_FUNCTION 1
#define PHASE_FUNCTION 2
#define SINE_SCALE 0
//ATAN stuff
#define SCALE_MULIPLIER_INPUT 0.0078125f //2^-7
#define SCALE_MULIPLIER_OUTPUT 128.0f //2^-7

#define CORDIC_ATAN_OUT_OF_RANGE 999

void Cordic_setup(CORDIC_HandleTypeDef hcordic,CORDIC_ConfigTypeDef sCordicConfig);
void RunCordic_Sine(CORDIC_HandleTypeDef hcordic,float theta, float *cos_out, float *sin_out);
void RunCordic_TwoSines(CORDIC_HandleTypeDef hcordic,float theta1,float theta2, float *sin1,float *sin2);

void RunCordic_Phase(CORDIC_HandleTypeDef hcordic,float Vd, float Vq,float *theta,float *mod);

void DQ_Transform(CORDIC_HandleTypeDef hcordic,float a,float b,float c,float rotAngle,float *wd,float *wq);
void ABC_Transform(CORDIC_HandleTypeDef hcordic,float d,float q,float rotAngle,float *a,float *b,float *c);

#endif /* GEN4_DRIVERS_INC_CORDIC_H_ */
