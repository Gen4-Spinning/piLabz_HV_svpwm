/*
 * Cordic.c
 *
 *  Created on: Feb 22, 2024
 *      Author: harsha
 */

#include "MathConstants.h"
#include "Cordic.h"

void Cordic_setup(CORDIC_HandleTypeDef hcordic,CORDIC_ConfigTypeDef sCordicConfig){
	sCordicConfig.Function         = CORDIC_FUNCTION_SINE;     /* sine function */
	sCordicConfig.Precision        = CORDIC_PRECISION_6CYCLES; /* 1.15 -> 2^-7 residual error  */
	sCordicConfig.Scale            = CORDIC_SCALE_0;           /* no scale */
	sCordicConfig.NbWrite          = CORDIC_NBWRITE_1;         /* One input data: angle. Second input data (modulus) is 1 after cordic reset */
	sCordicConfig.NbRead           = CORDIC_NBREAD_1;          /* One output data: sine */
	sCordicConfig.InSize           = CORDIC_INSIZE_16BITS;     /* q1.15 format for input data */
	sCordicConfig.OutSize          = CORDIC_OUTSIZE_16BITS;    /* q1.15 format for output data */
	HAL_CORDIC_Configure(&hcordic, &sCordicConfig);
}

void RunCordic_Phase(CORDIC_HandleTypeDef hcordic,float Vd, float Vq,float *theta,float *mod){
	// this function gives atan2
	MODIFY_REG(hcordic.Instance->CSR,CORDIC_CSR_FUNC,PHASE_FUNCTION);

	short Vd_in = Vd * 32768;
	short Vq_in = Vq * 32768;
	int32_t cordicin =  Vd_in << 16 ;  // this is second argument y
	cordicin += Vq_in; //this is first argument x

	// and result is atan2(y,x)

	CORDIC->WDATA = cordicin;
	int32_t out0 = CORDIC->RDATA;

	short out2 = (out0&0xffff0000)>>16;
	short out1 = out0&0xffff;

	*theta = (float)(out1 * PI_F)/32768.0f;
	*mod = (float)out2/32768.0f;
}


void RunCordic_Sine(CORDIC_HandleTypeDef hcordic,float theta, float *sin_out, float *cos_out) {
	MODIFY_REG(hcordic.Instance->CSR,CORDIC_CSR_FUNC,SINE_FUNCTION);
	//32 bit input to the function has two parts, the first part is the magnitude the result is
	// multiplied by, that has to be 1 for us. the second 16 bits is the theta value. here 2pi radians
	// gets spread over 65535. so theta * 65535/6.28 becomes the 16 bit value to be appended to input.
	// not using the hal function cordic calculate makes this entire function to return in 1.5 us
	// vs 3.28 with it. so we dont use it. output is one 32 bit variable, which has sin in the first half,
	// and cos in the second half.
	int32_t cordicin = 0x7fff0000;   //  mag = 1
	short thetashort = theta*10435;
	cordicin += thetashort;

	CORDIC->WDATA = cordicin;
	int32_t out0 = CORDIC->RDATA;

	short out2 = (out0&0xffff0000)>>16;
	short out1 = out0&0xffff;
	*sin_out = (float)out1/32768.0f;
	*cos_out = (float)out2/32768.0f;
}

void RunCordic_TwoSines(CORDIC_HandleTypeDef hcordic,float theta1,float theta2,float *sin1 , float *sin2) {
	MODIFY_REG(hcordic.Instance->CSR,CORDIC_CSR_FUNC,SINE_FUNCTION);
	//meant to calculate theta and 60-theta,while doing SVPWM
	int32_t cordicin = 0x7fff0000;   //  mag = 1
	short thetashort = theta1*10435;
	cordicin += thetashort;

	CORDIC->WDATA = cordicin;
	int32_t out0 = CORDIC->RDATA;

	short out1 = out0&0xffff;
	*sin1 = (float)out1/32768.0f;

	//----------------------------------
	cordicin = 0x7fff0000;
	thetashort = theta2*10435;
	cordicin += thetashort;

	CORDIC->WDATA = cordicin;
	out0 = CORDIC->RDATA;

	out1 = out0&0xffff;
	*sin2 = (float)out1/32768.0f;

}


void DQ_Transform(CORDIC_HandleTypeDef hcordic,float a,float b,float c,float rotAngle,float *wd,float *wq){
	float sin_theta=0, cos_theta=0;

	//this transform is the usual one, and makes the result of sin(A),
	//sin(A - 120),sin(A - 240) = -Q when A = 0;

	/*float alpha= a;
	float beta = a/SQRT_3 + 2.0f*b/SQRT_3;

	RunCordic(hcordic,rotAngle,&sin_theta,&cos_theta);
	*wd = alpha * cos_theta + beta * sin_theta;
	*wq = beta * cos_theta - alpha * sin_theta;*/

	// this transform makes it +Q
	float alpha= a;
	float beta = -a/SQRT_3 - 2.0f*b/SQRT_3;

	RunCordic_Sine(hcordic,rotAngle,&sin_theta,&cos_theta);
	*wd = alpha * cos_theta - beta * sin_theta;
	*wq = beta * cos_theta + alpha * sin_theta;
}

void ABC_Transform(CORDIC_HandleTypeDef hcordic,float d,float q,float rotAngle,float *a,float *b,float *c){
	float sin_theta=0, cos_theta=0;
	RunCordic_Sine(hcordic,rotAngle,&sin_theta,&cos_theta);

	float alpha= d*cos_theta -q*sin_theta;
	float beta = q*cos_theta + d*sin_theta;

	*a = alpha;
	*b = -alpha + SQRT_3_2*beta;
	*c = -alpha - SQRT_3_2*beta;
}


