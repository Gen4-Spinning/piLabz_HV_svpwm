/*
 * Calibartion.c
 *
 *  Created on: Feb 26, 2024
 *      Author: harsha
 */

#include "Calibration.h"
#include "FOC.h"
#include "PositionSensor.h"
#include <stdio.h>

extern char UART_buffer[60];
extern MotorSetup ms;

void Calibration_init(Calib *c){
	c->setVoltage = 0.05f;
	c->mechSecPerRev = PI_F; //tim
	c->mechSpeedRadSec = TWO_PI_F/c->mechSecPerRev;
	c->noOfTurns = 6;
	c->loopcounter = 0;
	c->virtualMechRadians = 0;
	c->virtualElecRadians = 0;

	//for zeroing
	c->CWTurns=-1;
	c->CCWTurns = -1;
	for (int i=0;i<5;i++){
		c->encoderZeros_CW[i] = 0;
		c->encoderZeros_CCW[i] = 0;
	}
	c->finalCW=0;
	c->finalCCW = 0;
	c->finalAverage=0;

	//for linearize and CG
	c->sampleTime = c->mechSecPerRev/CALIB_SAMPLES;
	c->prevSampleTime = 0;
	c->sampleIndex = 0;

}

//for finding the zero position, run more than one loop and find the place where the reading crosses
//zero
uint8_t Calibration_encoderZeroing(Calib *c,PositionSensor *ps){
	int16_t virtualEncReading = 0;
	c->loopcounter++;
	c->time = c->loopcounter*TIM1_DT;
	float t0 = 1.0f;
	float t1 = 1.0f + (c->noOfTurns*TWO_PI_F)/c->mechSpeedRadSec;
	float t2 = t1 + 1.0f;
	float t3 = t2 + (c->noOfTurns*TWO_PI_F)/c->mechSpeedRadSec;//we dont count want to take a sensor reading after the first overflow when we change direction. So to discount that and still get the required no of readings, we add another turn
	if (c->time < t0){
		ZeroAllCCRs(&svpwm);
		StartAllPWM(&hw);
		return 0;
	}
	else if (c->time < t1){
		c->virtualMechRadians = c->virtualMechRadians + TIM1_DT*c->mechSpeedRadSec;
		if (c->virtualMechRadians > TWO_PI_F){
			c->virtualMechRadians  = c->virtualMechRadians  - TWO_PI_F;
			virtualEncReading = (int16_t)((c->virtualMechRadians/TWO_PI_F)*16384);
			//we get six turns including the first half turn
			if(c->CWTurns > -1){
				c->encoderZeros_CW[c->CWTurns] = ps->encoder_raw;
			}
			//c->encoderZeros_CW[CWTurns] = virtualEncReading-ps->encoder_raw;
			//sprintf(UART_buffer,"D,%05.2f,%05.2f,%05d,%05d,%01d,E\r\n",c->time,c->virtualMechRadians,virtualEncReading,ps->encoder_raw,c->CWTurns);
			//HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,32);
			c->CWTurns++;
		}
		//get electrical rads from mech rads
		c->virtualElecRadians = getElecAngleFromMechAngle(c->virtualMechRadians);
		//applySVPWM
		FOC_calcSVPWM(&svpwm,c->setVoltage,c->virtualElecRadians,0);
		FOC_applyPWM(&svpwm,0,ms.reversePhases);
		return 0;

	}else if ((c->time > t2)&&(c->time < t3)){
		c->virtualMechRadians = c->virtualMechRadians - TIM1_DT*c->mechSpeedRadSec;
		if (c->virtualMechRadians < 0){
			c->virtualMechRadians  = c->virtualMechRadians  + TWO_PI_F;
			//some complicated stuff to not use turncounter as the index of the array
			if(c->CCWTurns > -1){
				c->encoderZeros_CCW[c->CCWTurns] = ps->encoder_raw;
			}
			//virtualEncReading = (int16_t)((c->virtualMechRadians/TWO_PI_F)*16384);
			//sprintf(UART_buffer,"D,%05.2f,%05.2f,%05d,%05d,%02d,E\r\n",c->time,c->virtualMechRadians,virtualEncReading,ps->encoder_raw,c->CCWTurns);
			//HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,32);
			c->CCWTurns++;
		}
		c->virtualElecRadians = getElecAngleFromMechAngle(c->virtualMechRadians);
		//applySVPWM
		FOC_calcSVPWM(&svpwm,c->setVoltage,c->virtualElecRadians,0);
		FOC_applyPWM(&svpwm,0,ms.reversePhases);
		return 0;
	}
	else if (c->time > t3 + 1.0f){
		c->virtualMechRadians = 0;
		c->virtualElecRadians = 0;
		c->loopcounter=0;
		c->time = 0;
		StopAllPWM(&hw);

		//calculate average from the array
		long totalCW = 0;
		long totalCCW = 0;
		for (int i=0;i<5;i++){
			totalCW += c->encoderZeros_CW[i];
			totalCCW += c->encoderZeros_CCW[i];
		}
		c->finalCW = (uint16_t)(totalCW/5);
		c->finalCCW = (uint16_t)(totalCCW/5);
		c->finalAverage = (c->finalCW+c->finalCCW)/2;
		c->deltaBtwSides = c->finalCW - c->finalCCW;

		//sprintf(UART_buffer,"Final=CW:%05d,CCW:%05d,AVG:%05d\r\n",c->finalCW,c->finalCCW,c->finalAverage);
		//HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,37);
		return 1;
	}
	return 0;
}

// for cogging torque, and linear error calculation do this - do this later.
/*uint8_t Calibration_Linearize(Calib *c,PositionSensor *ps){
	int16_t virtualEncReading=0;

	c->loopcounter++;
	c->time = c->loopcounter*TIM1_DT;
	float t0 = 1.0f;
	float t1 = 1.0f + (c->noOfTurns*TWO_PI_F)/c->mechSpeedRadSec;
	float t2 = t1 + 1.0f;
	float t3 = t2 + (c->noOfTurns*TWO_PI_F)/c->mechSpeedRadSec;//we dont count want to take a sensor reading after the first overflow when we change direction. So to discount that and still get the required no of readings, we add another turn
	if (c->time < t0){
		ZeroAllCCRs(&svpwm);
		StartAllPWM(&hw);
		return 0;
	}
	else if (c->time < t1){
		c->virtualMechRadians = c->virtualMechRadians + TIM1_DT*c->mechSpeedRadSec;
		if (c->virtualMechRadians > TWO_PI_F){
			c->virtualMechRadians  = c->virtualMechRadians  - TWO_PI_F;
		}
		//get electrical rads from mech rads
		c->virtualElecRadians = getElecAngleFromMechAngle(c->virtualMechRadians);
		//applySVPWM
		FOC_calcSVPWM(&svpwm,c->setVoltage,c->virtualElecRadians,0);
		FOC_applyPWM(&svpwm);


		if (c->time - c->prevSampleTime > c->sampleTime){
			virtualEncReading = (int16_t)((c->virtualMechRadians/TWO_PI_F)*16384);
			if (c->sampleIndex > CALIB_SAMPLES){
				return 0;
			}
			c->prevSampleTime = c->time;
			sprintf(UART_buffer,"D,%05.2f,%05d,%05.2f,%05d,%05d,E\r\n",c->time,c->sampleIndex,c->virtualMechRadians,virtualEncReading,ps->encoder_raw);
			HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,35);
			c->sampleIndex++;
		}

		return 0;

	}else if ((c->time > t2)&&(c->time < t3)){
		c->virtualMechRadians = c->virtualMechRadians - TIM1_DT*c->mechSpeedRadSec;
		if (c->virtualMechRadians < 0){
			c->virtualMechRadians  = c->virtualMechRadians  + TWO_PI_F;
		}
		c->virtualElecRadians = getElecAngleFromMechAngle(c->virtualMechRadians);
		//applySVPWM
		FOC_calcSVPWM(&svpwm,c->setVoltage,c->virtualElecRadians,0);
		FOC_applyPWM(&svpwm,15);
		return 0;
	}
	else if (c->time > t3 + 1.0f){
		c->virtualMechRadians = 0;
		c->virtualElecRadians = 0;
		c->loopcounter=0;
		c->time = 0;
		StopAllPWM(&hw);
		return 1;
	}
	return 0;
}

*/
