/*
 * Logger.h
 *
 *  Created on: 25-Mar-2024
 *      Author: harsha
 */

#ifndef LOGGER_H_
#define LOGGER_H_
#include "stdio.h"
#include "ADC.h"
#include "FOC.h"
#include "PositionSensor.h"
#include "MathConstants.h"
#include "ControlConfig.h"
#include "CoggingFrictionRemoval.h"

#define HSARRAYSIZE 2000 // TODO: setup so that its a multiple of 30
#define HSLOGSIZE 34

#define NOBUFFER 0
#define BUFFER1 1
#define BUFFER2 2

#define LOG_TURNS 2

typedef struct HSLoggerStruct{
	uint8_t enable;
	uint8_t firstTime;

	//turns, scope, see if more sinusoidal with more voltage..
	float start_multiTurns;
	float end_multiTurns;
	uint8_t stopAndFlushBuffer;

	//start logging a no of turns and get that mech radians from the ps
	uint8_t runningBuffer;
	char HSbuffer1[HSARRAYSIZE];
	char HSbuffer2[HSARRAYSIZE];
	uint16_t bufferIndex;

	uint8_t sendOut;
	uint16_t sendOutBufferSize;
	uint8_t DMAdataSentOut;

	long startLoopIndex;
	long endLoopIndex;
	long addDatas;


}HSLogger;



uint16_t addData(HSLogger *hL, SVPWM *svpwm, PositionSensor *ps, FOC *foc,friction *fr , cogging *cg);
void switchBuffers(HSLogger *hL);
void hsLogInit(HSLogger *hsLog);
void hsLogStart(HSLogger *hsLog,PositionSensor *ps);
void hsLogReset(HSLogger *hsLog);
void hsLog_CheckStopConditionReached(HSLogger *hL,PositionSensor *ps,int8_t direction);

uint16_t add8Bit(HSLogger *hL,uint8_t var,uint16_t index);
uint16_t add16Bit(HSLogger *hL,uint16_t var,uint16_t index);
uint16_t add32Bit(HSLogger *hL,uint32_t var,uint16_t index);
uint16_t addFloat(HSLogger *hL,float var,uint16_t index);

#endif /* GEN4_DRIVERS_INC_LOGGER_H_ */
