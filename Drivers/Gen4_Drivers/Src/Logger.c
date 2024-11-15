/*
 * Logger.c
 *
 *  Created on: 25-Mar-2024
 *      Author: harsha
 */

#include "Logger.h"

extern float OL_elecRadians;
union {
  float float_variable;
  uint8_t byte_array[4];
} uHs;

void hsLogInit(HSLogger *hsLog){
	hsLog->enable = 0;
	hsLog->bufferIndex = 0;
	hsLog->runningBuffer = BUFFER1;
	hsLog->sendOut = NOBUFFER;
}

void hsLogStart(HSLogger *hsLog,PositionSensor *ps){
	hsLog->enable = 1;
	hsLog->firstTime = 1;
	hsLog->bufferIndex = 0;
	hsLog->runningBuffer = BUFFER1;
	hsLog->sendOut = NOBUFFER;
	hsLog->start_multiTurns = ps->multiturn_mech_radians;
	hsLog->stopAndFlushBuffer = 0;
}

void hsLogReset(HSLogger *hsLog){
	hsLog->enable = 0;
	hsLog->firstTime = 0;
	hsLog->bufferIndex = 0;
	hsLog->runningBuffer = NOBUFFER;
	hsLog->start_multiTurns  = 0;
	hsLog->end_multiTurns = 0;
}


void switchBuffers(HSLogger *hL){
	if (hL->runningBuffer == BUFFER1){
		hL->sendOutBufferSize = hL->bufferIndex;
		hL->sendOut = BUFFER1;
		hL->bufferIndex  = 0;
		hL->runningBuffer = BUFFER2;
	}else if (hL->runningBuffer == BUFFER2){
		hL->sendOutBufferSize = hL->bufferIndex;
		hL->sendOut = BUFFER2;
		hL->bufferIndex  = 0;
		hL->runningBuffer = BUFFER1;
	}
}

uint16_t add8Bit(HSLogger *hL,uint8_t var,uint16_t index){
	if (hL->runningBuffer == BUFFER1){
		hL->HSbuffer1[index] = var & 0xFF;
	}else{
		hL->HSbuffer2[index] = var & 0xFF;
	}
	return index+1;
}

uint16_t add16Bit(HSLogger *hL,uint16_t var,uint16_t index){
	if (hL->runningBuffer == BUFFER1){
		hL->HSbuffer1[index] = var >> 8 & 0xFF;
		hL->HSbuffer1[index+1] = var & 0xFF;
	}else{
		hL->HSbuffer2[index] = var >> 8 & 0xFF;
		hL->HSbuffer2[index+1] = var & 0xFF;
	}
	return index+2;
}

uint16_t addFloat(HSLogger *hL,float var,uint16_t index){
	uHs.float_variable = var;
	if (hL->runningBuffer == BUFFER1){
		hL->HSbuffer1[index]= (uint8_t)(uHs.byte_array[3]);
		hL->HSbuffer1[index+1]= (uint8_t)(uHs.byte_array[2]);
		hL->HSbuffer1[index+2]= (uint8_t)(uHs.byte_array[1]);
		hL->HSbuffer1[index+3]= (uint8_t)(uHs.byte_array[0]);
	}else{
		hL->HSbuffer2[index]= (uint8_t)(uHs.byte_array[3]);
		hL->HSbuffer2[index+1]= (uint8_t)(uHs.byte_array[2]);
		hL->HSbuffer2[index+2]= (uint8_t)(uHs.byte_array[1]);
		hL->HSbuffer2[index+3]= (uint8_t)(uHs.byte_array[0]);
	}
	return index+4;
}

uint16_t add32Bit(HSLogger *hL,uint32_t var,uint16_t index){
	if (hL->runningBuffer == BUFFER1){
		hL->HSbuffer1[index]= (uint8_t)(var >> 24 & 0xFF);
		hL->HSbuffer1[index+1]= (uint8_t)(var >> 16 & 0xFF);
		hL->HSbuffer1[index+2]= (uint8_t)(var >> 8 & 0xFF);
		hL->HSbuffer1[index+3]= (uint8_t)(var & 0xFF);
	}else{
		hL->HSbuffer2[index]= (uint8_t)(var >> 24 & 0xFF);
		hL->HSbuffer2[index+1]= (uint8_t)(var >> 16 & 0xFF);
		hL->HSbuffer2[index+2]= (uint8_t)(var >> 8 & 0xFF);
		hL->HSbuffer2[index+3]= (uint8_t)(var & 0xFF);
	}
	return index+4;
}

extern float OL_elecRadians;
uint16_t addData(HSLogger *hL, SVPWM *svpwm, PositionSensor *ps, FOC *foc,friction *fr , cogging *cg){
	uint16_t idx = hL->bufferIndex;
	idx = add8Bit(hL,0x68,idx); // D
	idx = add8Bit(hL,0x58,idx); // :

	idx = add32Bit(hL,svpwm->loopCounter,idx);
	idx = add16Bit(hL,ps->encoder_raw,idx);
	idx = addFloat(hL,ps->velocity_radsec,idx);

	idx = add8Bit(hL,fr->frictionCompensationOn,idx);
	idx = addFloat(hL,OL_elecRadians,idx);

	idx = add8Bit(hL,cg->coggingCompensationOn,idx);
	idx = addFloat(hL,ps->elecRadians,idx);

	idx = addFloat(hL,svpwm->voltagePercent,idx);
	idx = addFloat(hL,foc->m,idx);

	idx = add8Bit(hL,0x58,idx); // :
	idx = add8Bit(hL,0x69,idx); // E

	idx = add8Bit(hL,0x0A,idx); // 0A
	idx = add8Bit(hL,0x0D,idx); // 0D
	return idx;
}


void hsLog_CheckStopConditionReached(HSLogger *hL,PositionSensor *ps,int8_t direction){
	 hL->stopAndFlushBuffer = 0;
	if (direction == CW){
	  if (ps->multiturn_mech_radians > hL->end_multiTurns){
		  hL->stopAndFlushBuffer = 1;
	  	 }
	  }
	else if (direction == CCW){
	  if (ps->multiturn_mech_radians < hL->end_multiTurns){
		  hL->stopAndFlushBuffer = 1;
	  }
	}
	else{}
}
/*
uint16_t addDataOld(HSLogger *hL, FOC *foc, PositionSensor *ps,RAW_ADC_I_V *adcIV,FOC_snsrs *focIV){
	uint16_t idx = hL->bufferIndex;
	if (hL->runningBuffer == BUFFER1){
		hL->HSbuffer1[idx]= (uint8_t)(0x68);  // D
		hL->HSbuffer1[idx+1]= (uint8_t)(0x58); // :

		hL->HSbuffer1[idx+2]= (uint8_t)(foc->loopCounter >> 24 & 0xFF);
		hL->HSbuffer1[idx+3]= (uint8_t)(foc->loopCounter  >> 16 & 0xFF);
		hL->HSbuffer1[idx+4]= (uint8_t)(foc->loopCounter  >> 8 & 0xFF);
		hL->HSbuffer1[idx+5]= (uint8_t)(foc->loopCounter & 0xFF);
		hL->HSbuffer1[idx+6]= (uint8_t)(ps->encoder_raw >> 8 & 0xFF);
		hL->HSbuffer1[idx+7]= (uint8_t)(ps->encoder_raw & 0xFF);
		hL->HSbuffer1[idx+8]= (uint8_t)((adcIV->I_u >> 8) & 0xFF);
		hL->HSbuffer1[idx+9]= (uint8_t)(adcIV->I_u & 0xFF);
		hL->HSbuffer1[idx+10]= (uint8_t)((adcIV->I_v >> 8) & 0xFF);
		hL->HSbuffer1[idx+11]= (uint8_t)(adcIV->I_v & 0xFF);
		hL->HSbuffer1[idx+12]= (uint8_t)((adcIV->I_w >> 8) & 0xFF);
		hL->HSbuffer1[idx+13]= (uint8_t)(adcIV->I_w & 0xFF);
		hL->HSbuffer1[idx+14]= (uint8_t)((adcIV->V_u >> 8) & 0xFF);
		hL->HSbuffer1[idx+15]= (uint8_t)(adcIV->V_u & 0xFF);
		hL->HSbuffer1[idx+16]= (uint8_t)((adcIV->V_v >> 8) & 0xFF);
		hL->HSbuffer1[idx+17]= (uint8_t)(adcIV->V_v & 0xFF);
		hL->HSbuffer1[idx+18]= (uint8_t)((adcIV->V_w >> 8) & 0xFF);
		hL->HSbuffer1[idx+19]= (uint8_t)(adcIV->V_w & 0xFF);
		hL->HSbuffer1[idx+20]= (uint8_t)((adcIV->DC_Current >> 8) & 0xFF);
		hL->HSbuffer1[idx+21]= (uint8_t)(adcIV->DC_Current & 0xFF);
		hL->HSbuffer1[idx+22]= (uint8_t)((adcIV->Fet_temp >> 8) & 0xFF);
		hL->HSbuffer1[idx+23]= (uint8_t)(adcIV->Fet_temp & 0xFF);

		uHs.float_variable = focIV->Id;
		hL->HSbuffer1[idx+24]= (uint8_t)(uHs.byte_array[3]);
		hL->HSbuffer1[idx+25]= (uint8_t)(uHs.byte_array[2]);
		hL->HSbuffer1[idx+26]= (uint8_t)(uHs.byte_array[1]);
		hL->HSbuffer1[idx+27]= (uint8_t)(uHs.byte_array[0]);
		uHs.float_variable = focIV->Iq;
		hL->HSbuffer1[idx+28]= (uint8_t)(uHs.byte_array[3]);
		hL->HSbuffer1[idx+29]= (uint8_t)(uHs.byte_array[2]);
		hL->HSbuffer1[idx+30]= (uint8_t)(uHs.byte_array[1]);
		hL->HSbuffer1[idx+31]= (uint8_t)(uHs.byte_array[0]);

		hL->HSbuffer1[idx+32]= (uint8_t)(hL->runningBuffer);

		hL->HSbuffer1[idx+33]= (uint8_t)(0x58); // :
		hL->HSbuffer1[idx+34]= (uint8_t)(0x69); // E

		hL->HSbuffer1[idx+35]= (uint8_t)(0x0A); //  \r
		hL->HSbuffer1[idx+36]= (uint8_t)(0x0D); //  \n
	}
	else if (hL->runningBuffer == BUFFER2){
		hL->HSbuffer2[idx]= (uint8_t)(0x68);  // D
		hL->HSbuffer2[idx+1]= (uint8_t)(0x58); // :

		hL->HSbuffer2[idx+2]= (uint8_t)(foc->loopCounter >> 24 & 0xFF);
		hL->HSbuffer2[idx+3]= (uint8_t)(foc->loopCounter  >> 16 & 0xFF);
		hL->HSbuffer2[idx+4]= (uint8_t)(foc->loopCounter  >> 8 & 0xFF);
		hL->HSbuffer2[idx+5]= (uint8_t)(foc->loopCounter & 0xFF);
		hL->HSbuffer2[idx+6]= (uint8_t)(ps->encoder_raw >> 8 & 0xFF);
		hL->HSbuffer2[idx+7]= (uint8_t)(ps->encoder_raw & 0xFF);
		hL->HSbuffer2[idx+8]= (uint8_t)((adcIV->I_u >> 8) & 0xFF);
		hL->HSbuffer2[idx+9]= (uint8_t)(adcIV->I_u & 0xFF);
		hL->HSbuffer2[idx+10]= (uint8_t)((adcIV->I_v >> 8) & 0xFF);
		hL->HSbuffer2[idx+11]= (uint8_t)(adcIV->I_v & 0xFF);
		hL->HSbuffer2[idx+12]= (uint8_t)((adcIV->I_w >> 8) & 0xFF);
		hL->HSbuffer2[idx+13]= (uint8_t)(adcIV->I_w & 0xFF);
		hL->HSbuffer2[idx+14]= (uint8_t)((adcIV->V_u >> 8) & 0xFF);
		hL->HSbuffer2[idx+15]= (uint8_t)(adcIV->V_u & 0xFF);
		hL->HSbuffer2[idx+16]= (uint8_t)((adcIV->V_v >> 8) & 0xFF);
		hL->HSbuffer2[idx+17]= (uint8_t)(adcIV->V_v & 0xFF);
		hL->HSbuffer2[idx+18]= (uint8_t)((adcIV->V_w >> 8) & 0xFF);
		hL->HSbuffer2[idx+19]= (uint8_t)(adcIV->V_w & 0xFF);
		hL->HSbuffer2[idx+20]= (uint8_t)((adcIV->DC_Current >> 8) & 0xFF);
		hL->HSbuffer2[idx+21]= (uint8_t)(adcIV->DC_Current & 0xFF);
		hL->HSbuffer2[idx+22]= (uint8_t)((adcIV->Fet_temp >> 8) & 0xFF);
		hL->HSbuffer2[idx+23]= (uint8_t)(adcIV->Fet_temp & 0xFF);

		uHs.float_variable = focIV->Id;
		hL->HSbuffer2[idx+24]= (uint8_t)(uHs.byte_array[3]);
		hL->HSbuffer2[idx+25]= (uint8_t)(uHs.byte_array[2]);
		hL->HSbuffer2[idx+26]= (uint8_t)(uHs.byte_array[1]);
		hL->HSbuffer2[idx+27]= (uint8_t)(uHs.byte_array[0]);
		uHs.float_variable = focIV->Iq;
		hL->HSbuffer2[idx+28]= (uint8_t)(uHs.byte_array[3]);
		hL->HSbuffer2[idx+29]= (uint8_t)(uHs.byte_array[2]);
		hL->HSbuffer2[idx+30]= (uint8_t)(uHs.byte_array[1]);
		hL->HSbuffer2[idx+31]= (uint8_t)(uHs.byte_array[0]);

		hL->HSbuffer2[idx+32]= (uint8_t)(hL->runningBuffer);

		hL->HSbuffer2[idx+33]= (uint8_t)(0x58); // :
		hL->HSbuffer2[idx+34]= (uint8_t)(0x69); // E

		hL->HSbuffer2[idx+35]= (uint8_t)(0x0A); //  \r
		hL->HSbuffer2[idx+36]= (uint8_t)(0x0D); //  \n
	}

	return idx+37; // send out plus one and start from idx + 0 the next time
}
*/

