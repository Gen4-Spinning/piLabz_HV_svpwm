/*
 * EncoderSetup.c
 *
 *  Created on: 22-Mar-2023
 *      Author: harsha
 */

#include "AS5x47P.h"
#include "EncoderFns.h"

uint8_t Encoder_checkHealth(void){
	Diaagc diag;
	diag.raw = AS5047_SPI_Read(DIAGC_READ_FRAME,0);
	if((diag.values.magh == 1) || (diag.values.magl == 1) || (diag.values.cof == 1)){
		return 1;
	}else{
		return 0;
	}
}


void SetupABIwithoutPWM(void){
  // we setup ABI with 2000 resolution but we dont use it, becuase we directly read
  // the angle through SPI. But we want to set this up so that we get the index pin
  // signal which we can probe.
  Settings1 settings1;
  settings1.raw = 0;
  settings1.values.factorySetting = 1;
  settings1.values.not_used = 0;
  settings1.values.dir = 0;  // By definition A leads B for CW direction. for us seen from the front, rotating in a CW direction gives A leading B.
  settings1.values.uvw_abi = 0; // 0-ABI with W pin as PWM, 1-UVW with I pin as PWM
  settings1.values.daecdis = 0;
  settings1.values.abibin = 0; // ABI-decimal or binary.
  settings1.values.dataselect = 0; //1 is cordic Angle, 0 is dynamic angle compensation. Remove for very slow speeds.
  settings1.values.pwmon = 0; //sets pwm on W pin if 1.

  AS5047_writeRegister(SETTINGS1_REG, settings1.raw);

  Settings2 settings2;
  settings2.raw = 0;
  settings2.values.abires = 0; // with abibin sets the resolution
  settings2.values.uvwpp = 4; // 5 pole pairs - 0b100
  AS5047_writeRegister(SETTINGS2_REG,settings2.raw);
}

uint8_t Check_ABI_SetCorrectly(Settings1 settings1, Settings2 settings2){
  if ((settings1.values.uvw_abi == 0) && (settings1.values.abibin == 0 ) && (settings1.values.pwmon == 0 ) && ( settings1.values.dir == 0)
      && (settings2.values.abires == 0)  && ( settings2.values.uvwpp == 4)){
        return 1;
      }
  else{
    return 0;
  }
}


uint8_t Encoder_setup(void){
	//setup with ABI on, so that we can probe the I pin if we want.

	Settings1 settings1Reg;
	Settings2 settings2Reg;
	uint8_t abiSettingsOK;

	SetupABIwithoutPWM();
	settings1Reg.raw = AS5047_SPI_Read(SETTINGS1_READ_FRAME, 0);
	settings2Reg.raw = AS5047_SPI_Read(SETTINGS2_READ_FRAME, 0);
	abiSettingsOK = Check_ABI_SetCorrectly(settings1Reg,settings2Reg);
	return abiSettingsOK;
}

uint8_t Encoder_updateZeroPosition(uint16_t zeroValue){
	uint16_t zeroPos;
	zeroPos = AS5047_ReadZeroValue();
	if (zeroPos != zeroValue){
		AS5047_WriteZeroValue(zeroValue); //function must check if it got back the same value it wrote.
		zeroPos = AS5047_ReadZeroValue(); //to check if this value is same as what we wrote
		if (zeroPos != zeroValue){
			return 0;
		}else{
			return 1;
		}
	  }
	return 1;
}


uint8_t Encoder_enableMagErrors(void){
	ZPOSL_frame ZPOS_L;
	ZPOS_L.raw = AS5047_readRegister(ZPOSL_REG,0);
	if ((ZPOS_L.values.comp_h_error == 0 ) || (ZPOS_L.values.comp_l_error == 0)){
		ZPOS_L.values.comp_h_error = 1;
		ZPOS_L.values.comp_l_error = 1;
		AS5047_writeRegister(ZPOSL_REG, ZPOS_L.raw);

		//check if it was written properly
		ZPOS_L.raw = 0;
		ZPOS_L.raw = AS5047_readRegister(ZPOSL_REG,0);
		if ((ZPOS_L.values.comp_h_error == 0 ) || (ZPOS_L.values.comp_l_error == 0)){
			return 0;
		}else{
			return 1;
		}
	}
	return 1;
}


//make continuous Read = 1 if your reading in the while loop etc. If your doing
//continuous read you cant read any other register. if you do you have to restart
//this function by calling it again.
float Encoder_getMechAngle_SPI(uint8_t continuousRead){
	uint16_t angleData = 0;
	float angleMech = 0;
	ReadDataFrame readdataframe;
	Angle angle;
	readdataframe.raw = AS5047_SPI_Read(ANGLE_READ_FRAME,continuousRead);//AS5047_readRegister(ANGLE_REG,1);
	angle.raw = readdataframe.values.data;
	angleData =  angle.values.cordicang;
	angleMech = angleData*SPI_RDNG_TO_MECH_ANGLE;
	return angleMech;
}

uint16_t Encoder_get16BitMechAngle_Single(uint8_t continuous){
	ReadDataFrame readdataframe;
	Angle angle;
	readdataframe.raw = AS5047_SPI_Read(ANGLE_READ_FRAME,continuous);
	angle.raw = readdataframe.values.data;
	return angle.values.cordicang;
}
