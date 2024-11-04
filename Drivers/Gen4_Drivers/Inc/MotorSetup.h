/*
 * MotorSetup.h
 *
 *  Created on: 29-Feb-2024
 *      Author: harsha
 */

#ifndef MOTORSETUP_H_
#define MOTORSETUP_H_

typedef struct Setup_Struct{
	float encCW_offset; // some magic nos cos calibration is not working properly
	float encCCW_offset;
	uint16_t encAvg_offset;
	uint8_t reversePhases;
	uint8_t settingsOK;
}MotorSetup;


#endif /* GEN4_DRIVERS_INC_MOTORSETUP_H_ */
