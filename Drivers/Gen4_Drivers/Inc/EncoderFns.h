/*
 * EncoderFns.h
 *
 *  Created on: 22-Mar-2023
 *      Author: harsha
 */

#ifndef ENCODERFNS_H_
#define ENCODERFNS_H_

#include "AS5x47P.h"
#include "HWConfig.h"

void SetupABIwithoutPWM(void);
uint8_t Check_ABI_SetCorrectly(Settings1 settings1, Settings2 settings2);

uint8_t Encoder_setup(void);
uint8_t Encoder_updateZeroPosition(uint16_t zeroValue);
uint8_t Encoder_checkHealth(void);
uint8_t Encoder_enableMagErrors(void);

float Encoder_getMechAngle_SPI(uint8_t continuousRead);
uint16_t Encoder_get16BitMechAngle_Single(uint8_t continuous);

#endif /* ENCODERFNS_H_ */
