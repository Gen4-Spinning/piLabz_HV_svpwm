/*
 * ADC.h
 *
 *  Created on: Mar 5, 2024
 *      Author: harsha
 */

#ifndef ADC_H_
#define ADC_H_
#include <stdint.h>

//has to be in same order as ADC ranks.
typedef struct ADC_Struct{
	uint16_t I_u;
	uint16_t I_v;
	uint16_t V_u;
	uint16_t V_w ;
	uint16_t V_v;
	uint16_t Fet_temp;
	uint16_t I_w;
	uint16_t DC_Current;
}RAW_ADC_I_V;

typedef struct ADC_Struct1{
	float I_u_Avg;
	float I_v_Avg;
	float I_w_Avg;
	float V_u_Avg;
	float V_v_Avg;
	float V_w_Avg;
}RAW_ADC_Averages;

typedef struct ADC_Struct2{
	uint8_t FET_thermistor_open;
	uint8_t motor_thermistor_open;
	uint16_t FetTemp_ADC;
	uint16_t motorTemp_ADC;
	uint16_t DCV_ADC;
	float FetTemp_C;
	float motorTemp_C;
	float DC_Volts;
	uint8_t updated;
}ADC;

uint8_t get_temperature(uint16_t adcVal);

#endif /* GEN4_DRIVERS_INC_ADC_H_ */
