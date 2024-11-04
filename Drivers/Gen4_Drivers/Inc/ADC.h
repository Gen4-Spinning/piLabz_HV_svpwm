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

#endif /* GEN4_DRIVERS_INC_ADC_H_ */
