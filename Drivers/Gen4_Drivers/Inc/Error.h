/*
 * Error.h
 *
 *  Created on: Nov 16, 2024
 *      Author: harsha
 */

#ifndef GEN4_DRIVERS_INC_ERROR_H_
#define GEN4_DRIVERS_INC_ERROR_H_


typedef struct ErrorType{
	uint8_t globalErrorFlag;
	uint8_t fetThermistorOpen;
	uint8_t windingsThermistorOpen;
	uint8_t temp_FET_outOfBounds;
	uint8_t temp_windings_outOfBounds;
	uint8_t PID_error;
	long counter;
}Error;
#endif /* GEN4_DRIVERS_INC_ERROR_H_ */
