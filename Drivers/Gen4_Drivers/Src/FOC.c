/*
 * FOC.c
 *
 *  Created on: 23-Feb-2024
 *      Author: harsha
 */

#include "FOC.h"


void FOC_calcSVPWM(SVPWM *svpwm,float m, float encoderAngle,float deltaTovoltageAngle){
	// encoder angle should not go above 2Pi
	svpwm->encoderAngle = encoderAngle;
	svpwm->voltageAngle = encoderAngle + deltaTovoltageAngle;
	if (svpwm->voltageAngle > TWO_PI_F){
		svpwm->voltageAngle = svpwm->voltageAngle - TWO_PI_F;
	}else if (svpwm->voltageAngle < 0){
		svpwm->voltageAngle = svpwm->voltageAngle + TWO_PI_F;
	}else{
		//do nothing
	}

	svpwm->sector = (uint8_t)(svpwm->voltageAngle/PI_BY_3F);
	svpwm->sectorAngle = svpwm->voltageAngle - (svpwm->sector * PI_BY_3F);
	float sixtyMinusTheta = PI_BY_3F - svpwm->sectorAngle ;

	float sinTheta,sinSixyMinusTheta;

	RunCordic_TwoSines(hcordic,svpwm->sectorAngle,sixtyMinusTheta,&sinTheta,&sinSixyMinusTheta);
	svpwm->PV1 = (uint16_t)(m * TIMER1_ARR * sinSixyMinusTheta);
	svpwm->PV2 = (uint16_t)(m * TIMER1_ARR * sinTheta);
	svpwm->null = TIMER1_ARR - svpwm->PV1 - svpwm->PV2;

	if (svpwm->sector == 0){
			// U4 is pv1, U6 is PV2
			//100, then 110
			svpwm->CCR1 = svpwm->null/2;
			svpwm->CCR2 = svpwm->CCR1 + svpwm->PV1;
			svpwm->CCR3 = svpwm->CCR2 + svpwm->PV2;
		}else if (svpwm->sector == 1){
			//U2 is PV2,U6 is PV1
			//010, then 110
			svpwm->CCR2 = svpwm->null/2;
			svpwm->CCR1 = svpwm->CCR2 + svpwm->PV2;
			svpwm->CCR3 = svpwm->CCR1 + svpwm->PV1;
		}else if (svpwm->sector == 2){
			//U2 is PV1,U3 is PV2
			//010, then 011
			svpwm->CCR2 = svpwm->null/2;
			svpwm->CCR3 = svpwm->CCR2 + svpwm->PV1;
			svpwm->CCR1 = svpwm->CCR3 + svpwm->PV2;
		}else if (svpwm->sector == 3){
			//U3 is PV1,U1 is PV2
			//001 then 011
			svpwm->CCR3 = svpwm->null/2;
			svpwm->CCR2 = svpwm->CCR3 + svpwm->PV2;
			svpwm->CCR1 = svpwm->CCR2 + svpwm->PV1;
		}else if (svpwm->sector == 4){
			//U1 is PV1,U5 is PV2
			//001 , then 101
			svpwm->CCR3 = svpwm->null/2;
			svpwm->CCR1 = svpwm->CCR3 + svpwm->PV1;
			svpwm->CCR2 = svpwm->CCR1 + svpwm->PV2;
		}else if (svpwm->sector == 5){
			//U5 is PV1,U4 is PV2
			//CCR2 = PV2,CCR3 = PV1
			svpwm->CCR1 = svpwm->null/2;
			svpwm->CCR3 = svpwm->CCR1 + svpwm->PV2;
			svpwm->CCR2 = svpwm->CCR3 + svpwm->PV1;
		}
}

void FOC_applyPWM(SVPWM *foc,uint8_t DT_compensation,uint8_t reversePhases){
	htim1.Instance->CCR1 = fast_fminf(foc->CCR1 + DT_compensation,TIMER1_ARR);
	if (reversePhases==0){
		htim1.Instance->CCR2 = fast_fminf(foc->CCR2 + DT_compensation,TIMER1_ARR);
		htim1.Instance->CCR3 = fast_fminf(foc->CCR3 + DT_compensation,TIMER1_ARR);
	}else{
		htim1.Instance->CCR2 = fast_fminf(foc->CCR3 + DT_compensation,TIMER1_ARR);
		htim1.Instance->CCR3 = fast_fminf(foc->CCR2 + DT_compensation,TIMER1_ARR);
	}
}

void ZeroAllCCRs(SVPWM *svpwm){
	svpwm->CCR1 = 0;
	svpwm->CCR2 = 0;
	svpwm->CCR3 = 0;
}




//------------------HW state struct functions-----------------

void HW_statesInit(HW *hw){
	hw->tim1PwmHWOn = 0;
}


void StartAllPWM(HW *hw){
	if (hw->tim1PwmHWOn == 0){
	  htim1.Instance->CCR1 = 0;
	  htim1.Instance->CCR2 = 0;
	  htim1.Instance->CCR3 = 0;
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	  hw->tim1PwmHWOn = 1;
	}
}

void StopAllPWM(HW *hw){
	if(hw->tim1PwmHWOn == 1){
		htim1.Instance->CCR1 = 0;
		htim1.Instance->CCR2 = 0;
		htim1.Instance->CCR3 = 0;
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
		hw->tim1PwmHWOn = 0;
		htim1.State = HAL_TIM_STATE_READY;
	}
}

void Zero_PID_Terms(PID *pid){
	pid->integralError = 0;
	pid->KiTerm = 0;
	pid->error = 0;
	pid->out = 0;
}
void Init_PID_Terms(PID *pid,float Kp,float Ki,float SO,float FF){
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->FF = FF;
	pid->sO = SO;
	pid->KiTerm = 0;
	pid->out = 0;
}

float ExecVoltagePID(PID *pid,float target, float actual,float min,float max){
	pid->error = target - actual;
	pid->integralError = pid->integralError + (pid->error*TIM1_DT);
	pid->KpTerm = pid->Kp * pid->error;
	pid->KiTerm = pid->Ki*pid->integralError;
	pid->FFTerm = pid->FF*target ;
	float out =  pid->KpTerm + pid->KiTerm + pid->FFTerm + pid->sO;
	out = fast_fmaxf(fast_fminf(out,max),min);
	return out;
}

void getTargetCurrentsDQ(PID *pid,FOC *foc,float target, float actual){
	pid->error = target - actual;
	pid->integralError = pid->integralError + pid->error;
	pid->KpTerm = pid->Kp * pid->error;
	pid->KiTerm = pid->Ki*pid->integralError;
	pid->FFTerm = pid->FF*target ;
	foc->IqRef =  pid->KpTerm + pid->KiTerm + pid->FFTerm + pid->sO;
	foc->IqRef = fast_fmaxf(fast_fminf(foc->IqRef,foc->IpkMax),-foc->IpkMax);
	foc->IdRef = 0;
	/*if (doMTPA){
		IdRef = ..
	}*/
}



void FOC_updateSensorVals(FOC *foc,RAW_ADC_I_V *adc,RAW_ADC_Averages *adcAvg){
	foc->U_amps = ((float)adc->I_u - adcAvg->I_u_Avg) * CS_CONSTANT;
	foc->V_amps = ((float)adc->I_v - adcAvg->I_v_Avg) * CS_CONSTANT;
	foc->W_amps = ((float)adc->I_w - adcAvg->I_w_Avg)* CS_CONSTANT;
	foc->U_volts = ((float)adc->V_u - adcAvg->V_u_Avg) * V_CONSTANT;
	foc->V_volts = ((float)adc->V_v - adcAvg->V_v_Avg) * V_CONSTANT;
	foc->W_volts = ((float)adc->V_w - adcAvg->V_w_Avg) * V_CONSTANT;
}


void getVq(FOC *foc,PID *pid){
//implement antiwindup
	pid->error = foc->IqRef - foc->Iq;
	pid->integralError = pid->integralError + pid->error;
	pid->KpTerm = pid->Kp * pid->error;
	pid->KiTerm = pid->Ki*pid->integralError;
	pid->FFTerm = pid->FF*foc->IqRef ;
	foc->Vq =  pid->KpTerm + pid->KiTerm + pid->FFTerm + pid->sO;
}

void getVd(FOC *foc,PID *pid){
	pid->error = foc->IdRef - foc->Id;
	pid->integralError = pid->integralError + pid->error;
	pid->KpTerm = pid->Kp * pid->error;
	pid->KiTerm = pid->Ki*pid->integralError;
	pid->FFTerm = pid->FF*foc->IdRef ;
	foc->Vd =  pid->KpTerm + pid->KiTerm + pid->FFTerm + pid->sO;
}

void Initialize_FOC_Terms(FOC *foc,float IpkMax){
	foc->IdRef = 0;
	foc->IqRef = 0;
	foc->IpkMax = IpkMax;
}


