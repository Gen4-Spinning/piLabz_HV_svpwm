/*
 * UnusedFns.c
 *
 *  Created on: 29-Mar-2024
 *      Author: harsha
 */


/*
if (doDQ){
	  DQ_Transform(hcordic,a,b,c_,rotAngle,&wd,&wq);
	  doDQ = 0;
  }

if (doABC){
	ABC_Transform(hcordic,wd,wq,rotAngle,&a,&b,&c_);
	doABC = 0;
}

if(test_FOC){
	  for (int i=0;i<628;i++){
		  float eAngle = i/100.0;
		  FOC_calcSVPWM(&foc,0.5,eAngle,0);
		  sprintf(UART_buffer,"D,%05.2f,%05.2f,%05d,%05d,%05d,E\r\n",foc.voltageAngle,eAngle,foc.CCR1,foc.CCR2,foc.CCR3);
		  HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,36);
	  }
	  test_FOC = 0;
  }

  if (calibration == 1){
			cOver=Calibration_encoderZeroing(&c,&ps);
			if (cOver==1){
				//for now saving is manual in the while loop
				calibration = 0;
				cOver = 0;
				turnOffPWMS = 1;// turnoff in the while loop
			}
		}

  if(checkUartDmA){
	  HAL_UART_Transmit_DMA(&huart3,(uint8_t *)"is this Working????\r\n",22);
	  checkUartDmA = 0;
  }

  	  -----Debug CODES---------
	  enc_mechAngle = Encoder_getMechAngle_SPI(0);
	  enc_healthNOK = Encoder_checkHealth();
	  PositionSensor_update(&ps,0.01f);
	  start_ticks = SysTick->VAL;
	  RunCordic_Sine(hcordic,inputRadians, &sinOut, &cosOut);
	  sixtyMinustheta = PI_BY_3F-inputRadians;
	  RunCordic_TwoSines(hcordic,inputRadians,sixtyMinustheta,&sin1_out,&sin2_out);
	  FOC_calcSVPWM(&foc,0.1,ps.elecRadians,PI_F/2);
	  delta_ticks = SysTick->VAL - start_ticks;
	  HAL_Delay(10);
	  sprintf(UART_buffer,"D,%05.2f,%05.2f,%05d,%05d,E\r\n",ps.mechRadians_singleTurn,ps.elecRadians,ps.encoder_raw,ps.turns);
	  HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,30);
	  if (debugCWPWM){
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		  debugCWPWM = 0;
	  }
	  ------------End of Debug Fns ---------------

	  if (checkPHASE){
		  RunCordic_Phase(hcordic,Vd_test,Vq_test,&radiansOut,&modOut);
		  checkPHASE = 0;
	  }
*/
