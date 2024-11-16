/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "EncoderFns.h"
#include "PositionSensor.h"
#include "Cordic.h"
#include "FOC.h"
#include "Calibration.h"
#include "MotorSetup.h"
#include "Ramp.h"
#include "ADC.h"
#include "Logger.h"
#include "MathOps.h"
#include "SpeedSensor.h"
#include "ControlConfig.h"
#include "PosControl-JC.h"
#include "CoggingFrictionRemoval.h"
#include "Error.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

CORDIC_HandleTypeDef hcordic;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
CORDIC_HandleTypeDef hcordic;
CORDIC_ConfigTypeDef sCordicConfig;
PositionSensor ps;
Speed s;
SVPWM svpwm;
HW hw;
MotorSetup ms;
RampDuty r;
FOC foc;
RampRPM rampRPM;
HSLogger hsLog;
PID speedPID;
PID VqPID;
PID VdPID;
PID posPID;
posControlJC p;
friction fr;
cogging cg;
ADC adc;
Error err;

char UART_buffer[80];
uint16_t ADC2_buff[3];//voltage, two temperatures
//encoder Setup
uint8_t setupOK = 0,errorSetupOK,encZeroPosSetup;
//Ramps
uint8_t dbg_rampDuty_RUStart=0,dbg_rampDuty_RDStart=0,dbg_rampDuty_ChangeDuty=0,dbg_rampDuty_Stop=0;
uint8_t dbg_rampRPM_RUStart=0,dbg_rampRPM_RDStart=0,dbg_rampRPM_ChangeRPM=0,dbg_rampRPM_Stop=0;
uint16_t RPM_transitionTime=0,RPM_transitionTarget = 0;
uint16_t targetDuty = 0,targetRPM = 0;
int8_t direction = CW; // 1 or -1
//control dbg
int8_t turnOffPWMS=0,readPosition=0,turnOff_sensors = 0;
//OL debug mode
float OL_elecRadians = 0,d_elecRadians = 0,OL_RPM = 0;
uint8_t runOL = 0,OL_init = 0,OL_stop = 0;
long lastOL_Counter = 0;
uint16_t OL_delayCycles = 10;
float error_elecRadians = 0,actual_elRadians = 0,OL_offsetRadians = 0,temp =0,correctionPercent=0;
uint8_t addCorrectionPercent = 0;
float out2 = 0;
//posControl
uint8_t setupPosJC = 0,startPosC=0,stopPosC=0;
float pos_targetThetaDeg=0,pos_targetTime_ms = 0;

//logging
long dt,t0,dt2;
uint8_t hsLogOn = 0,checkUartDmA = 0,logEncoderDMA=0;
uint16_t buffer1Sent = 0,buffer1flush=0,buffer2flush=0,buffer2Sent = 0;
long totalBytes=0;

uint8_t setupFrictionAddition = 0;
uint16_t frictionMaxPWM = 6; // 6 experimentally found.
uint8_t setupCoggingAddition = 0;
uint16_t coggingMaxPWM = 10;

uint8_t checkEncoderHealth = 0;
uint8_t encoderNOK = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_CORDIC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	hsLog.DMAdataSentOut += 1; // use this and keep a single buffer?
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* TIM1 is set to fire at half the pwm freq, so 10Khz as the pwm freq is 40Khz. Since RPR= 3
	 * is set before the timer is on, the timer fires at the underflow interrupt
	 */
	if (htim->Instance==TIM1){
		dt = SysTick->VAL - t0;
		t0 = SysTick->VAL;
		svpwm.loopCounter++;

		if (readPosition){
			PositionSensor_update(&ps,TIM1_DT);
			getAveragedVelocityRadSec(&ps);
			updateSpeedCalc(&s,&ps);
			actual_elRadians = ps.elecRadians - OL_offsetRadians;
			if (actual_elRadians < 0){ //works For CW
				temp = 6.28f - actual_elRadians;
				actual_elRadians = temp;
			}
		}

		if (runOL){
			if (svpwm.loopCounter > lastOL_Counter+OL_delayCycles){
				OL_elecRadians += d_elecRadians;
				if (OL_elecRadians >= 6.28f){
					OL_elecRadians = 0;
				}
				if (OL_elecRadians < 0){
					OL_elecRadians = 6.28f;
				}
				error_elecRadians = OL_elecRadians - actual_elRadians;
				posPID.out = ExecPID(&posPID,OL_elecRadians,actual_elRadians,-0.5,0.5);
				svpwm.voltagePercent = r.currentDutyF/TIMER1_ARR;
				correctionPercent = posPID.out;
				out2  = svpwm.voltagePercent + correctionPercent;
				foc.m = svpwm.voltagePercent;
				if (addCorrectionPercent){
					foc.m = out2;
				}
				FOC_calcSVPWM(&svpwm,foc.m,OL_elecRadians,0);//PI_BY_2F);
				FOC_applyPWM(&svpwm,0,ms.reversePhases);
				lastOL_Counter = svpwm.loopCounter;
			}
		}

		//only changing the duty cycle and SVPWM
		if (r.rampPhase!=RAMP_WAIT){
			svpwm.voltagePercent = r.currentDutyF/TIMER1_ARR;
			fr.inst_frictionAddition = 0;
			cg.inst_coggingAddition = 0;
			if (fr.frictionCompensationOn){
				lookupFrictionAddition(&fr,ps.encoder_raw); // NOT sure why sign is changing for the two sides..
			}
			if (cg.coggingCompensationOn){
				lookupCoggingAddition(&cg,ps.elecRadians);
			}
			if (direction == CW){
				foc.m = svpwm.voltagePercent + fr.inst_frictionAddition + cg.inst_coggingAddition;//
				FOC_calcSVPWM(&svpwm,foc.m,ps.elecRadians,PI_BY_3F+ms.encCW_offset);
				FOC_applyPWM(&svpwm,0,ms.reversePhases);
			}else if (direction == CCW){
				foc.m = svpwm.voltagePercent - fr.inst_frictionAddition + cg.inst_coggingAddition;//
				FOC_calcSVPWM(&svpwm,foc.m,ps.elecRadians,-PI_BY_3F+ms.encCCW_offset);
				FOC_applyPWM(&svpwm,0,ms.reversePhases);
			}else{}
		} //closes RAMP wait

		if (rampRPM.rampPhase!=RAMP_WAIT){
			//quick and dirty voltage pid has no anti windup and is limited to only positive nos
			speedPID.out = ExecVoltagePID(&speedPID,rampRPM.instTargetRPM_F,fabs(ps.velocityRPM),0,800); // voltage PID running inside the Hight Task Freq!
			if (fabs(speedPID.error >300)){
				StopAllPWM(&hw)	;
				err.globalErrorFlag = 1;
				err.PID_error = 1;
			}else{
				svpwm.voltagePercent = speedPID.out/TIMER1_ARR;

				fr.inst_frictionAddition = 0;
				cg.inst_coggingAddition = 0;
				if (fr.frictionCompensationOn){
					lookupFrictionAddition(&fr,ps.encoder_raw); // NOT sure why sign is changing for the two sides..
				}
				if (cg.coggingCompensationOn){
					lookupCoggingAddition(&cg,ps.elecRadians);
				}

				if (direction == CW){
					foc.m = svpwm.voltagePercent + fr.inst_frictionAddition + cg.inst_coggingAddition;//
					FOC_calcSVPWM(&svpwm,foc.m,ps.elecRadians,PI_BY_3F+ms.encCW_offset);
					FOC_applyPWM(&svpwm,0,ms.reversePhases);
				}else if (direction == CCW){
					foc.m = svpwm.voltagePercent - fr.inst_frictionAddition + cg.inst_coggingAddition;//
					FOC_calcSVPWM(&svpwm,svpwm.voltagePercent,ps.elecRadians,-PI_BY_3F+ms.encCCW_offset);
					FOC_applyPWM(&svpwm,0,ms.reversePhases);
				}else{}
			}

		} //closes RAMP wait

		//Logging
		if (hsLog.enable){
			if(hsLog.addDatas == 0){
				hsLog.startLoopIndex = svpwm.loopCounter;
			}
			if (hsLog.bufferIndex < HSARRAYSIZE - HSLOGSIZE-1){
				hsLog.bufferIndex = addData(&hsLog, &svpwm, &ps,&foc,&fr,&cg);
				hsLog.addDatas+=1;
			}else{
				switchBuffers(&hsLog);
				hsLog.bufferIndex  = 0;
				hsLog.bufferIndex = addData(&hsLog, &svpwm, &ps,&foc,&fr,&cg);
				hsLog.addDatas+=1;
			}
		}

		dt2 = SysTick->VAL - t0;

	}//closes tim1if

	if (htim->Instance ==TIM6){ // 20ms Loop

		if (r.rampPhase != RAMP_WAIT){
			ExecRampDuty(&r);
			if (r.rampPhase == RAMP_WAIT){
				turnOffPWMS = 1;
			}
		}

		if (rampRPM.rampPhase != RAMP_WAIT){
			ExecRampRPM(&rampRPM);
			if (rampRPM.rampPhase == RAMP_WAIT){
				turnOffPWMS = 1;
			}
		}
		/*if (msLogOn){
			//sprintf(UART_buffer,"F:%06.2f,%06.2f,%06.2f,%06.2f,%06.2f,%06.2f,%06.2f,%06.2f,%06.2f:E\r\n",rampRPM.instTargetRPM_F,s.RPM,speedPID.error,foc.IqRef,foc.IdRef,foc.Iq,foc.Id,speedPID.KpTerm,speedPID.KiTerm);
			//HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,70);
		}*/
	}

	if (htim->Instance ==TIM7){ // 1ms Loop
		 ExecPosTrajectory(&p);
	}

}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  if(hadc==&hadc2)
  {
	  adc.FetTemp_ADC =ADC2_buff[0];
	  adc.motorTemp_ADC =ADC2_buff[1];
	  adc.DCV_ADC =ADC2_buff[2];
	  adc.updated = 1;
  }
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_CORDIC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  //Calibrate ADCs
  HAL_Delay(100);
  HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
  HAL_Delay(100);

  HW_statesInit(&hw);
  hsLogInit(&hsLog);

  /*This board has no external Eeprom. Till we add the code to use the
  internal flash as an eeprom, we have to keep seperate codes for the 45mm stack
  and the 30 mm stack motor*/

  //---45 mm Values ----//
  ms.encAvg_offset = 5210;//d axis 45mmstack -> Daxis 5210,
  ms.reversePhases = 1;
  ms.encCW_offset = 0; //find these nos by watching where the current is lowest for say 1000 rpm in closed loop
  ms.encCCW_offset = -1.3;
  //---30mm stack values ---//
//  ms.encAvg_offset = 3706;
//  ms.reversePhases = 1;
//  ms.encCW_offset = 0; //find these nos by watching where the current is lowest for say 1000 rpm in closed loop
//  ms.encCCW_offset = -1.3;

  setupOK = Encoder_setup(); //setup ABI mode without PWM
  encZeroPosSetup = Encoder_updateZeroPosition(ms.encAvg_offset);
  errorSetupOK = Encoder_enableMagErrors();
  Cordic_setup(hcordic,sCordicConfig);

  //position trajectory setup
  Init_ConstJerk_PosControl(&p);
  Reset_posControlJC(&p);

  r.rampPhase = RAMP_WAIT;
  rampRPM.rampPhase = RAMP_WAIT;

  // plotted and found Kp ki constant values with dbgTorque..
  Init_PID_Terms(&speedPID,3.0f,0.4f,0.0f,0.05f);
  Init_PID_Terms(&posPID,3.0f,0.001f,0.00f,0.005f);

  //only start the timer after youve done calibration properly, cos your reading the ADC in the timer also.
  HAL_TIM_Base_Start_IT(&htim1);
  htim1.Instance->RCR = 1; // If its after the counter has started, interrupt is on the OVF, and division of interrupt is at (RCR+1), so for 1, divide by 2, for 0->1

  HAL_TIM_Base_Start_IT(&htim6); // 20 ms interrupt
  HAL_TIM_Base_Start_IT(&htim7); // 1 ms interrupt
  //first values of the position sensor are noisy. so start reading it here, wait
  //for a while and then turn it off
  readPosition = 1;
  HAL_Delay(10);
  readPosition = 0;

  setMaxFrictionPWM(&fr,frictionMaxPWM);
  setMaxCoggingPWM(&cg,coggingMaxPWM);
  fr.frictionCompensationOn = 0;
  cg.coggingCompensationOn = 0;

  //Start the ADCs -only for DC voltage and Temperature.
  HAL_TIM_Base_Start_IT(&htim4);//start timer for adc2 trigger
  HAL_ADC_Start_DMA(&hadc2,(uint32_t*)ADC2_buff,3);
  HAL_Delay(100);
  adc.FET_thermistor_open = checkThermistorOpen(adc.FetTemp_ADC);
  adc.motor_thermistor_open = checkThermistorOpen(adc.motorTemp_ADC);

  if (adc.FET_thermistor_open == 1){
	  err.globalErrorFlag = 1;
	  err.fetThermistorOpen = 1;
  }
  if (adc.motor_thermistor_open == 1){
	  err.globalErrorFlag = 1;
	  err.windingsThermistorOpen = 1;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (adc.updated){
		  adc.FetTemp_C = get_temperature(adc.FetTemp_ADC);
		  adc.motorTemp_C = get_temperature(adc.motorTemp_ADC);
		  if (adc.FetTemp_C > 65){
			  err.globalErrorFlag = 1;
			  err.temp_FET_outOfBounds = 1;
		  }
		  if (adc.motorTemp_C > 90){
			  err.globalErrorFlag = 1;
			  err.temp_windings_outOfBounds = 1;
		  }

		  adc.updated=0;
	  }

	  if (err.globalErrorFlag == 1){
		  StopAllPWM(&hw);
		  while(1){
			  err.counter++;
			  HAL_Delay(1000);
		  }
	  }

	  if (turnOffPWMS){
		  StopAllPWM(&hw);
		  turnOffPWMS = 0;
	  }

	  if (checkEncoderHealth){
		  encoderNOK = Encoder_checkHealth();
		  checkEncoderHealth = 0;
	  }

	  if (turnOff_sensors){
		  readPosition = 0;
		  turnOff_sensors = 0;
		  turnOffPWMS = 1;
	  }

	  if (setupPosJC){
		  Reset_posControlJC(&p);
		  Setup_posControlJC(&p,pos_targetThetaDeg,pos_targetTime_ms);
		  setupPosJC = 0;
	  }

	  if (startPosC){
		  p.state = POS_RUNNING;
		  startPosC = 0;
	  }
	  if (stopPosC){
		  p.state = POS_OVER;
		  stopPosC = 0;
	  }

	  if (setupFrictionAddition){
		  setMaxFrictionPWM(&fr,frictionMaxPWM);
		  setupFrictionAddition = 0;
	  }

	  if (setupCoggingAddition){
		  setMaxCoggingPWM(&cg,coggingMaxPWM);
		  setupCoggingAddition = 0;
	  }
	  /*RMF control!
	  change d_elec Radians. MAke negative to change direction.
	  start with a value of 0.005 and slowly increase or decrease.
	  V.IMP -> motor has to rotate clockwise if d_elec is positive*/
	  if (OL_init){ // do with a position Loop
		  //turn on the PWMS
		  readPosition = 1;
		  d_elecRadians = 0.001;
		  r.currentDutyF = 100;
		  //turn on the PWMS
		  ZeroAllCCRs(&svpwm);
		  StartAllPWM(&hw);
		  runOL = 1;
		  OL_init = 0;
		  //pid stuff
		  HAL_Delay(100); // time for a reading to be taken.
		  OL_offsetRadians = ps.elecRadians;
	  }

	  if (OL_stop){
		  runOL = 0;
		  StopAllPWM(&hw);
		  turnOff_sensors =1;
		  OL_stop = 0;
	  }


	  /*---------------*/
	  //RUN with some DUTY CYCLE
	  if (dbg_rampDuty_RUStart){
		readPosition = 1;
		//direction = CW ; not set here so u can change direction in debug mode. 1 or -1 ONLY!!
		//fill up the Ramp Duty Struct- TargetRPRm,rampUp Time, rampDownTime, and steadY state runTime
		InitRampDutyStruct(&r,targetDuty,5000,5000,300);
		StartRampDuty(&r);
		//turn on the PWMS
		ZeroAllCCRs(&svpwm);
		StartAllPWM(&hw);
		dbg_rampDuty_RUStart = 0;
	  }

	  if (dbg_rampDuty_RDStart){
		  StartRampDownDuty(&r);
		  dbg_rampDuty_RDStart= 0;
		  // change this later to keep track of where rpm becomes zero,
		  // and there make swithc of the PWMs
	  }

	  //Set transition Target and Transition Time in RM
	  if(dbg_rampDuty_ChangeDuty){
		  ChangeDuty(&r);
		  Recalculate_RampDuty_RampRates(&r,r.transitionTarget);
		  r.rampPhase = RAMP_CHANGE;
		  dbg_rampDuty_ChangeDuty = 0;
	  }

	  if (dbg_rampDuty_Stop){
		  // we need to stop the six sector Obj, and then stop the Ramp
		  StopAllPWM(&hw);
		  StopRampDuty(&r);
		  dbg_rampDuty_Stop =0;
	  }


	  /*------------------------------------------*/
	  //run with RPM
	  if (dbg_rampRPM_RUStart){
		//fill up the Ramp Duty Struct- TargetRPRm,rampUp Time, rampDownTime, and steadY state runTime
		InitRampRPMStruct(&rampRPM,targetRPM,10.0f,12.0f,300.0f);
		readPosition = 1;
		Zero_PID_Terms(&speedPID);
		StartRampRPM(&rampRPM); 		// we need to start the Ramp
		ZeroAllCCRs(&svpwm);
		StartAllPWM(&hw); 		//turn on the PWMS
		//need to reset the timer to allow for calculation of Time to take place
		dbg_rampRPM_RUStart = 0;
	  }

	  if (dbg_rampRPM_RDStart){
		  StartRampDownRPM(&rampRPM);
		  dbg_rampRPM_RDStart= 0;
	  }

	  //Set transition Target and Transition Time in RM
	  if(dbg_rampRPM_ChangeRPM){
		  ChangeRPM(&rampRPM,RPM_transitionTarget,RPM_transitionTime);
		  Recalculate_RampRPM_RampRates(&rampRPM,RPM_transitionTarget);
		  rampRPM.rampPhase = RAMP_CHANGE;
		  dbg_rampRPM_ChangeRPM = 0;
	  }

	  //STOP CLOSED LOOP
	  if (dbg_rampRPM_Stop){
		  StopAllPWM(&hw);
		  StopRampRPM(&rampRPM);
		  dbg_rampRPM_Stop =0;
	  }

	  /*--------------------------------------------------*/

	  if(checkUartDmA){
		  HAL_UART_Transmit_DMA(&huart2,(uint8_t *)"is this Working????\r\n",22);
		  checkUartDmA = 0;
	  }
	  /*------------------------------------------------*/

	  if (hsLogOn){
		  if (hsLog.firstTime == 0){
			  hsLogStart(&hsLog,&ps);
			  hsLog.end_multiTurns = hsLog.start_multiTurns  + (direction * TWO_PI_F * LOG_TURNS);
			  hsLog.addDatas = 0;
			  totalBytes = 0;
		  }

		  if (hsLog.sendOut == BUFFER1){
			  HAL_UART_Transmit_DMA(&huart2,(uint8_t *)hsLog.HSbuffer1,hsLog.sendOutBufferSize);
			  //hsLog.DMAdataSentOut = 0;
			  hsLog.sendOut = NOBUFFER;
			  totalBytes += hsLog.sendOutBufferSize;
			  hsLog.sendOutBufferSize = 0;
			  buffer1Sent+=1;
		  }
		  else if (hsLog.sendOut == BUFFER2){
			  HAL_UART_Transmit_DMA(&huart2,(uint8_t *)hsLog.HSbuffer2,hsLog.sendOutBufferSize);
			  //hsLog.DMAdataSentOut = 0;
			  hsLog.sendOut = NOBUFFER;
			  totalBytes += hsLog.sendOutBufferSize;
			  hsLog.sendOutBufferSize = 0;
			  buffer2Sent+=1;
		  }
		  else {}
		  hsLog_CheckStopConditionReached(&hsLog,&ps,direction);
		  if (hsLog.stopAndFlushBuffer){
			  //stop  the log, send out whatever data
			 hsLog.enable = 0;
			 hsLog.endLoopIndex = svpwm.loopCounter;
			 if (hsLog.runningBuffer == BUFFER1){
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)hsLog.HSbuffer1,hsLog.bufferIndex);
				buffer1flush+=1;
				totalBytes += hsLog.sendOutBufferSize;
			 }else{
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)hsLog.HSbuffer2,hsLog.bufferIndex);
				buffer2flush+=1;
				totalBytes += hsLog.sendOutBufferSize;
			 }
			 hsLogOn = 0;
		  }
	  }else{
		  hsLogReset(&hsLog);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T4_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 5;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1249;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 90;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1249;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2399;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1249;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2399;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1499;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 99;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 4800000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin|FAULT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PG10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA6 PA7 PA11
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_INDEX_Pin */
  GPIO_InitStruct.Pin = ENC_INDEX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_INDEX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB11
                           PB12 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin FAULT_LED_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|FAULT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
