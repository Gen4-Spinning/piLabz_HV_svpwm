#include "main.h"
#include "AS5x47P.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
extern SPI_HandleTypeDef hspi1;

uint8_t txbuf[2];
uint8_t rxbuf[2];
uint8_t finalrxbuf[2];
uint16_t finalDataVal;
float readdata;


uint8_t checkReadForError(uint16_t data){
	return (data & 0x4000)>>15; // check if 15th bit is 1.
}

// Check Parity of a given data.
bool parityCheck(uint16_t data){
  uint16_t count=0;
  uint16_t b = 1;
  for (int i=0; i<15; i++){
    if (data & (b << i)) {
      count++;
    }
  }

  if (count%2==0) {
    return 0;
  } else {
    return 1;
  }
}

//SPI_write and read are polling Fns;Timeout is set for 100ms
uint8_t AS5047_SPI_Write(uint16_t addressFrame, uint16_t valueFrame) {

  //write which address needed to be updated.
  txbuf[0] = (addressFrame) >> 8;
  txbuf[1] = addressFrame & 0xFF;
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txbuf, (uint8_t*)&rxbuf, 2, 100);
  while( hspi1.State == HAL_SPI_STATE_BUSY );
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  uint16_t recievedFrame = ((uint16_t)rxbuf[0])<<8;
  recievedFrame += rxbuf[1];

  //this is value that will be stored inside the respective register
  txbuf[0] = (valueFrame) >> 8;
  txbuf[1] = valueFrame & 0xFF;
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txbuf, (uint8_t*)&rxbuf, 2, 100);
  while( hspi1.State == HAL_SPI_STATE_BUSY );
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  recievedFrame = ((uint16_t)rxbuf[0])<<8;
  recievedFrame += rxbuf[1];

  //check if data has been written properly, will receive the written value in the recieve buffer
  txbuf[0] = NOP_FRAME >> 8;
  txbuf[1] = NOP_FRAME & 0xFF;
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txbuf, (uint8_t*)&rxbuf, 2, 100);
  while( hspi1.State == HAL_SPI_STATE_BUSY );
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  //DEBUG WHY THIS IS NOT WORKING LATER.
  //check if recieved data is same as what we wanted to write
  recievedFrame = ((uint16_t)rxbuf[0])<<8;
  recievedFrame += rxbuf[1];

  uint16_t receivedData = recievedFrame & 0x3FFF;
  uint16_t writtenData = valueFrame & 0x3FFF;

  if ((writtenData) == receivedData){
	  return 1;
  }

  return 0;
}


uint16_t AS5047_SPI_Read(uint16_t command, uint8_t continuousRead) {
  //write command frame.
  uint16_t finalDataVal;
  txbuf[0] = command >> 8U;
  txbuf[1] = command & 0xFF;
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txbuf, (uint8_t*)&rxbuf,2,100);
  while( hspi1.State == HAL_SPI_STATE_BUSY );
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  if (continuousRead ==0){
	  //this is the NOP command frame for receiving data if you want to read the register once.
	  txbuf[0] = NOP_FRAME >> 8;
	  txbuf[1] = NOP_FRAME & 0xFF;
	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txbuf, (uint8_t*)&rxbuf,2,100);
	  while( hspi1.State == HAL_SPI_STATE_BUSY );
	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  }
  finalDataVal = ((uint16_t)rxbuf[0])<<8;
  finalDataVal += rxbuf[1];

  return finalDataVal;

}

uint16_t AS5047_readRegister(uint16_t registerAddress,uint8_t continuousRead){
  CommandFrame command;
  command.raw = 0;
  command.values.rw = READ;
  command.values.commandFrame = registerAddress;
  command.values.parc = parityCheck(command.raw);
  uint16_t out= AS5047_SPI_Read(command.raw, continuousRead);
  return out;
}


void AS5047_writeRegister(uint16_t registerAddress, uint16_t registerValue) {
  CommandFrame command;
  command.raw=0;
  command.values.rw = WRITE;
  command.values.commandFrame = registerAddress;
  command.values.parc = parityCheck(command.raw);

  WriteDataFrame contentFrame;
  contentFrame.raw = 0;
  contentFrame.values.data = registerValue;
  contentFrame.values.low = 0; //always low
  contentFrame.values.pard = parityCheck(contentFrame.raw);
  //TODO Use the out in the below fn and handle errors
  //uint8_t out = AS5047_SPI_Write(command.raw, contentFrame.raw);
  AS5047_SPI_Write(command.raw, contentFrame.raw);
}

uint16_t AS5047_ReadZeroValue(void){
  ZPOSH_frame ZPOS_H;
  ZPOSL_frame ZPOS_L;

  ZPOS_H.raw = AS5047_readRegister(ZPOSM_REG,0);
  ZPOS_L.raw = AS5047_readRegister(ZPOSL_REG,0);

  uint16_t zpos = (ZPOS_H.raw << 6) + (ZPOS_L.raw & 0x3F);
  return zpos;
}


void AS5047_WriteZeroValue(uint16_t zeroValue){
  ZPOSH_frame ZPOS_H;
  ZPOSL_frame ZPOS_L;

  ZPOS_L.values.zposl = zeroValue & 0x003F;
  ZPOS_H.values.zposh = (zeroValue >> 6) & 0x00ff;

  AS5047_writeRegister(ZPOSM_REG, ZPOS_H.raw);
  AS5047_writeRegister(ZPOSL_REG, ZPOS_L.raw);

}


