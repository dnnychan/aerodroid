/*******************************************************************************
 * @file
 * @purpose        
 * @version        0.1
 *------------------------------------------------------------------------------
 * Copyright (C) 2011 Gumstix Inc.
 * All rights reserved.
 *
 * Contributer(s):
 *   Danny Chan   <danny@gumstix.com>
 *------------------------------------------------------------------------------
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 * 
 * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <math.h>

#include "return.h"
#include "table.h"
#include "extras.h"

#include "LPC17xx.h"
#include "lpc_types.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_can.h"
#include "lpc17xx_pwm.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_systick.h"
#include "cr_dsplib.h"
#include "aeroangle.h"
#include "aeroflight.h"
#include "aero.h"


/*****************************************************************************

        Hardware Initialization Routine

*****************************************************************************/ 
int Init_PWM(uint8_t MatchChannel,uint32_t MatchValue)
{
    PWM_MATCHCFG_Type PWMMatchCfgDat;

    PWM_MatchUpdate(LPC_PWM1, MatchChannel, MatchValue, PWM_MATCH_UPDATE_NOW);
    PWMMatchCfgDat.IntOnMatch = DISABLE;
    PWMMatchCfgDat.MatchChannel = MatchChannel;
    if (!MatchChannel)
        PWMMatchCfgDat.ResetOnMatch = ENABLE;
    else
        PWMMatchCfgDat.ResetOnMatch = DISABLE;
    PWMMatchCfgDat.StopOnMatch = DISABLE;
    PWM_ConfigMatch(LPC_PWM1, &PWMMatchCfgDat);

    return 0;
}

int twosComplement(uint8_t low_byte, uint8_t high_byte)
{
  return ((int)((low_byte + (high_byte << 8)) + pow(2,15)) % (int)pow(2,16) - pow(2,15));
}

int writeReg(I2C_M_SETUP_Type* device,uint8_t reg, uint32_t value)
{
    device->tx_data[0] = reg;
    device->tx_data[1] = value;
    device->tx_length = 2;
    device->rx_data = 0;
    device->rx_length = 0;
    ret = I2C_MasterTransferData(LPC_I2C0, device, I2C_TRANSFER_POLLING);
    return 0;
}

uint8_t* read6Reg(I2C_M_SETUP_Type* device,uint8_t reg, uint8_t* rx_data6)
{
    device->tx_data[0] = reg | 0b10000000; //MSB must be equal to 1 to read multiple bytes
    device->tx_length = 1;
    device->rx_data = rx_data6;
    device->rx_length = 6;
    ret = I2C_MasterTransferData(LPC_I2C0, device,I2C_TRANSFER_POLLING);
    return rx_data6;
}

int aeroInit(uint8_t * args)
{ 
    uint8_t * arg_ptr;
    I2C_M_SETUP_Type* accelerometer;
    I2C_M_SETUP_Type* gyro;
    
    if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
    accelerometer = (I2C_M_SETUP_Type*) strtoul((char *) arg_ptr, NULL, 16);
    if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
    gyro = (I2C_M_SETUP_Type*) strtoul((char *) arg_ptr, NULL, 16);
    
    uint8_t accel_ctrl_reg1 = 0x20;
    uint8_t accel_ctrl_reg4 = 0x23;
    
    uint8_t gyro_ctrl_reg1 = 0x20;
    //uint8_t gyro_ctrl_reg2 = 0x21;
    uint8_t gyro_ctrl_reg3 = 0x22;
    uint8_t gyro_ctrl_reg4 = 0x23;
    //uint8_t gyro_ctrl_reg5 = 0x24;
    //uint8_t gyro_status_reg = 0x27;
    uint8_t i;

    _roboveroConfig(NULL);
    
    Init_PWM(0, 20000);
    for (i=1; i<5; i++)
    {
      Init_PWM(i, 1250);
      PWM_ChannelCmd(LPC_PWM1, 1, ENABLE);
    }
    
    PWM_ResetCounter(LPC_PWM1);
    PWM_CounterCmd(LPC_PWM1, ENABLE);
    PWM_Cmd(LPC_PWM1, ENABLE);
    
    uint8_t* accel_tx_data=malloc(2*sizeof(uint8_t));
    uint8_t* gyro_tx_data=malloc(2*sizeof(uint8_t));
    
    accelerometer->sl_addr7bit=0x18;
    accelerometer->tx_data=accel_tx_data;
    accelerometer->retransmissions_max=3;
    
    writeReg(accelerometer,accel_ctrl_reg1,0x27);
    writeReg(accelerometer,accel_ctrl_reg4, 0x00);
    
    gyro->sl_addr7bit=0x68;
    gyro->tx_data=gyro_tx_data;
    gyro->retransmissions_max=3;
    
    writeReg(gyro,gyro_ctrl_reg3, 0x08);
    writeReg(gyro,gyro_ctrl_reg4, 0x80);
    writeReg(gyro,gyro_ctrl_reg1, 0x0F);
    
    return 0;
}

void aeroPIDValuesInit(tS_pid_Coeff* PID[10])
{
  //scaled by *100
  PID[ROLL]->Kp = 10000;
  PID[ROLL]->Ki = 00;
  PID[ROLL]->Kd = -30000;
  PID[ROLL]->IntegratedError = 0;
  PID[ROLL]->LastError = 0;
  PID[PITCH]->Kp = 10000;
  PID[PITCH]->Ki = 00;
  PID[PITCH]->Kd = -30000;
  PID[PITCH]->IntegratedError = 0;
  PID[PITCH]->LastError = 0;
  PID[YAW]->Kp = 20000;
  PID[YAW]->Ki = 500;
  PID[YAW]->Kd = 00;
  PID[YAW]->IntegratedError = 0;
  PID[YAW]->LastError = 0;
  PID[LEVELROLL]->Kp = 400;
  PID[LEVELROLL]->Ki = 060;
  PID[LEVELROLL]->Kd = 00;
  PID[LEVELROLL]->IntegratedError = 0;
  PID[LEVELROLL]->LastError = 0;
  PID[LEVELPITCH]->Kp = 400;
  PID[LEVELPITCH]->Ki = 060;
  PID[LEVELPITCH]->Kd = 00;
  PID[LEVELPITCH]->IntegratedError = 0;
  PID[LEVELPITCH]->LastError = 0;
  PID[HEADING]->Kp = 300;
  PID[HEADING]->Ki = 010;
  PID[HEADING]->Kd = 00;
  PID[HEADING]->IntegratedError = 0;
  PID[HEADING]->LastError = 0;
  PID[LEVELGYROROLL]->Kp = 10000;
  PID[LEVELGYROROLL]->Ki = 00;
  PID[LEVELGYROROLL]->Kd = -30000;
  PID[LEVELGYROROLL]->IntegratedError = 0;
  PID[LEVELGYROROLL]->LastError = 0;
  PID[LEVELGYROPITCH]->Kp = 10000;
  PID[LEVELGYROPITCH]->Ki = 00;
  PID[LEVELGYROPITCH]->Kd = -30000;
  PID[LEVELGYROPITCH]->IntegratedError = 0;
  PID[LEVELGYROPITCH]->LastError = 0;
}

int aeroLoop(uint8_t * args)
{
  uint8_t * arg_ptr;
  I2C_M_SETUP_Type* accelerometer;
  I2C_M_SETUP_Type* gyro;
  
  if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
  accelerometer = (I2C_M_SETUP_Type*) strtoul((char *) arg_ptr, NULL, 16);
  if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
  gyro = (I2C_M_SETUP_Type*) strtoul((char *) arg_ptr, NULL, 16);
  
  uint8_t accel_x_low = 0x28;
  uint8_t gyro_x_low = 0x28;
  uint8_t* accel_raw=malloc(6*sizeof(uint8_t));
  uint8_t* gyro_raw=malloc(6*sizeof(uint8_t));
  
  VECTOR accel_data, gyro_data;
  float accel_OneG=9.8;
  int i;
  FLIGHT_ANGLE_TYPE* flight_angle = flightAngleInitialize(1.0, 0.0);
  MOTORS_TYPE* motors = motorsInit();
  tS_pid_Coeff* PID[10];
  
  for (i=0; i<10; i++)
    PID[i]=(tS_pid_Coeff*)malloc(sizeof(tS_pid_Coeff));

  aeroPIDValuesInit(PID);
    
  while (1)
  {
    for (i=0; i<72000; i++);
    
    accel_raw=read6Reg(accelerometer,accel_x_low,accel_raw);
    gyro_raw=read6Reg(gyro,gyro_x_low,gyro_raw);
    
    accel_data.x=twosComplement(accel_raw[0], accel_raw[1])/835.9;
    accel_data.y=twosComplement(accel_raw[2], accel_raw[3])/835.9;
    accel_data.z=twosComplement(accel_raw[4], accel_raw[5])/835.9;
    
    gyro_data.x=twosComplement(gyro_raw[0], gyro_raw[1])/3754.9;//131.072;
    gyro_data.y=twosComplement(gyro_raw[2], gyro_raw[3])/3754.9;//131.072;
    gyro_data.z=twosComplement(gyro_raw[4], gyro_raw[5])/3754.9;//131.072;
    
    flightAngleCalculate(flight_angle, gyro_data.x, gyro_data.y, gyro_data.z, accel_data.x, accel_data.y, accel_data.z, accel_OneG, 1, 0);
    
    //sprintf((char *) str, "accel=[%i,%i,%i]\r\n", (int)(accel_data.x*1000), (int)(accel_data.y*1000), (int)(accel_data.z*1000));
    //writeUSBOutString(str);
    
    sprintf((char *) str, "gyro=[%i,%i,%i]\r\n", (int)(gyro_data.x*1000), (int)(gyro_data.y*1000), (int)(gyro_data.z*1000));
    writeUSBOutString(str);
    
    printstuff(flight_angle);
    
    processFlightControl(motors, flight_angle, PID, gyro_data);
  }

  return 0;
}
