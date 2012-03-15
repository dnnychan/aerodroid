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
  uint8_t* accel_data=malloc(6*sizeof(uint8_t));
  uint8_t* gyro_data=malloc(6*sizeof(uint8_t));
  float accel_x, accel_y, accel_z;
  float accel_OneG=9.8;
  float gyro_x, gyro_y, gyro_z;
  int i;
  FLIGHT_ANGLE_TYPE* flight_angle = flightAngleInitialize(1.0, 0.0);
    
  while (1)
  {
    for (i=0; i<72000; i++);
    
    accel_data=read6Reg(accelerometer,accel_x_low,accel_data);
    gyro_data=read6Reg(gyro,gyro_x_low,gyro_data);
    
    accel_x=twosComplement(accel_data[0], accel_data[1])/835.9;
    accel_y=twosComplement(accel_data[2], accel_data[3])/835.9;
    accel_z=twosComplement(accel_data[4], accel_data[5])/835.9;
    
    gyro_x=twosComplement(gyro_data[0], gyro_data[1])/3754.9;//131.072;
    gyro_y=twosComplement(gyro_data[2], gyro_data[3])/3754.9;//131.072;
    gyro_z=twosComplement(gyro_data[4], gyro_data[5])/3754.9;//131.072;
    
    flightAngleCalculate(flight_angle, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, accel_OneG, 1, 0);
    //flightAngleCalculate(flight_angle, 0, 0, 0, 0, 0, 0, accel_OneG, 1, 0);
    //sprintf((char *) str, "angle=[%f,%f,%f]\r\n", getAngle(flight_angle, ROLL), getAngle(flight_angle, PITCH), getAngle(flight_angle, YAW));
    //writeUSBOutString(str);
    
    sprintf((char *) str, "accel=[%i,%i,%i]\r\n", (int)(accel_x*1000), (int)(accel_y*1000), (int)(accel_z*1000));
    writeUSBOutString(str);
    
    sprintf((char *) str, "gyro=[%i,%i,%i]\r\n", (int)(gyro_x*1000), (int)(gyro_y*1000), (int)(gyro_z*1000));
    writeUSBOutString(str);
    
    printstuff(flight_angle);
  }

  return 0;
}
