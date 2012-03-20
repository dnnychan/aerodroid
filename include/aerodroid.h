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

#ifndef __AERODROID_H__
#define __AERODROID_H__

#include "lpc_types.h"
#include "lpc17xx_i2c.h"
#include "LPC17xx.h"

#include <math.h>

#include "return.h"
#include "table.h"
#include "extras.h"

#include "aero.h"
#include "aeroangle.h"
#include "aeroflight.h"

I2C_M_SETUP_Type* accelerometer;
I2C_M_SETUP_Type* gyro;

uint8_t* accel_raw;
uint8_t* gyro_raw;

VECTOR accel_data, gyro_data;

tS_pid_Coeff* PID[10];
FLIGHT_ANGLE_TYPE* flight_angle;
MOTORS_TYPE* motors;

int twosComplement(uint8_t low_byte, uint8_t high_byte);
void writeReg(I2C_M_SETUP_Type* device,uint8_t reg, uint32_t value);
uint8_t* read6Reg(I2C_M_SETUP_Type* device,uint8_t reg, uint8_t* rx_data6);
int aeroInit(uint8_t * args);
void aeroLoop(uint8_t * args);
int _aeroLoopOff(uint8_t * args);
int _aeroLoopOn(uint8_t * args);
int _getMotorCommands(uint8_t* args);
void stopAllMotors(void);

#endif

