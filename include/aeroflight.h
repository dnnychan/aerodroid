/*******************************************************************************
 * @file
 * @purpose
 * @version        0.1
 *------------------------------------------------------------------------------
 * Copyright (C) 2012 Gumstix Inc.
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

#ifndef AEROFLIGHT_H_
#define AEROFLIGHT_H_

#include "LPC17xx.h"
#include "lpc_types.h"
#include "lpc17xx_pwm.h"

#include "return.h"
#include "cr_dsplib.h"
#include "PID.h"

#include "aeroangle.h"
#include "aero.h"

float dummy_roll_altitude_cmd;
float dummy_pitch_altitude_cmd;
float dummy_gyro_y;
float dummy_gyro_x;

int _setThrottle(uint8_t * args);
int _getFlightCmds (uint8_t * args);
int _setTargetAltitude (uint8_t * args);

//Motor related commands
typedef struct {
  int axis_command[3];
  int motor_command[LASTMOTOR];
  int min_command[LASTMOTOR];
  int max_command[LASTMOTOR];
} MOTORS_TYPE;

MOTORS_TYPE* motorsInit(void);
void setMotorAxisCommand(MOTORS_TYPE*motors, int motor, int value);
const int getMotorAxisCommand(MOTORS_TYPE*motors, int motor);
void setMotorCommand(MOTORS_TYPE*motors, int motor, int value);
const int getMotorCommand(MOTORS_TYPE*motors, int motor);
void writeMotors(MOTORS_TYPE* motors);

//Flight commands
void processFlightControl(MOTORS_TYPE* motors, FLIGHT_ANGLE_TYPE* flight_angle, PID_TYPE* PID[10], VECTOR gyro_data, uint32_t altitude, int altitude_control);

#endif
