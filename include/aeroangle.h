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

#ifndef AEROANGLE_H_
#define AEROANGLE_H_

#include "LPC17xx.h"
#include "lpc_types.h"

#include "aero.h"
#include "cr_dsplib.h"
#include "matrix_math.h"

typedef struct {
  float angle[3];
  float gyro_angle[2];
  float corrected_rate_vector[3];
  float earth_accel[3];
  float dcm_matrix[9];
  float omegaP[3];
  float omegaI[3];
  float omega[3];
  float error_course;
  float kp_roll_pitch;
  float ki_roll_pitch;
  float kp_yaw;
  float ki_yaw;
  float delta_t
} FLIGHT_ANGLE_TYPE;

int _changeMew (uint8_t * args);
int _getBalanceAngles(uint8_t * args);

FLIGHT_ANGLE_TYPE* flightAngleInitialize(float hdgX, float hdgY);
float getAngle(FLIGHT_ANGLE_TYPE* flight_angle, uint8_t axis);
void flightAngleCalculate(FLIGHT_ANGLE_TYPE* flight_angle, \
    float rollRate,           float pitchRate,     float yawRate, \
    float longitudinalAccel,  float lateralAccel,  float verticalAccel, \
    float oneG,               float magX,          float magY);

#endif
