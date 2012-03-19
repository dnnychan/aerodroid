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

#include <math.h>
#include "table.h"
#include "return.h"

#include "aeroangle.h"

#include "LPC17xx.h"
#include "lpc_types.h"

#include "cr_dsplib.h"
#include "matrix_math.h"

void printmatrix(float * matrix, char hi)
{
  sprintf((char *) str, "%c=[%i, %i, %i]\r\n",hi,(int)(matrix[0]*1000000), (int)(matrix[1]*1000000), (int)(matrix[2]*1000000));
  writeUSBOutString(str);
}

FLIGHT_ANGLE_TYPE* flightAngleInitialize(float hdgX, float hdgY)
{
  int axis, i;
  FLIGHT_ANGLE_TYPE* flight_angle = (FLIGHT_ANGLE_TYPE*) malloc(sizeof(FLIGHT_ANGLE_TYPE));

  for (axis = ROLL; axis < LASTAXIS; axis++)
    flight_angle->angle[axis] = 0.0;
  flight_angle->gyro_angle[ROLL] = 0;
  flight_angle->gyro_angle[PITCH] = 0;

  for (i=0; i<3; i++) {
    flight_angle->omegaP[i] = 0;
    flight_angle->omegaI[i] = 0;
  }

  flight_angle->dcm_matrix[0] =  hdgX;
  flight_angle->dcm_matrix[1] = -hdgY;
  flight_angle->dcm_matrix[2] =  0;
  flight_angle->dcm_matrix[3] =  hdgY;
  flight_angle->dcm_matrix[4] =  hdgX;
  flight_angle->dcm_matrix[5] =  0;
  flight_angle->dcm_matrix[6] =  0;
  flight_angle->dcm_matrix[7] =  0;
  flight_angle->dcm_matrix[8] =  1;

  // Original from John
//    flight_angle->kp_roll_pitch = 1.6;
//    flight_angle->ki_roll_pitch = 0.005;

//    flight_angle.kpYaw = -1.6;
//    flight_angle.kiYaw = -0.005;

    // released in 2.2
  flight_angle->kp_roll_pitch = 1.0;
  flight_angle->ki_roll_pitch = 0.002;


  //flight_angle->kp_roll_pitch = 0.1;        // alternate 0.05;
  //flight_angle->ki_roll_pitch = 0.0002;     // alternate 0.0001;

  flight_angle->kp_yaw = -1.0;
  flight_angle->ki_yaw = -0.002;

//    flight_angle.kpYaw = -0.1;             // alternate -0.05;
//    flight_angle.kiYaw = -0.0002;          // alternate -0.0001;

  flight_angle->delta_t = 0.01;
  return flight_angle;
}

float getAngle(FLIGHT_ANGLE_TYPE* flight_angle, uint8_t axis)
{
  return flight_angle->angle[axis];//*180/3.14159;
}

void printstuff(FLIGHT_ANGLE_TYPE* flight_angle)
{
  //sprintf((char *) str, "dcm=[%i,%i,%i\r\n  %i,%i,%i\r\n  %i,%i,%i]\r\n", (int)(flight_angle->dcm_matrix[0]*1000), (int)(flight_angle->dcm_matrix[1]*1000), (int)(flight_angle->dcm_matrix[2]*1000), (int)(flight_angle->dcm_matrix[3]*1000), (int)(flight_angle->dcm_matrix[4]*1000), (int)(flight_angle->dcm_matrix[5]*1000), (int)(flight_angle->dcm_matrix[6]*1000), (int)(flight_angle->dcm_matrix[7]*1000), (int)(flight_angle->dcm_matrix[8]*1000));
  //writeUSBOutString(str);
  sprintf((char *) str, "angles=[%i, %i, %i]\r\n",(int)(flight_angle->angle[ROLL]*180/3.14159), (int)(flight_angle->angle[PITCH]*180/3.14159), (int)(flight_angle->angle[YAW]*180/3.14159));
  writeUSBOutString(str);
}

void matrixUpdate (FLIGHT_ANGLE_TYPE* flight_angle, float rollRate, float pitchRate, float yawRate)
{
  float rate_gyro_vector[3];
  float update_matrix[9], product_matrix[9];
  int i;

  rate_gyro_vector[ROLL]  = rollRate;
  rate_gyro_vector[PITCH] = pitchRate;
  rate_gyro_vector[YAW]   = yawRate;

  matrixSubtract(1,3, &(flight_angle->omega), &rate_gyro_vector, &(flight_angle->omegaI));
  matrixSubtract(1,3, &(flight_angle->corrected_rate_vector), &(flight_angle->omega), &(flight_angle->omegaP));

  //printmatrix(&(flight_angle->omega),'w');
  //printmatrix(&(flight_angle->omegaI),'I');
  //printmatrix(&(flight_angle->omegaP),'P');
  //printmatrix(&(rate_gyro_vector),'r');
  //printmatrix(&(flight_angle->corrected_rate_vector),'c');

  update_matrix[0] = 0;
  update_matrix[1] = -flight_angle->delta_t * flight_angle->corrected_rate_vector[YAW];    // -r
  update_matrix[2] =  flight_angle->delta_t * flight_angle->corrected_rate_vector[PITCH];  //  q
  update_matrix[3] =  flight_angle->delta_t * flight_angle->corrected_rate_vector[YAW];    //  r
  update_matrix[4] = 0;
  update_matrix[5] = -flight_angle->delta_t * flight_angle->corrected_rate_vector[ROLL];   // -p
  update_matrix[6] = -flight_angle->delta_t * flight_angle->corrected_rate_vector[PITCH];  // -q
  update_matrix[7] =  flight_angle->delta_t * flight_angle->corrected_rate_vector[ROLL];
  update_matrix[8] = 0;

  //printmatrix(&update_matrix[0],'u');
  //printmatrix(&update_matrix[3],'p');
  //printmatrix(&update_matrix[6],'d');

  matrixMultiply(3 ,3 ,3, &product_matrix, &flight_angle->dcm_matrix, &update_matrix);
  matrixAdd(3,3,&flight_angle->dcm_matrix,&flight_angle->dcm_matrix,&product_matrix);

  //for (i=0; i<9; i++)
  //  flight_angle->dcm_matrix[i]=product_matrix[i];


  //printmatrix(&flight_angle->dcm_matrix[0],'d');
  //printmatrix(&flight_angle->dcm_matrix[3],'c');
  //printmatrix(&flight_angle->dcm_matrix[6],'m');

}

void normalize (FLIGHT_ANGLE_TYPE* flight_angle)
{
  //can be further optimized
  float X[3], Y[3], X_orth[3], Y_orth[3], Z_orth[3], temp[3], temp_dcm[9];
  float error, renorm;
  int i;

  for (i=0; i<9; i++)
    //temp_dcm[i]=floatToQ(flight_angle->dcm_matrix[i]);
    temp_dcm[i]=flight_angle->dcm_matrix[i];

  X[0]=temp_dcm[0];
  X[1]=temp_dcm[1];
  X[2]=temp_dcm[2];

  //printmatrix(&X,'X');

  Y[0]=temp_dcm[3];
  Y[1]=temp_dcm[4];
  Y[2]=temp_dcm[5];

  //printmatrix(&Y,'Y');

  error=.5*vectorDotProduct(3, &X, &Y);
  //sprintf((char *) str, "error=%i\r\n",(int)(error*1000));
  //writeUSBOutString(str);

  matrixScale(1,3,&temp, &Y, error);
  matrixSubtract(1,3,&X_orth, &X, &temp);

  matrixScale(1,3,&temp, &X, error);
  matrixSubtract(1,3,&Y_orth, &Y, &temp);

  vectorCrossProduct(&Z_orth, &X_orth, &Y_orth);

  //printmatrix(&X_orth,'X');
  //printmatrix(&Y_orth,'Y');
  //printmatrix(&Z_orth,'Z');

  renorm=.5*(3-vectorDotProduct(3,&X_orth, &X_orth));
  //sprintf((char *) str, "renorm=%i\r\n",(int)(renorm*1000));
  //writeUSBOutString(str);
  matrixScale(1,3,&(temp_dcm[0]), &X_orth, renorm);

  renorm=.5*(3-vectorDotProduct(3,&Y_orth, &Y_orth));
  //sprintf((char *) str, "renorm=%i\r\n",(int)(renorm*1000));
  //writeUSBOutString(str);
  matrixScale(1,3,&(temp_dcm[3]), &Y_orth, renorm);

  renorm=.5*(3-vectorDotProduct(3,&Z_orth, &Z_orth));
  //sprintf((char *) str, "renorm=%i\r\n",(int)(renorm*1000));
  //writeUSBOutString(str);
  matrixScale(1,3,&(temp_dcm[6]), &Z_orth, renorm);

  for (i=0; i<9; i++)
    //flight_angle->dcm_matrix[i]=QToFloat(temp_dcm[i]);
    flight_angle->dcm_matrix[i]=temp_dcm[i];
}

void driftCorrection(FLIGHT_ANGLE_TYPE* flight_angle, float ax, float ay, float az, float oneG, float magX, float magY)
{
    //  Compensation of the Roll, Pitch and Yaw drift.
  float accel_magnitude;
  float accel_vector[3];
  float accel_weight;
  float error_roll_pitch[3];
  float scaled_omegaI[3];

  accel_vector[XAXIS] = ax;
  accel_vector[YAXIS] = ay;
  accel_vector[ZAXIS] = az;

  accel_magnitude = (sqrt(accel_vector[XAXIS] * accel_vector[XAXIS] + \
                         accel_vector[YAXIS] * accel_vector[YAXIS] + \
                         accel_vector[ZAXIS] * accel_vector[ZAXIS])) / oneG;

  //sprintf((char *) str, "accel_magnitude: %i\r\n",(int)(accel_magnitude*1000));
  //writeUSBOutString(str);

  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  //accel_weight = constrain(1 - 2 * abs(1 - accel_magnitude), 0, 1);
  /*accel_weight = 1 - 2 * abs(1 - accel_magnitude);
  if (accel_weight > 1.0)
    accel_weight = 1.0;
  else if (accel_weight < 0.0)
    accel_weight = 0.0;*/
  accel_weight=1;

  //sprintf((char *) str, "accel_weight: %i\r\n",(int)(accel_weight*1000));
  //writeUSBOutString(str);

  vectorCrossProduct(&error_roll_pitch, &accel_vector, &flight_angle->dcm_matrix[6]);
  //printmatrix(&error_roll_pitch,'e');
  matrixScale(1, 3, &flight_angle->omegaP, &error_roll_pitch, flight_angle->kp_roll_pitch * accel_weight);

  matrixScale(1, 3, &scaled_omegaI, &error_roll_pitch, flight_angle->ki_roll_pitch * accel_weight);
  //printmatrix(&scaled_omegaI,'s');
  matrixAdd(1, 3, &flight_angle->omegaI, &flight_angle->omegaI, &scaled_omegaI);

  flight_angle->omegaP[YAW] = 0.0;
  flight_angle->omegaI[YAW] = 0.0;
}

void eulerAngles(FLIGHT_ANGLE_TYPE* flight_angle)
{
  flight_angle->angle[ROLL]  =  atan2(flight_angle->dcm_matrix[7], flight_angle->dcm_matrix[8]);
  flight_angle->angle[PITCH] =  -asin(flight_angle->dcm_matrix[6]);
  flight_angle->angle[YAW]   =  atan2(flight_angle->dcm_matrix[3], flight_angle->dcm_matrix[0]);
}

void earthAxisAccels(FLIGHT_ANGLE_TYPE* flight_angle, float ax, float ay, float az, float oneG)
{

}

void flightAngleCalculate(FLIGHT_ANGLE_TYPE* flight_angle,
         float rollRate,            float pitchRate,      float yawRate,  \
         float longitudinalAccel,   float lateralAccel,   float verticalAccel, \
         float oneG,                float magX,           float magY)
{
  matrixUpdate(flight_angle, rollRate, pitchRate, yawRate);
  normalize(flight_angle);
  driftCorrection(flight_angle, longitudinalAccel, lateralAccel, verticalAccel, oneG, magX, magY);
  eulerAngles(flight_angle);
  //earthAxisAccels(flight_angle, longitudinalAccel, lateralAccel, verticalAccel, oneG);
}
