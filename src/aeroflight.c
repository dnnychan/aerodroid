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

#include "aeroflight.h"

int throttle=1000;
int target_altitude = 15;

int _setThrottle(uint8_t * args)
{
  uint8_t * arg_ptr;

  if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
  throttle = (int) strtoul((char *) arg_ptr, NULL, 16);
  
  return 0;
}

int _setTargetAltitude (uint8_t * args)
{
  uint8_t * arg_ptr;

  if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
  target_altitude = (int) strtoul((char *) arg_ptr, NULL, 16);
    
  return 0;
}

MOTORS_TYPE* motorsInit(void) {
  MOTORS_TYPE * motors= (MOTORS_TYPE*) malloc(sizeof(MOTORS_TYPE));
  int i;
  motors->axis_command[ROLL] = 0;
  motors->axis_command[PITCH] = 0;
  motors->axis_command[YAW] = 0;
  for (i=0; i<LASTMOTOR; i++) {
    motors->min_command[i]=MINCOMMAND;
    motors->motor_command[i]=MINCOMMAND;
    motors->max_command[i]=MAXCOMMAND;
  }
  return motors;
}

void setMotorAxisCommand(MOTORS_TYPE*motors, int motor, int value) {
  motors->axis_command[motor] = value;
}

const int getMotorAxisCommand(MOTORS_TYPE*motors, int motor) {
  return motors->axis_command[motor];
}

void setMotorCommand(MOTORS_TYPE*motors, int motor, int value) {
  motors->motor_command[motor] = value;
}

const int getMotorCommand(MOTORS_TYPE*motors, int motor) {
  return motors->motor_command[motor];
}

const int getMaxCommand(MOTORS_TYPE*motors, int motor) {
  return motors->max_command[motor];
}

const int getMinCommand(MOTORS_TYPE*motors, int motor) {
  return motors->min_command[motor];
}

void writeMotors(MOTORS_TYPE* motors) {
  PWM_MatchUpdate(LPC_PWM1, FRONT+5, (uint32_t)getMotorCommand(motors, FRONT), PWM_MATCH_UPDATE_NEXT_RST);
  PWM_MatchUpdate(LPC_PWM1, REAR+1,  (uint32_t)getMotorCommand(motors, REAR),  PWM_MATCH_UPDATE_NEXT_RST);
  PWM_MatchUpdate(LPC_PWM1, RIGHT+1, (uint32_t)getMotorCommand(motors, RIGHT), PWM_MATCH_UPDATE_NEXT_RST);
  PWM_MatchUpdate(LPC_PWM1, LEFT+1,  (uint32_t)getMotorCommand(motors, LEFT),  PWM_MATCH_UPDATE_NEXT_RST);
}

void calculateFlightError(MOTORS_TYPE* motors, FLIGHT_ANGLE_TYPE* flight_angle, PID_TYPE* PID[10], VECTOR gyro_data) {
  //float rollAttitudeCmd = updatePID((receiver.getData(ROLL) - receiver.getZero(ROLL)) * ATTITUDE_SCALING, flightAngle->getData(ROLL), &PID[LEVELROLL]);
  //float pitchAttitudeCmd = updatePID((receiver.getData(PITCH) - receiver.getZero(PITCH)) * ATTITUDE_SCALING, -flightAngle->getData(PITCH), &PID[LEVELPITCH]);
  //motors.setMotorAxisCommand(ROLL, updatePID(rollAttitudeCmd, gyro.getData(ROLL), &PID[LEVELGYROROLL]));
  //motors.setMotorAxisCommand(PITCH, updatePID(pitchAttitudeCmd, -gyro.getData(PITCH), &PID[LEVELGYROPITCH]));
  
  float roll_altitude_cmd  = doPID( 0,  getAngle(flight_angle, ROLL),  PID[LEVELROLL]);
  float pitch_altitude_cmd = doPID( 0, -getAngle(flight_angle, PITCH), PID[LEVELPITCH]);
  
  dummy_roll_altitude_cmd = roll_altitude_cmd;
  dummy_pitch_altitude_cmd = pitch_altitude_cmd;
  dummy_gyro_y = gyro_data.y;
  dummy_gyro_x = -gyro_data.x;

  setMotorAxisCommand(motors, ROLL,  doPID( roll_altitude_cmd,  gyro_data.y, PID[LEVELGYROROLL]));
  setMotorAxisCommand(motors, PITCH, doPID(pitch_altitude_cmd, +gyro_data.x, PID[LEVELGYROPITCH]));
}

void processAltitudeControl(uint32_t altitude_data, PID_TYPE* PID[10]) 
{
  int throttle_adjust = doPID(target_altitude, altitude_data, PID[ALTITUDE]);
  
  throttle = throttle + throttle_adjust;
}

int _getFlightCmds (uint8_t * args)
{
  sprintf((char *) str, "%x %x %x %x\r\n",(int)(dummy_gyro_y*1000), (int)(dummy_gyro_x*1000), (int)(dummy_roll_altitude_cmd*1000), (int)(dummy_pitch_altitude_cmd*1000));
  writeUSBOutString(str);
  
  return 0;
}

void processFlightControl(MOTORS_TYPE* motors, FLIGHT_ANGLE_TYPE* flight_angle, PID_TYPE* PID[10], VECTOR gyro_data, uint32_t altitude_data, int altitude_control)
{
  int motor;

  calculateFlightError(motors, flight_angle, PID, gyro_data);

  //do some yaw thing

  //altitude
  if (altitude_control)
    processAltitudeControl(altitude_data, PID);

  // Plus mode
  setMotorCommand(motors, FRONT, throttle - getMotorAxisCommand(motors, PITCH) - getMotorAxisCommand(motors, YAW));
  setMotorCommand(motors, RIGHT, throttle - getMotorAxisCommand(motors, ROLL)  + getMotorAxisCommand(motors, YAW));
  setMotorCommand(motors, LEFT,  throttle + getMotorAxisCommand(motors, ROLL)  + getMotorAxisCommand(motors, YAW));
  setMotorCommand(motors, REAR,  throttle + getMotorAxisCommand(motors, PITCH) - getMotorAxisCommand(motors, YAW));

  // X mode
  /*setMotorCommand(motors, FRONT, throttle - getMotorAxisCommand(motors, PITCH) + getMotorAxisCommand(motors, ROLL) - getMotorAxisCommand(motors, YAW));
  setMotorCommand(motors, RIGHT, throttle - getMotorAxisCommand(motors, PITCH) - getMotorAxisCommand(motors, ROLL) + getMotorAxisCommand(motors, YAW));
  setMotorCommand(motors, LEFT,  throttle + getMotorAxisCommand(motors, PITCH) + getMotorAxisCommand(motors, ROLL) + getMotorAxisCommand(motors, YAW));
  setMotorCommand(motors, REAR,  throttle + getMotorAxisCommand(motors, PITCH) - getMotorAxisCommand(motors, ROLL) - getMotorAxisCommand(motors, YAW));
  */
  //maxmin
  for (motor = FRONT; motor < LASTMOTOR; motor++) {
    if (getMotorCommand(motors, motor) > getMaxCommand(motors,motor))
      setMotorCommand(motors, motor, getMaxCommand(motors,motor));
    else if (getMotorCommand(motors, motor) < getMinCommand(motors,motor))
      setMotorCommand(motors, motor, getMinCommand(motors,motor));
  }

  //writeMotors(motors);
}
