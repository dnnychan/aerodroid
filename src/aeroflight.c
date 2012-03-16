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

MOTORS_TYPE* motorsInit(void)
{
  MOTORS_TYPE * motors= (MOTORS_TYPE*) malloc(sizeof(MOTORS_TYPE));
  int i;
  motors->axis_command[ROLL] = 0;
  motors->axis_command[PITCH] = 0;
  motors->axis_command[YAW] = 0;
  for (i=0; i<LASTMOTOR; i++) {
    motors->min_command[i]=1250;
    motors->motor_command[i]=0;
    motors->max_command[i]=1500;
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
  PWM_MatchUpdate(LPC_PWM1, FRONT, motors->motor_command[FRONT], PWM_MATCH_UPDATE_NOW);
  PWM_MatchUpdate(LPC_PWM1, REAR, motors->motor_command[REAR], PWM_MATCH_UPDATE_NOW);
  PWM_MatchUpdate(LPC_PWM1, RIGHT, motors->motor_command[RIGHT], PWM_MATCH_UPDATE_NOW);
  PWM_MatchUpdate(LPC_PWM1, LEFT, motors->motor_command[LEFT], PWM_MATCH_UPDATE_NOW);
}

void calculateFlightError(MOTORS_TYPE* motors, FLIGHT_ANGLE_TYPE* flight_angle, tS_pid_Coeff* PID[10]) {
  //float rollAttitudeCmd = updatePID((receiver.getData(ROLL) - receiver.getZero(ROLL)) * ATTITUDE_SCALING, flightAngle->getData(ROLL), &PID[LEVELROLL]);
  //float pitchAttitudeCmd = updatePID((receiver.getData(PITCH) - receiver.getZero(PITCH)) * ATTITUDE_SCALING, -flightAngle->getData(PITCH), &PID[LEVELPITCH]);
  //motors.setMotorAxisCommand(ROLL, updatePID(rollAttitudeCmd, gyro.getData(ROLL), &PID[LEVELGYROROLL]));
  //motors.setMotorAxisCommand(PITCH, updatePID(pitchAttitudeCmd, -gyro.getData(PITCH), &PID[LEVELGYROPITCH]));
  float roll_altitude_cmd = vF_dspl_pid((int)getAngle(flight_angle,ROLL), PID[LEVELROLL])/100.0;
  float pitch_altitude_cmd = vF_dspl_pid((int)getAngle(flight_angle,PITCH), PID[LEVELPITCH])/100.0;
  setMotorAxisCommand(motors,ROLL,vF_dspl_pid((int)(roll_altitude_cmd - getgyro(flight_angle,ROLL)), PID[LEVELGYROROLL])/100.0);
  setMotorAxisCommand(motors,PITCH,vF_dspl_pid((int)(pitch_altitude_cmd + getAngle(flight_angle,PITCH)), PID[LEVELGYROPITCH])/100.0);
}

void processFlightControl(MOTORS_TYPE* motors, FLIGHT_ANGLE_TYPE* flight_angle, tS_pid_Coeff* PID[10])
{
  int throttle=0, motor;
  
  calculateFlightError(motors,flight_angle,PID);
  
  //do some yaw thing
  
  //altitude

  //Plus mode
  setMotorCommand(motors, FRONT, throttle - getMotorAxisCommand(motors, PITCH) - getMotorAxisCommand(motors, YAW));
  setMotorCommand(motors, RIGHT, throttle - getMotorAxisCommand(motors, ROLL) + getMotorAxisCommand(motors, YAW));
  setMotorCommand(motors, LEFT, throttle + getMotorAxisCommand(motors, ROLL) + getMotorAxisCommand(motors, YAW));
  setMotorCommand(motors, REAR, throttle + getMotorAxisCommand(motors, PITCH) - getMotorAxisCommand(motors, YAW));

  //maxmin
  for (motor = FRONT; motor < LASTMOTOR; motor++) {
  if (getMotorCommand(motors, motor) > getMaxCommand(motors,motor))
    setMotorCommand(motors,motor,getMaxCommand(motors,motor));
  else if (getMotorCommand(motors, motor) > getMinCommand(motors,motor))
    setMotorCommand(motors,motor,getMinCommand(motors,motor));
  }
  
  writeMotors(motors);
}
