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

#include "lpc17xx_pinsel.h"
#include "lpc17xx_pwm.h"
#include "lpc17xx_systick.h"
#include "cr_dsplib.h"
#include "aerodroid.h"

int aeroLoop_on = FALSE;
int armed = 0;
float angle_limit = .175;   // 10 degrees

void aeroPIDValuesInit(PID_TYPE* PID[10])
{
  // from 3DRquad.param
  PID[ROLL]->Kp = 100.00;
  PID[ROLL]->Ki = 00;
  PID[ROLL]->Kd = -300.00;
  PID[ROLL]->IntegratedError = 0;
  PID[ROLL]->LastError = 0;
  PID[ROLL]->IMax = 1000;
  
  PID[PITCH]->Kp = 100.00;
  PID[PITCH]->Ki = 00;
  PID[PITCH]->Kd = -300.00;
  PID[PITCH]->IntegratedError = 0;
  PID[PITCH]->LastError = 0;
  PID[PITCH]->IMax = 1000;
  
  PID[YAW]->Kp = 200.00;
  PID[YAW]->Ki = 5.00;
  PID[YAW]->Kd = 00;
  PID[YAW]->IntegratedError = 0;
  PID[YAW]->LastError = 0;
  PID[YAW]->IMax = 1000;
  
  PID[LEVELROLL]->Kp = 3.5;
  PID[LEVELROLL]->Ki = 0;
  PID[LEVELROLL]->Kd = 00;
  PID[LEVELROLL]->IntegratedError = 0;
  PID[LEVELROLL]->LastError = 0;
  PID[LEVELROLL]->IMax = 4000;
  
  PID[LEVELPITCH]->Kp = 3.5;
  PID[LEVELPITCH]->Ki = 0;
  PID[LEVELPITCH]->Kd = 00;
  PID[LEVELPITCH]->IntegratedError = 0;
  PID[LEVELPITCH]->LastError = 0;
  PID[LEVELPITCH]->IMax = 4000;
  
  PID[HEADING]->Kp = 3.00;
  PID[HEADING]->Ki = 0.10;
  PID[HEADING]->Kd = 00;
  PID[HEADING]->IntegratedError = 0;
  PID[HEADING]->LastError = 0;
  PID[HEADING]->IMax = 1000;
  
  PID[LEVELGYROROLL]->Kp = .149;
  PID[LEVELGYROROLL]->Ki = 0.039;
  PID[LEVELGYROROLL]->Kd = -.01;
  PID[LEVELGYROROLL]->IntegratedError = 0;
  PID[LEVELGYROROLL]->LastError = 0;
  PID[LEVELGYROROLL]->IMax = 250;
  
  PID[LEVELGYROPITCH]->Kp = .149;
  PID[LEVELGYROPITCH]->Ki = 0.039;
  PID[LEVELGYROPITCH]->Kd = -.01;
  PID[LEVELGYROPITCH]->IntegratedError = 0;
  PID[LEVELGYROPITCH]->LastError = 0;
  PID[LEVELGYROPITCH]->IMax = 250;
}

int _setLevelRollPID (uint8_t * args)
{
  uint8_t * arg_ptr;
  int P, I, D;

	if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	P = (int) strtoul((char *) arg_ptr, NULL, 16);
	if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	I = (int) strtoul((char *) arg_ptr, NULL, 16);
	if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	D = (int) strtoul((char *) arg_ptr, NULL, 16);
  
  PID[LEVELROLL]->Kp = P/1000.0;
  PID[LEVELROLL]->Ki = I/1000.0;
  PID[LEVELROLL]->Kd = D/1000.0;
  PID[LEVELROLL]->IntegratedError = 0;
  PID[LEVELROLL]->LastError = 0;
  
  return 0;
}

int _setLevelPitchPID (uint8_t * args)
{
  uint8_t * arg_ptr;
  int P, I, D;

	if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	P = (int) strtoul((char *) arg_ptr, NULL, 16);
	if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	I = (int) strtoul((char *) arg_ptr, NULL, 16);
	if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	D = (int) strtoul((char *) arg_ptr, NULL, 16);
  
  PID[LEVELPITCH]->Kp = P/1000.0;
  PID[LEVELPITCH]->Ki = I/1000.0;
  PID[LEVELPITCH]->Kd = D/1000.0;
  PID[LEVELPITCH]->IntegratedError = 0;
  PID[LEVELPITCH]->LastError = 0;
  
  return 0;
}

int _setLevelGyroRollPID (uint8_t * args)
{
  uint8_t * arg_ptr;
  int P, I, D;

	if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	P = (int) strtoul((char *) arg_ptr, NULL, 16);
	if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	I = (int) strtoul((char *) arg_ptr, NULL, 16);
	if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	D = (int) strtoul((char *) arg_ptr, NULL, 16);
  
  PID[LEVELGYROROLL]->Kp = P/1000.0;
  PID[LEVELGYROROLL]->Ki = I/1000.0;
  PID[LEVELGYROROLL]->Kd = D/1000.0;
  PID[LEVELGYROROLL]->IntegratedError = 0;
  PID[LEVELGYROROLL]->LastError = 0;
  
  return 0;
}

int _setLevelGyroPitchPID (uint8_t * args)
{
  uint8_t * arg_ptr;
  int P, I, D;

	if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	P = (int) strtoul((char *) arg_ptr, NULL, 16);
	if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	I = (int) strtoul((char *) arg_ptr, NULL, 16);
	if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	D = (int) strtoul((char *) arg_ptr, NULL, 16);
  
  PID[LEVELGYROPITCH]->Kp = P/1000.0;
  PID[LEVELGYROPITCH]->Ki = I/1000.0;
  PID[LEVELGYROPITCH]->Kd = D/1000.0;
  PID[LEVELGYROPITCH]->IntegratedError = 0;
  PID[LEVELGYROPITCH]->LastError = 0;
  
  return 0;
}

int twosComplement(uint8_t low_byte, uint8_t high_byte)
{
  return ((int)((low_byte + (high_byte << 8)) + pow(2,15)) % (int)pow(2,16) - pow(2,15));
}

void writeReg(I2C_M_SETUP_Type* device,uint8_t reg, uint32_t value)
{
  device->tx_data[0] = reg;
  device->tx_data[1] = value;
  device->tx_length = 2;
  device->rx_data = 0;
  device->rx_length = 0;
  ret = I2C_MasterTransferData(LPC_I2C0, device, I2C_TRANSFER_POLLING);
}

uint8_t* read6Reg(I2C_M_SETUP_Type* device,uint8_t reg, uint8_t* rx_data6)
{
  device->tx_data[0] = reg | 0b10000000; //MSB must be equal to 1 to read multiple bytes
  device->tx_length = 1;
  device->rx_data = rx_data6;
  device->rx_length = 6;
  ret = I2C_MasterTransferData(LPC_I2C0, device, I2C_TRANSFER_POLLING);
  return rx_data6;
}

int aeroInit(uint8_t * args)
{
  int i;

  uint8_t accel_ctrl_reg1 = 0x20;
  uint8_t accel_ctrl_reg4 = 0x23;

  uint8_t gyro_ctrl_reg1 = 0x20;
  //uint8_t gyro_ctrl_reg2 = 0x21;
  uint8_t gyro_ctrl_reg3 = 0x22;
  uint8_t gyro_ctrl_reg4 = 0x23;
  //uint8_t gyro_ctrl_reg5 = 0x24;
  //uint8_t gyro_status_reg = 0x27;
  
  accelerometer = (I2C_M_SETUP_Type*) malloc(sizeof(I2C_M_SETUP_Type));
  gyro = (I2C_M_SETUP_Type*) malloc(sizeof(I2C_M_SETUP_Type));

  uint8_t* accel_tx_data = (uint8_t*) malloc(2*sizeof(uint8_t));
  uint8_t* gyro_tx_data  = (uint8_t*) malloc(2*sizeof(uint8_t));

  accelerometer->sl_addr7bit=0x18;
  accelerometer->tx_data=accel_tx_data;
  accelerometer->retransmissions_max=3;

  writeReg(accelerometer,accel_ctrl_reg1,0x2F);
  writeReg(accelerometer,accel_ctrl_reg4, 0x00);

  gyro->sl_addr7bit=0x68;
  gyro->tx_data=gyro_tx_data;
  gyro->retransmissions_max=3;

  writeReg(gyro,gyro_ctrl_reg3, 0x08);
  writeReg(gyro,gyro_ctrl_reg4, 0x80);
  writeReg(gyro,gyro_ctrl_reg1, 0x0F);
  
  accel_raw = (uint8_t*) malloc(6*sizeof(uint8_t));
  gyro_raw  = (uint8_t*) malloc(6*sizeof(uint8_t));
  
  for (i=0; i<10; i++)
    PID[i] = (PID_TYPE*) malloc(sizeof(PID_TYPE));

  aeroPIDValuesInit(PID);
  
  flight_angle = flightAngleInitialize(1.0, 0.0);
  motors = motorsInit();
  writeMotors(motors);

  return 0;
}

int stopAllMotors(uint8_t * args)
{
  setMotorCommand(motors,FRONT,MINCOMMAND);
  setMotorCommand(motors,RIGHT,MINCOMMAND);
  setMotorCommand(motors,REAR,MINCOMMAND);
  setMotorCommand(motors,LEFT,MINCOMMAND);
  
  armed = 0;
  
  writeMotors(motors);
  
  return 0;
}

void aeroLoopOff(void)
{
  int i;
  
  if (aeroLoop_on == TRUE)
  {
    SYSTICK_IntCmd(DISABLE);
    SYSTICK_Cmd(DISABLE);
    
    aeroLoop_on = FALSE;
    stopAllMotors(0);
    
    free(accelerometer->tx_data);
    free(gyro->tx_data);
    
    free(accelerometer);
    free(gyro);
    free(accel_raw);
    free(gyro_raw);
    free(flight_angle);
    free(motors);
    
    for (i=0; i<10; i++)
      free(PID[i]);
  }
}

int _aeroLoopOff(uint8_t * args)
{
  aeroLoopOff();
  
  //~ sprintf((char *) str, "%x\r\n",(int)(1));
  //~ writeUSBOutString(str);

  return 0;
}

int _armMotors(uint8_t * args)
{
  armed = 1;
  
  return 0;
}

int _aeroLoopOn(uint8_t * args)
{
  aeroLoop_on = TRUE;
    
  //Initialize System Tick with 20ms time interval
  SYSTICK_InternalInit(20);
  //Enable System Tick interrupt
  SYSTICK_IntCmd(ENABLE);
  //Enable System Tick Counter
  SYSTICK_Cmd(ENABLE);
  
  safety_counter=10;
  
  return 0;
}

int _maintainConnection(uint8_t * args)
{
  safety_counter=10;
  
  return 0;
}

int _getMotorCommands(uint8_t * args)
{
  sprintf((char *) str, "%x %x %x %x\r\n",(int)(getMotorCommand(motors,FRONT)), (int)(getMotorCommand(motors,RIGHT)), (int)(getMotorCommand(motors,REAR)), (int)(getMotorCommand(motors,LEFT)));
  writeUSBOutString(str);
  
  return 0;
}

int _getFlightAngles(uint8_t * args)
{
  sprintf((char *) str, "%x %x %x\r\n",(int)(flight_angle->angle[ROLL]*180/3.14159), (int)(flight_angle->angle[PITCH]*180/3.14159), (int)(flight_angle->angle[YAW]*180/3.14159));
  writeUSBOutString(str);
  
  return 0;
}

int _getGyroReadings(uint8_t * args)
{
  sprintf((char *) str, "%x %x %x\r\n",(int)(gyro_data.y*180/3.14159), (int)(gyro_data.x*180/3.14159), (int)(gyro_data.z*180/3.14159));
  writeUSBOutString(str);
  
  return 0;
}

int _getAccelReadings(uint8_t * args)
{
  sprintf((char *) str, "%x %x %x\r\n",(int)(accel_data.x*100), (int)(accel_data.y*100), (int)(accel_data.z*100));
  writeUSBOutString(str);
  
  return 0;
}

int _setAngleLimit(uint8_t * args)
{
  uint8_t * arg_ptr;
  int temp;
	
  if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	temp = (int) strtoul((char *) arg_ptr, NULL, 16);
  
  angle_limit = temp * 3.14159 / 180.0;
  
  return 0;
}

void aeroLoop(uint8_t * args)
{  
  accel_raw=read6Reg(accelerometer, ACCEL_X_LOW, accel_raw);
  gyro_raw=read6Reg(gyro, GYRO_X_LOW, gyro_raw);

  accel_data.x=twosComplement(accel_raw[0], accel_raw[1])/1671.8;
  accel_data.y=twosComplement(accel_raw[2], accel_raw[3])/1671.8;
  accel_data.z=twosComplement(accel_raw[4], accel_raw[5])/1671.8;

  gyro_data.x=twosComplement(gyro_raw[0], gyro_raw[1])/3754.956;  //radians //131.072; //degrees
  gyro_data.y=twosComplement(gyro_raw[2], gyro_raw[3])/3754.956;  //radians //131.072; //degrees
  gyro_data.z=twosComplement(gyro_raw[4], gyro_raw[5])/3754.956;  //radians //131.072; //degrees

  //flightAngleCalculate(flight_angle, gyro_data.x, gyro_data.y, gyro_data.z, accel_data.x, accel_data.y, accel_data.z, ACCEL_ONEG, 1, 0);
  flightAngleCalculate(flight_angle, gyro_data.y, -gyro_data.x, gyro_data.z, accel_data.x, accel_data.y, accel_data.z, ACCEL_ONEG, 1, 0);

  processFlightControl(motors, flight_angle, PID, gyro_data);
  
  if ((flight_angle->angle[ROLL] > angle_limit) || (flight_angle->angle[ROLL] < -angle_limit) || (flight_angle->angle[PITCH] > angle_limit) || (flight_angle->angle[PITCH] < -angle_limit))
  {
    aeroLoopOff();
  }
  else
    writeMotors(motors);
}
