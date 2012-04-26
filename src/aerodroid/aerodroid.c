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

#include "lpc17xx_pinsel.h"
#include "lpc17xx_pwm.h"
#include "lpc17xx_systick.h"
#include "cr_dsplib.h"
#include "aerodroid.h"

#define LOW_PASS_SIZE 10

int aeroLoop_on = FALSE;
float angle_limit = .175;   // 10 degrees
uint32_t altitude_data;
int altitude_control;

float ax = 0;
float ay = 0;
float az = -ACCEL_ONEG;

float low_pass_u = 0.1;

float ax_data[LOW_PASS_SIZE];
float ay_data[LOW_PASS_SIZE];
float az_data[LOW_PASS_SIZE];

int ax_pos=0;
int ay_pos=0;
int az_pos=0;

/**********************************************************************
 * 
 *                      Settings Functions
 * 
 * *******************************************************************/

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

int _setAltitudePID (uint8_t * args)
{
  uint8_t * arg_ptr;
  int P, I, D;

	if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	P = (int) strtoul((char *) arg_ptr, NULL, 16);
	if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	I = (int) strtoul((char *) arg_ptr, NULL, 16);
	if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	D = (int) strtoul((char *) arg_ptr, NULL, 16);
  
  PID[ALTITUDE]->Kp = P/1000.0;
  PID[ALTITUDE]->Ki = I/1000.0;
  PID[ALTITUDE]->Kd = D/1000.0;
  PID[ALTITUDE]->IntegratedError = 0;
  PID[ALTITUDE]->LastError = 0;
  
  return 0;
}

int _toggleAltitudeControl (uint8_t * args)
{
  uint8_t * arg_ptr;
  int toggle;

	if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	toggle = (int) strtoul((char *) arg_ptr, NULL, 16);
  
  if (toggle)
    altitude_control = TRUE;
  else
    altitude_control = FALSE;
  
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

int _setLowPassU(uint8_t * args)
{
  uint8_t * arg_ptr;
	
  if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	low_pass_u = (float) strtoul((char *) arg_ptr, NULL, 16)/1000.0;
  
  return 0;
}

/**********************************************************************
 * 
 *                        Printing Functions
 * 
 * *******************************************************************/
 
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

int _getAltitudeReadings(uint8_t * args)
{
  sprintf((char *) str, "%x \r\n",(int) altitude_data);
  writeUSBOutString(str);
  
  return 0;
}
 
/**********************************************************************
 * 
 *                        I2C Functions
 * 
 * *******************************************************************/
 
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

/**********************************************************************
 * 
 *                      Initialization Routine
 * 
 * *******************************************************************/
 
int aeroInit(uint8_t * args)
{
  int i;
  
  altitude_control = TRUE;

  uint8_t accel_ctrl_reg1 = 0x20;
  uint8_t accel_ctrl_reg4 = 0x23;

  uint8_t gyro_ctrl_reg1 = 0x20;
  //uint8_t gyro_ctrl_reg2 = 0x21;
  uint8_t gyro_ctrl_reg3 = 0x22;
  uint8_t gyro_ctrl_reg4 = 0x23;
  //uint8_t gyro_ctrl_reg5 = 0x24;
  //uint8_t gyro_status_reg = 0x27;
  
  // initialize acclerometer and gyro
  accelerometer = (I2C_M_SETUP_Type*) malloc(sizeof(I2C_M_SETUP_Type));
  gyro = (I2C_M_SETUP_Type*) malloc(sizeof(I2C_M_SETUP_Type));

  uint8_t* accel_tx_data = (uint8_t*) malloc(2*sizeof(uint8_t));
  uint8_t* gyro_tx_data  = (uint8_t*) malloc(2*sizeof(uint8_t));

  accelerometer->sl_addr7bit=0x18;
  accelerometer->tx_data=accel_tx_data;
  accelerometer->retransmissions_max=3;

  writeReg(accelerometer,accel_ctrl_reg1,0x2F);
  writeReg(accelerometer,accel_ctrl_reg4, 0x30); // +/- 8g
  
  for (i=0; i<LOW_PASS_SIZE; i++)
  {
    ax_data[i] = 0;
    ay_data[i] = 0;
    az_data[i] = ACCEL_ONEG;
  }

  gyro->sl_addr7bit=0x68;
  gyro->tx_data=gyro_tx_data;
  gyro->retransmissions_max=3;

  writeReg(gyro,gyro_ctrl_reg3, 0x08);
  writeReg(gyro,gyro_ctrl_reg4, 0x80);
  writeReg(gyro,gyro_ctrl_reg1, 0x0F);
  
  accel_raw = (uint8_t*) malloc(6*sizeof(uint8_t));
  gyro_raw  = (uint8_t*) malloc(6*sizeof(uint8_t));
  
  // Initialize PID values
  for (i=0; i<10; i++)
    PID[i] = (PID_TYPE*) malloc(sizeof(PID_TYPE));

  aeroPIDValuesInit(PID);
  
  // Set GPIO for rangefinder
  GPIO_SetDir(0, (1 << 4), 1);
  GPIO_SetDir(0, (1 << 5), 0);
  
  // Initialize flight_angle
  flight_angle = flightAngleInitialize(1.0, 0.0);
  
  // Initialize motors. Sets values to default
  motors = motorsInit();
  writeMotors(motors);

  return 0;
}

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
  
  PID[ALTITUDE]->Kp = .4;
  PID[ALTITUDE]->Ki = 0.02;
  PID[ALTITUDE]->Kd = 0;
  PID[ALTITUDE]->IntegratedError = 0;
  PID[ALTITUDE]->LastError = 0;
  PID[ALTITUDE]->IMax = 250;
}

/**********************************************************************
 * 
 *                      Start/Stop Functions
 * 
 * *******************************************************************/
 
int stopAllMotors(uint8_t * args)
{
  setMotorCommand(motors,FRONT,MINCOMMAND);
  setMotorCommand(motors,RIGHT,MINCOMMAND);
  setMotorCommand(motors,REAR,MINCOMMAND);
  setMotorCommand(motors,LEFT,MINCOMMAND);
  
  writeMotors(motors);
  
  return 0;
}

void aeroLoopOff(void)
{
  // Shuts down motors, turns off interrupt. Frees all variables
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
  //Resets count down to 10. Shuts off at 5
  safety_counter=10;
  
  return 0;
}

/**********************************************************************
 * 
 *                       Filtering Functions
 * 
 * *******************************************************************/
 
float lowPass(float *data, int size, float new_data, int* pos)
// Low pass filter. The average of data (which is size long). New data
// is added to pos
{
  float sum = 0;
  int c = 0;
  
  data[*pos] = new_data;
  *pos = ((*pos) + 1 )% size;
  
  for (c = 0; c < size; c++)
    sum +=data[c];
  
  return sum/size;
}

/**********************************************************************
 * 
 *                            MAIN LOOP
 * 
 * *******************************************************************/
 
 // Called from SysTickHandler every 20ms
void aeroLoop(uint8_t * args)
{  
  // Read sensors and convert appropriately
  accel_raw=read6Reg(accelerometer, ACCEL_X_LOW, accel_raw);
  gyro_raw=read6Reg(gyro, GYRO_X_LOW, gyro_raw);
  
  //417 for 8g, 1671 for 2g
  accel_data.x=twosComplement(accel_raw[0], accel_raw[1])/417.95; // /1671.8;
  accel_data.y=twosComplement(accel_raw[2], accel_raw[3])/417.95; // /1671.8;
  accel_data.z=twosComplement(accel_raw[4], accel_raw[5])/417.95; // /1671.8;

  //low pass the accelerometer readings
  /*ax = ((1.0-low_pass_u)*accel_data.x) + (low_pass_u*ax);
  ay = ((1.0-low_pass_u)*accel_data.y) + (low_pass_u*ay);
  az = ((1.0-low_pass_u)*accel_data.z) + (low_pass_u*az);*/
  
  ax = lowPass(&ax_data, LOW_PASS_SIZE, accel_data.x, &ax_pos);
  ay = lowPass(&ay_data, LOW_PASS_SIZE, accel_data.y, &ay_pos);
  az = lowPass(&az_data, LOW_PASS_SIZE, accel_data.z, &az_pos);

  gyro_data.x=(twosComplement(gyro_raw[0], gyro_raw[1])+36)/3754.956;  //radians //131.072; //degrees
  gyro_data.y=(twosComplement(gyro_raw[2], gyro_raw[3])-3)/3754.956;  //radians //131.072; //degrees
  gyro_data.z=(twosComplement(gyro_raw[4], gyro_raw[5])+85)/3754.956;  //radians //131.072; //degrees


  // Calculate the flight angles using sensor readings
  
  //flightAngleCalculate(flight_angle, gyro_data.x, gyro_data.y, gyro_data.z, accel_data.x, accel_data.y, accel_data.z, ACCEL_ONEG, 1, 0);
 // flightAngleCalculate(flight_angle, gyro_data.y, -gyro_data.x, gyro_data.z, accel_data.x, accel_data.y, accel_data.z, ACCEL_ONEG, 1, 0);
  flightAngleCalculate(flight_angle, gyro_data.y, -gyro_data.x, gyro_data.z, ax, ay, az, ACCEL_ONEG, 1, 0);
    
  if (altitude_control)
    altitude_data = pulseIn(0, (1 << 5), 0, (1 << 4), 9850)/58;
  
  // Use flight angles to update motor output
  processFlightControl(motors, flight_angle, PID, gyro_data, altitude_data, altitude_control);
  
  // Check to make sure angles are within limits (ie not tilted too much)
  if ((flight_angle->angle[ROLL] > angle_limit) || (flight_angle->angle[ROLL] < -angle_limit) || (flight_angle->angle[PITCH] > angle_limit) || (flight_angle->angle[PITCH] < -angle_limit))
  {
    // Turn everything off
    aeroLoopOff();
    
    // Turn on LED to signify error
    GPIO_ClearValue(3, (1 << 25));
  }
  else
  // If angles are ok, update motors
    writeMotors(motors);
}
