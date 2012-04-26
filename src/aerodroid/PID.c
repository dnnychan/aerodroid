#include "PID.h"

float doPID(float target, float current, PID_TYPE* PID)
// PID using floats
{
  float dTerm, dT=0.02;
  float error = target - current;
  
  PID->IntegratedError += error * dT;
  if (PID->IntegratedError > PID->IMax)
    PID->IntegratedError = PID->IMax;
  else if (PID->IntegratedError < -PID->IMax)
    PID->IntegratedError = -PID->IMax;
  
  dTerm = (error - PID->LastError) / dT;
  
  PID->LastError = error;
  
  return PID->Kp * error + PID->Ki * PID->IntegratedError + PID->Kd * dTerm;  
}
