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

#include "table.h"
#include "return.h"

#include "lpc17xx_gpio.h"

int pulseIn (uint8_t echo_port, uint32_t echo_pin, uint8_t trig_port, uint32_t trig_pin, uint32_t trig_pulse)
// Sends pulse of length trig_pulse at echo. Reads length of pulse at trig
// Used for HC-SR04 rangefinder
{
  uint32_t width = 0;
  uint32_t num_loops = 0;
  uint32_t max_loops = 50000;
  
  GPIO_ClearValue(trig_port, trig_pin);
  delay(trig_pulse);
  GPIO_SetValue(trig_port, trig_pin);
  delay(trig_pulse);
  GPIO_ClearValue(trig_port, trig_pin);
  
  // Wait for previous pulse to end
  while (GPIO_ReadValue(echo_port) & echo_pin) {
    num_loops++;
    if (num_loops > max_loops)
      break;
  }
  
  // Wait for pulse to start
  while ((GPIO_ReadValue(echo_port) & echo_pin) == 0) {
    num_loops++;
    if (num_loops > max_loops)
      break;
  }
    
  // Read length of pulse
  while (GPIO_ReadValue(echo_port) & echo_pin) {
    num_loops++;
    width++;
    if (num_loops > max_loops)
      break;
  }
  if (num_loops > max_loops)
    return 0;
  else
    return width;  
}

int _pulseIn (uint8_t * args)
// Wrapper function for above function
{
  uint8_t * arg_ptr;
	uint8_t echo_port;
  uint32_t echo_pin;
  uint8_t trig_port;
  uint32_t trig_pin;
  uint32_t trig_pulse;

	if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
	echo_port = (uint8_t) strtoul((char *) arg_ptr, NULL, 16);
  if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
  echo_pin = (uint32_t) strtoul((char *) arg_ptr, NULL, 16);
  if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
  trig_port = (uint8_t) strtoul((char *) arg_ptr, NULL, 16);
  if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
  trig_pin = (uint32_t) strtoul((char *) arg_ptr, NULL, 16);
  if ((arg_ptr = (uint8_t *) strtok(NULL, " ")) == NULL) return 1;
  trig_pulse = (uint32_t) strtoul((char *) arg_ptr, NULL, 16);
  
  sprintf((char *) str, "%x\r\n", (unsigned int) pulseIn (echo_port, echo_pin, trig_port, trig_pin, trig_pulse));
	writeUSBOutString(str);
	return 0;
}
