/*******************************************************************************
 * @file
 * @purpose
 * @version        0.1
 *------------------------------------------------------------------------------
 * Copyright (C) 2011 Gumstix Inc.
 * All rights reserved.
 *
 * Contributer(s):
 *   Neil MacMunn   <neil@gumstix.com>
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

#include "return.h"
#include "table.h"

#include "LPC17xx.h"
#include "lpc_types.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_pwm.h"
#include "lpc17xx_systick.h"
#include "aerodroid.h"

void SysTickHandler(void);
extern int aeroLoop_on;
extern int safety_counter;
//FunctionalState Cur_State = ENABLE;

/*****************************************************************************

    Generic Interrupt Service Routine

*****************************************************************************/
void IntHandler(void)
{
    unsigned int int_num;
    uint8_t int_str[8];

    /*
     * Get the interrupt number
     */
    __asm("mrs %0, ipsr;" : "=r"(int_num) );
    int_num -= 16;

    /*
     * Disable the interrupt
     */
    NVIC_DisableIRQ(int_num);

    /*
     * Send the interrupt signal and number
     */
    sprintf((char*) int_str, "\r\n%x\r\n", int_num);
    writeUSBOutString(int_str);
}

/*********************************************************************

        SysTick interrupt handler

 ***********************************************************************/
void SysTickHandler(void)
{
	//Clear System Tick counter flag
	SYSTICK_ClearCounterFlag();
  
  safety_counter--;
  
  if (safety_counter < 5) //starts at 10
  {
    aeroLoopOff();
    GPIO_ClearValue(3, (1 << 25));
  }

  if (aeroLoop_on)
    aeroLoop(0);
  else
  {
    stopAllMotors(0);
    aeroLoopOff();
  }
}

/*****************************************************************************

        Hardware Initialization Routine

*****************************************************************************/

int PWMMatchInit(uint8_t MatchChannel,uint32_t MatchValue)
{
    PWM_MATCHCFG_Type PWMMatchCfgDat;

    PWM_MatchUpdate(LPC_PWM1, MatchChannel, MatchValue, PWM_MATCH_UPDATE_NOW);
    PWMMatchCfgDat.IntOnMatch = DISABLE;
    PWMMatchCfgDat.MatchChannel = MatchChannel;
    if (!MatchChannel)
        PWMMatchCfgDat.ResetOnMatch = ENABLE;
    else
        PWMMatchCfgDat.ResetOnMatch = DISABLE;
    PWMMatchCfgDat.StopOnMatch = DISABLE;
    PWM_ConfigMatch(LPC_PWM1, &PWMMatchCfgDat);

    return 0;
}

void PWMInit(void)
// Initialize PWM for ESCs
{
    int i=1;
    _roboveroConfig(NULL);

    PWMMatchInit(0, 20000);
    for (i=2; i<6; i++)
    {
      PWMMatchInit(i, 1000);
      PWM_ChannelCmd(LPC_PWM1, i, ENABLE);
    }

    PWMMatchInit(1, 1000);
    PWM_ChannelCmd(LPC_PWM1, 1, ENABLE);
    PWM_ResetCounter(LPC_PWM1);
    PWM_CounterCmd(LPC_PWM1, ENABLE);
    PWM_Cmd(LPC_PWM1, ENABLE);

    //PWM_MatchUpdate(LPC_PWM1, 1, 1250, PWM_MATCH_UPDATE_NOW);
}
void hwInit(void)
{
    /*
     * make the LED pin an output and turn it on
     */
    GPIO_SetDir(3, (1 << 25), 1);
    GPIO_ClearValue(3, (1 << 25));

    /*
     * start the usb device wait until configuration completes before proceeding
     */
    USB_Init();
    USB_Connect(TRUE);
    while (!USB_Configuration);

}

void heartbeat(void)
{
    unsigned long i;

    for (i = 0; i < 1200000; i++);
    GPIO_ClearValue(3, (1 << 25));
    for (i = 0; i < 800000; i++);
    GPIO_SetValue(3, (1 << 25));
    for (i = 0; i < 3200000; i++);
    GPIO_ClearValue(3, (1 << 25));
    for (i = 0; i < 800000; i++);
    GPIO_SetValue(3, (1 << 25));
}

extern int heartbeat_on;

int main(void)
{
    PWMInit();
    hwInit();

    /*
     * let usbuser/robovero handle the rest
     */
    while (1)
    {
        if (heartbeat_on)
            heartbeat();
    }

    return 0;
}

// CAN TEST
/*PINSEL_CFG_Type PinCfg;
CAN_MSG_Type TXMsg;
PinCfg.Funcnum = 3;
PinCfg.OpenDrain = 0;
PinCfg.Pinmode = 0;
PinCfg.Pinnum = 21;
PinCfg.Portnum = 0;
PINSEL_ConfigPin(&PinCfg);
PinCfg.Pinnum = 22;
PINSEL_ConfigPin(&PinCfg);
CAN_Init(LPC_CAN1, 100000);
CAN_SetAFMode(LPC_CANAF,CAN_AccBP);
TXMsg.format = EXT_ID_FORMAT;
TXMsg.id = 0x00001234;
TXMsg.len = 8;
TXMsg.type = DATA_FRAME;
TXMsg.dataA[0] = TXMsg.dataA[1] = TXMsg.dataA[2] = TXMsg.dataA[3] = 0x01234567;
TXMsg.dataB[0] = TXMsg.dataB[1] = TXMsg.dataB[2] = TXMsg.dataB[3] = 0x89ABCDEF;
while (1) {
    CAN_SendMsg(LPC_CAN1, &TXMsg);
    TXMsg.id ++;
}*/

// MASTER CLOCK TEST
/*//Initialize clockout pin
PINSEL_CFG_Type PinCfg;
PinCfg.Portnum = 1;
PinCfg.Pinnum = 27;
PinCfg.Pinmode = 0;
PinCfg.OpenDrain = 0;
PinCfg.Funcnum = 1;
PINSEL_ConfigPin(&PinCfg);*/

///* Initialize UART1 */
//PINSEL_CFG_Type PinCfg;
//PinCfg.Portnum = 2;
//PinCfg.Pinnum = 0;
//PinCfg.Pinmode = 0;
//PinCfg.OpenDrain = 0;
//PinCfg.Funcnum = 2;
//PINSEL_ConfigPin(&PinCfg);
//PinCfg.Pinnum = 1;
//PINSEL_ConfigPin(&PinCfg);

//    UART_CFG_Type * UARTConfigStruct_ptr;
//    UARTConfigStruct_ptr = _UART_CFG_Type_malloc();
//    if (UARTConfigStruct_ptr == NULL)
//        while (1);
//    UART_ConfigStructInit(UARTConfigStruct_ptr);
//    _UART_CFG_Type_set_Baud_rate(UARTConfigStruct_ptr, 115200);
//    UART_Init(LPC_UART1, UARTConfigStruct_ptr);
//    UART_TxCmd(LPC_UART1, ENABLE);

