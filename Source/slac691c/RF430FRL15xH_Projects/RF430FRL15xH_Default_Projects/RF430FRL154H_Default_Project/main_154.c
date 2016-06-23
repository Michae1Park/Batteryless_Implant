/*
 * main.c
 *
 * RF430FRL152H Default Example Project
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/


#include <rf430frl152h.h>
#include "types.h"

//*****************************Summary********************************/

/*This project is meant to simply initialize the FRAM of the device so that
 * it will be operational.  It initializes the FRAM ISR to the correct value
 * to point to the ROM code, which runs the device.
 *
 */
//*****************************FUNCTION PROTOTYPES********************************/



//*****************************DEFINES *******************************************/
#define ROM_EUSCI_SUPPORT_ENABLED       BIT2
#define EROM_EUSCI_SUPPORT_DISABLED     0
#define ROM_SENSOR_SUPPORT_ENABLED      BIT7
#define ROM_SENSOR_SUPPORT_DISABLED     0
#define	NFC_BRIDGE_DISABLED 			BIT6
#define	NFC_BRIDGE_ENABLED  			0
#define	EIGHT_BYTE_BLOCK    			BIT0
#define	FOUR_BYTE_BLOCK     			0
#define	FIRST_ISO_PAGE      			BIT1
#define	SECOND_ISO_PAGE     			0

/*
 *     Bit 0: 	ISOBlockSize				0 - 4 byte,		1 - 8 byte
 *     Bit 1:	Page						0 - page 1, 	1 - page 0 (Effective only for 4-byte block mode)
 *     Bit 2: 	ROMEUSCISupportEnabled		0 - disabled, 	1 - enabled (Forced to 0 on RF430FRL153H)
 *     Bit 3-5: ReservedISO
 *     Bit 6: 	NFCBridgeDisable  			0 - enabled, 	1 - disabled (see note below)
 *     Bit 7:   ROMSensorSupportEnable		0 - disabled, 	1 - enabled (Forced to 0 on RF430FRL154H)
 *
 */

#define FIRMWARE_CONTROL_ADDRESS 	0xF867
#pragma RETAIN(Firmware_System_Control_Byte);
#pragma location = FIRMWARE_CONTROL_ADDRESS
const u08_t Firmware_System_Control_Byte = ROM_SENSOR_SUPPORT_DISABLED + ROM_EUSCI_SUPPORT_ENABLED + NFC_BRIDGE_DISABLED + EIGHT_BYTE_BLOCK + FIRST_ISO_PAGE; //0x7F,		// this value sets the firmware system control register


extern __interrupt void Reset_ISR(void);

//#pragma CODE_SECTION(RFPMM_ISR, ".fram_driver_code")   //comment this line for using ROM's RFPMM ISR, uncomment next one
#pragma CODE_SECTION(RFPMM_ISR, ".rfpmm_rom_isr") //comment this line for creating a custom RFPMM ISR that will exist in FRAM, bypassing ROM's, uncomment previous
#pragma vector = RFPMM_VECTOR
__interrupt void RFPMM_ISR(void)
{
}

//#pragma CODE_SECTION(PORT1_ISR, ".fram_driver_code")   //comment this line for using ROM's PORT1 ISR, uncomment next one
#pragma CODE_SECTION(PORT1_ISR, ".port1_rom_isr") //comment this line for creating a custom PORT1 ISR that will exist in FRAM, bypassing ROM's, uncomment previous
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
}


//#pragma CODE_SECTION(USCI_B0_ISR, ".fram_driver_code")   //comment this line for using ROM's USCI_B0 ISR, uncomment next one
#pragma CODE_SECTION(USCI_B0_ISR, ".usci_b0_rom_isr") //comment this line for creating a custom USCI_B0 ISR that will exist in FRAM, bypassing ROM's, uncomment previous
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
}

//#pragma CODE_SECTION(RF13M_ISR, ".fram_driver_code")   //comment this line for using ROM's RF13M ISR, uncomment next one
#pragma CODE_SECTION(RF13M_ISR, ".rf13m_rom_isr") //comment this line for creating a custom RF13M ISR that will exist in FRAM, bypassing ROM's, uncomment previous
#pragma vector = RF13M_VECTOR
__interrupt void RF13M_ISR(void)
{
}

//#pragma CODE_SECTION(WDT_ISR, ".fram_driver_code")   //comment this line for using ROM's WDT ISR, uncomment next one
#pragma CODE_SECTION(WDT_ISR, ".wdt_rom_isr") //comment this line for creating a custom WDT ISR that will exist in FRAM, bypassing ROM's, uncomment previous
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void)
{
}

//#pragma CODE_SECTION(TimerA1_ISR, ".fram_driver_code")   //comment this line for using ROM's Timer_A1 ISR, uncomment next one
#pragma CODE_SECTION(TimerA1_ISR, ".timer_a1_rom_isr") //comment this line for creating a custom WDT ISR TimerA1 that will exist in FRAM, bypassing ROM's, uncomment previous
#pragma vector = TIMER0_A1_VECTOR
__interrupt void TimerA1_ISR(void)
{
}

//#pragma CODE_SECTION(TimerA0_ISR, ".fram_driver_code")   //comment this line for using ROM's Timer_A0 ISR, uncomment next one
#pragma CODE_SECTION(TimerA0_ISR, ".timer_a0_rom_isr") //comment this line for creating a custom WDT ISR Timer_A0 that will exist in FRAM, bypassing ROM's, uncomment previous
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TimerA0_ISR(void)
{
}

//#pragma CODE_SECTION(UNMI_ISR, ".fram_driver_code")   //comment this line for using ROM's UNMI ISR, uncomment next one
#pragma CODE_SECTION(UNMI_ISR, ".unmi_rom_isr") //comment this line for creating a custom WDT UNMI ISR that will exist in FRAM, bypassing ROM's, uncomment previous
#pragma vector = UNMI_VECTOR
__interrupt void UNMI_ISR(void)
{
}

//#pragma CODE_SECTION(SysNMI_ISR, ".fram_driver_code")   //comment this line for using ROM's SYSNMI ISR, uncomment next one
#pragma CODE_SECTION(SysNMI_ISR, ".sysnmi_rom_isr") //comment this line for creating a custom WDT SYSNMI ISR that will exist in FRAM, bypassing ROM's, uncomment previous
#pragma vector = SYSNMI_VECTOR
__interrupt void SysNMI_ISR(void)
{
}

#pragma RETAIN (Reset_ISR);
//#pragma CODE_SECTION(Reset_ISR, ".fram_driver_code")   //comment this line for using ROM's RESET ISR, uncomment next one
#pragma CODE_SECTION(Reset_ISR, ".reset_rom_isr") //comment this line for creating a custom WDT RESET ISR that will exist in FRAM, bypassing ROM's, uncomment previous
#pragma vector = RESET_VECTOR
extern __interrupt void Reset_ISR(void)
{
}

