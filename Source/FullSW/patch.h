/*
 * Patch.h
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
#ifndef PATCH_H
#define PATCH_H

#include "types.h"


void userCustomCommand();

//****Patch and Errata functions********************************************************************/
u16_t BlockLockROM_Patched(u16_t blockNumber , u08_t checkLock);
extern void ErrataFix();

typedef void(*DriverFunction)(void);

#define CLEAR_BLOCK_LOCKS                            	BIT3
#define FRAM_LOCK_BLOCK_AREA_SIZE  						38
#define FRAM_LOCK_BLOCKS								0xF840  //Address of ISO15693 lock blocks

#define CRC_LENGTH_IN_BUFFER          2  // the CRC bytes take 2 bytes in the packet
#define DATA_IN_LENGTH				  1  // only 1 byte of data in expected


//Firmware System Control Register Settings
#define ROM_EUSCI_SUPPORT_ENABLED       BIT2
#define ROM_EUSCI_SUPPORT_DISABLED      0
#define ROM_SENSOR_SUPPORT_ENABLED      BIT7
#define ROM_SENSOR_SUPPORT_DISABLED     0
#define	NFC_BRIDGE_DISABLED 			BIT6
#define	NFC_BRIDGE_ENABLED  			0
#define	EIGHT_BYTE_BLOCK    			BIT0
#define FOUR_BYTE_BLOCK_MASK			BIT0
#define	FIRST_ISO_PAGE_MASK    			BIT1
#define	FOUR_BYTE_BLOCK     			0
#define	FIRST_ISO_PAGE      			BIT1
#define	SECOND_ISO_PAGE     			0
#define FRAM_BLOCKS_8					0xF3

#define CHECK_LOCK              	1
#define LOCK_BLOCK              	0
#define LOCKED_FLAG                 BIT0


//------------------------------------------------------------------------------
// Driver section
//------------------------------------------------------------------------------
#define DRIVER_TABLE_START 				0xFFCE               	// starting address for driver table
#define DRIVER_TABLE_KEY  				0xCECE               	// identifier indicating start and end of driver table
#define USER_CUSTOM_COMMAND_ID       	0x00AA               	// user custom command, range from A0 - D0

/********** Errata Fix Add**************/
#define FIX_ROM_ERRATA  				0x1600     				// the initialize digital sensor driver indentifier
#define BLOCK_LOCK_ID		       		0x2600               	// Block Lock Code for ROM

//------------------------------------------------------------------------------
#define CUSTOM_COMMAND         (DRIVER_TABLE_START-2)
#define CUSTOM_COMMAND_ADDR    (DRIVER_TABLE_START-4)

#define BLOCK_LOCK_ROM_COMMAND (DRIVER_TABLE_START-6)
#define BLOCK_LOCK_ROM_ADDR    (DRIVER_TABLE_START-8)

#define ERRATA_FIX_COMMAND	   (DRIVER_TABLE_START-10)
#define ERRATA_FIX_ADDR        (DRIVER_TABLE_START-12)

#define NUMBER_OF_DRIVER_FUNCTIONS 		3                     	// the amount of patched functions

#define DRIVER_TABLE_END  (DRIVER_TABLE_START-2-(NUMBER_OF_DRIVER_FUNCTIONS*4))

#endif
