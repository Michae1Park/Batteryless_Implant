/*
 * Patch.c
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
#include "patch.h"
#include "types.h"

/* Firmware System Control Byte
 *
 *     Bit 0: 	ISOBlockSize				0 - 4 byte,		1 - 8 byte
 *     Bit 1:	Page						0 - page 1, 	1 - page 0 (Effective only for 4-byte block mode)
 *     Bit 2: 	ROMEUSCISupportEnabled		0 - disabled, 	1 - enabled (Forced to 0 on RF430FRL153H)
 *     Bit 3-5: ReservedISO
 *     Bit 6: 	NFCBridgeDisable  			0 - enabled, 	1 - disabled (see note below)
 *     Bit 7:   ROMSensorSupportEnable		0 - disabled, 	1 - enabled (Forced to 0 on RF430FRL154H)
 *
 *     NFC bridge is recommended to be disabled in this project.  Unexpected behaviour can occur,
 *     trying to use it, due to the configuration being setup here.
 *
 *     If eUSCI host controller portion is needed along with the RF functionality, the default project
 *     must be used.  That is NFC cannot be supported in that application (because the I2C/SPI host controller
 *     control registers are in the same place that the NFC file needs to be).  However the rest of the FRAM
 *     memory can be used for storing and reading using ISO15693.
 */

//This project is based on the RF430FRL152H.  However it will work as well on the RF430FRL154H.
//However ROM_SENSOR_SUPPORT_DISABLED (or ROMSensorSupportEnable see above for both )must be set in the firmware system control register.  This is forced automatically on the RF430FRL154H.
//This setting is needed to disable the ROM which uses block 0... as virtual registers, however this memory is needed for NDEF purposes.
#define FIRMWARE_CONTROL_ADDRESS 	0xF867
#pragma RETAIN(Firmware_System_Control_Byte);
#pragma location = FIRMWARE_CONTROL_ADDRESS
//This variable needs to be kept declared and as "volatile" for the BlockLockROM_Patched function to work properly.  Assignment can be changed however.
volatile const u08_t Firmware_System_Control_Byte = ROM_SENSOR_SUPPORT_ENABLED | ROM_EUSCI_SUPPORT_ENABLED | NFC_BRIDGE_DISABLED | FOUR_BYTE_BLOCK | FIRST_ISO_PAGE; // this value sets the firmware system control register

/*******************************Driver/Patch Table Format*******************************/
/*
 *   Address	Value 			Comment
 *
 *   0xFFCE     0xCECE      	The driver table start key, always same address (0xFFCE)
 *
 *   0xFFCC		0x1B00			The command ID of the digital sensor sampling function
 *	 0xFFCA		Address			The address of the driver sensor sampling function in FRAM
 *
 *   0xFFC8		0x0100			The digital sensor function driver initialization function
 *   0xFFC6		Address			The address of the driver function initialization in FRAM
 *
 *
 *   Optional:
 *   0xFFC4		ID				Another driver/patch function ID
 *   0xFFC2		Address			Address of the function above
 *
 *      *          *			Pairs
 *      *		   *
 *
 *   End optional
 *
 *   0xFFC4		0xCECE			Ending key
 *****************************************************************************************/
/*****************************************************************************************
 *
 *  If start key not present in starting location, table does not exist
 *  If it does, a ROM routine will parse it and setup the calls to be made to the
 *  appropriate address when needed.
 *
******************************************************************************************/

/******************************************************************************************
 *
 *  Also custom command functions must be listed in the beggining of the patch table
 *  before patch functions to be recognized by the patch parser.
 *
*******************************************************************************************/

//Start key
#pragma RETAIN(START_KEY);
#pragma location = DRIVER_TABLE_START
const u16_t START_KEY = DRIVER_TABLE_KEY;

#pragma RETAIN(CustomCommandID);
#pragma location = CUSTOM_COMMAND														// the location of the command ID
const u16_t  CustomCommandID = USER_CUSTOM_COMMAND_ID;              					// the function identifier

#pragma RETAIN(CustomCommandAddress);
#pragma location = CUSTOM_COMMAND_ADDR														// the location of the address
const DriverFunction CustomCommandAddress = (DriverFunction)&userCustomCommand;     	// the location the function is in

/********** Errate Fix Add**************/
#pragma RETAIN(BlockLockROMID);
#pragma location = BLOCK_LOCK_ROM_COMMAND													// the location of the command ID
const u16_t  BlockLockROMID = BLOCK_LOCK_ID;              									// the function identifier

#pragma RETAIN(BlockLockROMAddress);
#pragma location = BLOCK_LOCK_ROM_ADDR														// the location of the address
const DriverFunction BlockLockROMAddress = (DriverFunction)&BlockLockROM_Patched;     		// the location the function is in

/********** Errate Fix Add**************/
#pragma RETAIN(Errata_ID);
#pragma location = ERRATA_FIX_COMMAND
const u16_t  Errata_ID = FIX_ROM_ERRATA;                    // the function identifier

#pragma RETAIN(ErrataFunctionAddress);
#pragma location = ERRATA_FIX_ADDR
const DriverFunction ErrataFunctionAddress = (DriverFunction)&ErrataFix;   // the location the function is in

#pragma RETAIN(END_KEY);
#pragma location = DRIVER_TABLE_END
const u16_t END_KEY = DRIVER_TABLE_KEY;

/**************************************************************************************************************************************************
*  ErrataFix
***************************************************************************************************************************************************
*
* Brief : 	Corrects single sensor repeated sampling error.
*
* Param[in] :   None
*
* Param[out]:   None
*
* Return        None
*****************************************************************************************************************************************************/
#pragma RETAIN (ErrataFix)	//needed to prevent of optimizing out this function
//the line below constrains the placement of the function to a specific section of code
//declared in the .cmd file.  Basically we want the function code to be placed in the end
// of the FRAM memory to not interfere with the virtual registers and log memory before it
#pragma CODE_SECTION (ErrataFix, ".fram_driver_code") //see .cmd file for details - all new firmware must go into fram_driver_code memory section space
void ErrataFix()
{
	asm ( " MOV.B #0x3F, &0x1C01 " );
	asm ( " CALL #0x51F4 " );
}


/**************************************************************************************************************************************************
*  BlockLockROM_Patched
***************************************************************************************************************************************************
*
* Brief : 	The ROM equivalent function did not have proper lock block operation in 4-byte block mode.
* 			This functions addresses the issue.
* 			Note: This function depends on Firmware_System_Control_Byte variable to be kept declared
* 			and as a "volatile".
* 			Do not call this function directly.  Rather use the associated API called: BlockLockAPI(...)
*			This function is called by the ROM RF stack.
*			Due to constraint on how much memory is used two 4-byte blocks are locked for each lock command.
*			if block 0 or block 1 is locked both block 0 and block 1 are locked.  Same for other blocks.
*			Paging is supported.
*
* Param[in] :   blockNumber
*				checkLock   - if 0 will perform a lock.  If 1 will return a status if the block is locked
*
* Param[out]:   None
*
* Return        Valid only when checkLock is set to 1
* 				0 - block not locked
* 				1 - block is locked
**************************************************************************************************************************************************/
#pragma RETAIN(BlockLockROM_Patched);
#pragma CODE_SECTION (BlockLockROM_Patched, ".fram_driver_code") //see .cmd file for details - all new firmware must go into fram_driver_code memory section space
u16_t BlockLockROM_Patched(u16_t blockNumber , u08_t checkLock)
{
	if ((Firmware_System_Control_Byte & FOUR_BYTE_BLOCK_MASK) == FOUR_BYTE_BLOCK && blockNumber < FRAM_BLOCKS_8)
	{
		asm (" push.w R10 ");
		asm (" mov.w R12, R15 ");
		asm (" mov.w R12, R14 ");
		asm (" add.w #0xFA00, R14 ");
		asm (" call #0x5CCC"); 			// shifts block number register R12
		asm (" mov.w R12, R10 ");		// copy result
		if ((Firmware_System_Control_Byte & FIRST_ISO_PAGE_MASK) == FIRST_ISO_PAGE)
		{
			asm (" add.w #0xF840, R10");
		}
		else
		{
			asm (" add.w #0xF850, R10");
		}
		asm (" mov.b #0x1, R14 " ); //correction
		asm (" br #0x544E ");
    }
    else
    {
    	asm (" br #0x542C ");  //Call the ROM function, no fixes necessary
    }

	{
		return 0; 	// will not be reached
	}
}
