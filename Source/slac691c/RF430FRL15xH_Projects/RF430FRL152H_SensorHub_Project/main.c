/*
 * main.c
 *
 * RF430FRL152H Sensor Hub Example Project
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

//*****************************FUNCTION PROTOTYPES********************************/
u08_t DigitalSensorMeasurement ();
void SHT_21_I2C_Master_Measurement(u08_t command, u08_t * rxData);
void DigitalSensorInit();
void ISL29023_I2C_Read(u08_t command, u08_t * rxData);
void ISL29023_I2C_Write(u08_t command, u08_t * rxData);
void NoResponse();
void NoResponse2();

typedef void(*DriverFunction)(void);
//********************************************************************************/



//*****************************DEFINES *******************************************/
#define TEMP_MEASURE_HOLD_MASTER	0xE3        // SHT21 Command for temperature measurement
#define HUMIDITY_HOLD_MEASUREMENT	0xE5        // SHT21 Command for humidity measurement
#define MASTER_SLAVE_SELECT         BIT7

#define RESULT_MAILBOX   		*((u16_t *)0x1D02)   // location digital sensor measurement is to be placed in for the ROM code to use it
#define SENSOR_TYPE_MAILBOX   	*((u16_t *)0x1D04)   // ROM code sets this address to the current digital sensor to be sampled
#define HUMIDITY_SENSOR_ID		BIT1                        // SHT21 humidity sensor ID bit

// I2C HAL
#define PORT_I2C_OUT	P1OUT
#define PORT_I2C_DIR	P1DIR
#define PORT_I2C_SEL0	P1SEL0
#define PORT_I2C_SEL1	P1SEL1
#define SDA	BIT0
#define SCL BIT1

//ROM code sensor definitions
enum Sensors_Types
{
    REFERENCE_ADC1_SENSOR          = 0x01,
    THERMISTOR_ADC2_SENSOR         = 0x02,
    ADC0_SENSOR                    = 0x04,
    INTERNAL_SENSOR                = 0x08,
    DIGITAL_SENSOR1                = 0x10,
    DIGITAL_SENSOR2                = 0x20,
    DIGITAL_SENSOR3                = 0x40
};

// ISL29023 register definitions
enum ISL29023_Sensors_Types
{
    ISL29023_COMMAND_I_REGISTER		= 0x00,
    ISL29023_DATA_LSB_REGISTER		= 0x02
};

//------------------------------------------------------------------------------
// Driver section
//------------------------------------------------------------------------------
#define DRIVER_TABLE_START 				0xFFCE               	// starting address for driver table
#define DRIVER_TABLE_KEY  				0xCECE               	// identifier indicating start and end of driver table
#define DIGITAL_SENSOR_DRIVER_ID       	0x1B00               	// the digital sensor driver identified
#define INIT_DIGITAL_SENSOR_DRIVER_ID  	0x0100               	// the initialize digital sensor driver indentifier

#define NUMBER_OF_DRIVER_FUNCTIONS 		2                     	// we only driver two functions here
//------------------------------------------------------------------------------
#define DRIVER_1_COMMAND (DRIVER_TABLE_START-2)  				// DIGITAL_SENSOR_DRIVER_ID, see below
#define DRIVER_1_ADDR    (DRIVER_TABLE_START-4)  

#define DRIVER_2_COMMAND (DRIVER_1_ADDR-2)                		// INIT_DIGITAL_SENSOR_DRIVER_ID, see below
#define DRIVER_2_ADDR    (DRIVER_1_ADDR-4)

//#define DRIVER_3_COMMAND (DRIVER_2_ADDR-2)                		// INIT_DIGITAL_SENSOR_DRIVER_ID, see below
//#define DRIVER_3_ADDR    (DRIVER_2_ADDR-4)

#define DRIVER_TABLE_END  (DRIVER_TABLE_START-2-(NUMBER_OF_DRIVER_FUNCTIONS*4))
//********************************************************************************/

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
  /* If start key not present in starting location, table does not exist
   *  If it does, a ROM routine will parse it and setup the calls to be made to the
   *  appropriate address when needed.
   */
 /*****************************************************************************************/

//Start key
#pragma RETAIN(START_KEY);
#pragma location = DRIVER_TABLE_START
const u16_t START_KEY = DRIVER_TABLE_KEY;


//First ID, address pair
#pragma RETAIN(InitDigitalSensorID);
#pragma location = DRIVER_1_COMMAND														// the location of the command ID
const u16_t  InitDigitalSensorID = INIT_DIGITAL_SENSOR_DRIVER_ID;              	// the function identifier

#pragma RETAIN(DigitalSensorInitAddress);
#pragma location = DRIVER_1_ADDR														// the location of the address
const DriverFunction DigitalSensorInitAddress = (DriverFunction)&DigitalSensorInit;     // the location the function is in

//Second ID, address pair?
#pragma RETAIN(Digital_Sensor_Driver_ID);
#pragma location = DRIVER_2_COMMAND
const u16_t  Digital_Sensor_Driver_ID = DIGITAL_SENSOR_DRIVER_ID;                    // the function identifier

#pragma RETAIN(DigitalSensorMeasurementFunctionAddress);
#pragma location = DRIVER_2_ADDR
const DriverFunction DigitalSensorMeasurementFunctionAddress = (DriverFunction)&DigitalSensorMeasurement;   // the location the function is in


//Third ID, address pair?  If so, update NUMBER_OF_DRIVER_FUNCTIONS to 3...

//Ending key
#pragma RETAIN(END_KEY);
#pragma location = DRIVER_TABLE_END
const u16_t END_KEY = DRIVER_TABLE_KEY;

extern __interrupt void Reset_ISR(void);

/**************************************************************************************************************************************************
*   DigitalSensorInit
***************************************************************************************************************************************************
*
* Brief :       Called by the RF430FRL152H ROM code.
*               Initializes the RF430's eUSCI into master mode, for use with sampling digital sensors
*               Function extends functionality of the ROM firmware.  
*               Called on reset (PUC) during initialization, only once.
*
* Param[in] :   None
*
*
* Param[out]:   None
*
* Return :      None
*
**************************************************************************************************************************************************/
#pragma RETAIN (DigitalSensorInit)	//needed to prevent of optimizing out this function
//the line below constrains the placement of the function to a specific section of code
//declared in the .cmd file.  Basically we want the function code to be placed in the end
// of the FRAM memory to not interfere with the virtual registers and log memory before it
#pragma CODE_SECTION (DigitalSensorInit, ".fram_driver_code") //see .cmd file for details
void DigitalSensorInit()
{
    //ROM sets P1OUT = 0x0F, this then consumes some current
    P1OUT = 0x00; // needed to reduce power consumption on RF430FRL152H EVM, since P1.3 is connected to a 2.2K Ohm resistor on the EVM to ground

    P1DIR &= ~MASTER_SLAVE_SELECT; // check if digital sensor mode is selected
    if (P1IN & MASTER_SLAVE_SELECT)
    {
        //P1DIR &= ~MASTER_SLAVE_SELECT;  //host controller mode selected, exit
        return;
    }
    
    /* For custom digital sensor initialization, keep the previous code as is and change the following as needed.*/

    // Configure P1.0 and P1.1 pins for I2C mode
    PORT_I2C_SEL0 |= SCL + SDA;
    PORT_I2C_SEL1 &= ~(SCL + SDA);

    // configure eUSCI for I2C
    UCB0CTL1 |= UCSWRST;	               // Software reset enabled
    UCB0CTLW0 |= UCMODE_3  + UCMST + UCSYNC + UCTR;  	// I2C mode, Master mode, sync, transmitter
    UCB0CTLW0 |= UCSSEL_2;                           	// select SMCLK at 2MHz
    UCB0BRW = 10;                                    	// 2Mhz / 10 = 200kHz
    UCB0I2CSA  = 0x0040;		// slave address of SHT21, initially
    UCB0CTL1  &= ~UCSWRST;                       	// exit reset mode
    
    return;
}

/**************************************************************************************************************************************************
*   DigitalSensorMeasurement
***************************************************************************************************************************************************
*
* Brief :       Called by the RF430FRL152H ROM code.
*               ROM code sets SENSOR_TYPE_MAILBOX to indicate which digital sensor is to be sampled
*               Calls this function
*               This function, should sample that digital sensor
*               Store the result in RESULT_MAILBOX
*               and return 1 to indicate that a successful sampling occurred.
*               Called every time there is a digital sensor measurement needed determined by the scheduler
*
* Param[in] :   SENSOR_TYPE_MAILBOX - indicated which digital sensor needs to be sampled, 1,2 or 3
*
*
* Param[out]:   RESULT_MAILBOX -  the 16-bit result of the sampling
*
* Return :      Non-zero value indicated a successful sampling occurred. ROM code will then proceed to check thresholds and store the result in FRAM logging table.
*               Zero value will result in no logging of any result for this digital sensor.
*
**************************************************************************************************************************************************/\
#pragma RETAIN (DigitalSensorMeasurement)
#pragma CODE_SECTION (DigitalSensorMeasurement, ".fram_driver_code")//see .cmd file for details
u08_t DigitalSensorMeasurement ()
{
	u08_t temp_data[2];             // used for temporary data and the 16-bit data that is sampled
	u08_t sensor_sampled = 0;          // flag to keep track if any sensor was actually sampled

	if (SENSOR_TYPE_MAILBOX == DIGITAL_SENSOR1) // does the ROM code request digital sensor 1 (SHT21 temperature) to be sampled?
	{
		/* To add processing for custom digital sensor 1:
		 * Collect data over I2C
		 * Add lines below to send data to ROM application
		 * RESULT_MAILBOX = (u16_t)temp_data[1] + (((u16_t)(temp_data[0])) << 8);	// store the result in the mailbox so that the ROM code will use it
		 * sensor_read = 1;													    // sensor read was performed
		 *
		 * If this digital sensor is not needed use only line below in this block
		 * sensor_read = 0
		 */

		/***********  Sensor Hub Boosterpack SHT21 temperature measurement ************************/
		SHT_21_I2C_Master_Measurement(TEMP_MEASURE_HOLD_MASTER, temp_data);     // take the temperature measurement
		// Keep next two lines after modification for custom measurement
		RESULT_MAILBOX = (u16_t)temp_data[1] + (((u16_t)(temp_data[0])) << 8);	// store the result in the mailbox so that the ROM code will use it
		sensor_sampled = 1;													    // sensor sampling was performed
	}
	else if(SENSOR_TYPE_MAILBOX == DIGITAL_SENSOR2)  // does the ROM code request digital sensor 2 (SHT21 humidity) to be sampled?
	{
		/* To add processing for custom digital sensor 2:
		 * Collect data over I2C
		 * Add lines below to send data to ROM application
		 * RESULT_MAILBOX = (u16_t)temp_data[1] + (((u16_t)(temp_data[0])) << 8);	// store the result in the mailbox so that the ROM code will use it
		 * sensor_read = 1;													    // sensor sampling was performed
		 *
		 * If this digital sensor is not needed use only line below in this block
		 * sensor_read = 0
		 */

		/***********  Sensor Hub Boosterpack SHT21 humidity measurement ************************/
		SHT_21_I2C_Master_Measurement(HUMIDITY_HOLD_MEASUREMENT, temp_data);	// take the humidity measurement
		temp_data[1] &= ~HUMIDITY_SENSOR_ID;  								    // clear the humidity sensor ID from the result
		RESULT_MAILBOX = (u16_t)temp_data[1] + (((u16_t)(temp_data[0])) << 8);	// store the result in the mailbox so that the ROM code will use it
		sensor_sampled = 1;													    // sensor sampling was performed
	}
	else if(SENSOR_TYPE_MAILBOX == DIGITAL_SENSOR3) // does the ROM code request digital sensor 3 (ISL29023 light) to be sampled?
	{
		/* To add processing for custom digital sensor 3:
		 * Collect data over I2C
		 * Add lines below to send data to ROM application
		 * RESULT_MAILBOX = (unsigned int)temp_data[0] + (((unsigned int)(temp_data[1])) << 8);	// store the result in the mailbox so that the ROM code will use it
		 * sensor_sampled = 1;													    // sensor read was performed
		 *
		 * If this digital sensor is not needed use only line below in this block
		 * sensor_sampled = 0
		 */

		/***********  Sensor Hub Boosterpack ISL29023 light sensor measurement ************************/
        temp_data[0] = BIT5;                        // IC measures ALS every integration cycle
        temp_data[1] = BIT1;                        // 4,000 flux range setting
        ISL29023_I2C_Write(ISL29023_COMMAND_I_REGISTER, temp_data);
        __delay_cycles(220000);                     // 110ms, only 90ms needed, but extra for margin;
        ISL29023_I2C_Read(ISL29023_DATA_LSB_REGISTER, temp_data);   // read the resulting measurement out
		RESULT_MAILBOX = (u16_t)temp_data[0] + (((u16_t)(temp_data[1])) << 8);	// store the result in the mailbox so that the ROM code will use it
        //reset the device into power-down mode
        temp_data[0] = 0;
        temp_data[1] = 0;
        ISL29023_I2C_Write(ISL29023_COMMAND_I_REGISTER, temp_data);	    // reset the device into power-down mode
        sensor_sampled = 1;							// sensor read was performed
	}
	return sensor_sampled;			// if 1 indicates that a sensor read was performed, the ROM code will only store the value on a non-zero result here
}

/**************************************************************************************************************************************************
*   SHT_21_I2C_Master_Measurement
***************************************************************************************************************************************************
*
* Brief :       Performs a measurement, temperature or humidity and returns the result
*
* Param[in] :   command - the command to be performed
*
* Param[out]:   rxData - the resulting sensor measurement
*
* Return :      None.
*
**************************************************************************************************************************************************/
#pragma CODE_SECTION (SHT_21_I2C_Master_Measurement, ".fram_driver_code") //see .cmd file for details
void SHT_21_I2C_Master_Measurement(u08_t command, u08_t * rxData)
{
	UCB0CTL1 |= UCSWRST;	            // Software reset enabled
	UCB0I2CSA  = 0x0040;		        // I2C slave address of SHT21
	//UCBCTL1  &= ~UCSWRST;           // exit eusci reset

	UCB0CTL1  &= ~UCSWRST;			// put eUSCI out of reset mode
	UCB0CTLW1 = UCASTP_1;
	UCB0TBCNT = 0x0001;
	UCB0CTL1 |= UCTXSTT + UCTR;		// start i2c write operation
	while(!(UCB0IFG & UCTXIFG0));	// wait until transmit is needed
	UCB0TXBUF = command;			    // send the command
	while(!(UCB0IFG & UCBCNTIFG)); 	// wait until the stop counter
	UCB0CTL1 &= ~UCTR; 				// read operation
	UCB0CTL1 |= UCTXSTT; 			// repeated start
	while(!(UCB0IFG & UCRXIFG0));	// wait until read data available
	rxData[0] = UCB0RXBUF;			// read the MSB
	while(!(UCB0IFG & UCRXIFG0));	// wait until more data is available
	rxData[1] = UCB0RXBUF;			// read the LSB
	UCB0CTLW0 |= UCTXSTP; 			// send stop after next byte
	while (!(UCB0IFG & UCRXIFG0));	// wait until more data is available
	//end of while loop

	{
		u08_t crc = UCB0RXBUF;  // read the crc, currently not checked
	}
	while (!(UCB0IFG & UCSTPIFG));  // Ensure stop condition got sent
	UCB0CTL1  |= UCSWRST;		   // put I2C in reset mode

	return;
}

/**************************************************************************************************************************************************
*   ISL29023_I2C_Read
***************************************************************************************************************************************************
*
* Brief :       Performs an I2C write command on the ISL29023
*
* Param[in] :   command - the register address to be written to 
* 
* Param[out]:   rxData - the two byte data read from teh ISL29023
*
* Return :      None.
*
**************************************************************************************************************************************************/
#pragma CODE_SECTION (ISL29023_I2C_Read, ".fram_driver_code") //see .cmd file for details
void ISL29023_I2C_Read(u08_t command, u08_t * rxData)
{
	UCB0CTL1 |= UCSWRST;	            // Software reset enabled
	UCB0I2CSA  = 0x0044;		        // I2C slave address of ISL29023
	UCB0CTLW1 = UCASTP_1;
	UCB0TBCNT = 0x0001;
//	UCBCTL1  &= ~UCSWRST;           // exit eusci reset

	UCB0CTL1  &= ~UCSWRST;			// put eUSCI out of reset mode
	UCB0CTL1 |= UCTXSTT + UCTR;		// start i2c write operation
	while(!(UCB0IFG & UCTXIFG0));	// wait until transmit is needed
	UCB0TXBUF = command;				// send the command
	while(!(UCB0IFG & UCBCNTIFG)); 	// wait until the stop counter
	UCB0CTL1 &= ~UCTR; 				// read operation
	UCB0CTL1 |= UCTXSTT; 			// repeated start
	while(!(UCB0IFG & UCRXIFG0));	// wait until read data available
	rxData[0] = UCB0RXBUF;			// read the MSB
	UCB0CTLW0 |= UCTXSTP; 			// send stop after next byte
	while(!(UCB0IFG & UCRXIFG0));	// wait until read data available
	rxData[1] = UCB0RXBUF;			// read the MSB
	while (!(UCB0IFG & UCSTPIFG));  	// Ensure stop condition got sent
	UCB0CTL1  |= UCSWRST;			// put I2C in reset mode

	return;
}

/**************************************************************************************************************************************************
*   ISL29023_I2C_Write
***************************************************************************************************************************************************
*
* Brief :       Performs an I2C read command on the ISL29023
*
* Param[in] :   command - the register address to be written to
*               rxData - the two byte data to be written to the ISL29023
* 
* Param[out]:   None.
*
* Return :      None.
*
**************************************************************************************************************************************************/
#pragma CODE_SECTION (ISL29023_I2C_Write, ".fram_driver_code") //see .cmd file for details
void ISL29023_I2C_Write(u08_t command, u08_t * rxData)
{
	UCB0CTL1 |= UCSWRST;	            // Software reset enabled
	UCB0I2CSA  = 0x0044;		        // I2C slave address of ISL29023
	UCB0CTLW1 = UCASTP_1;
	UCB0TBCNT = 0x0003;
//	UCBCTL1  &= ~UCSWRST;           // exit eusci reset

	UCB0CTL1  &= ~UCSWRST;           // put eUSCI out of reset
	UCB0CTL1 |= UCTXSTT + UCTR;		// start i2c write operation
	while(!(UCB0IFG & UCTXIFG0));	// send the I2C address
	UCB0TXBUF = command;				// send the command
	while(!(UCB0IFG & UCTXIFG0));    // wait until the command is sent out
	UCB0TXBUF = rxData[0];           // send the LSB
	while(!(UCB0IFG & UCTXIFG0));    // wait until it has been transmitted
	UCB0TXBUF = rxData[1];           // send the MSB
	while (!(UCB0IFG & UCBCNTIFG));  // wait until it has been transmitted
	UCB0CTL1 |= UCTXSTP;             // send the stop condition
	while (!(UCB0IFG & UCSTPIFG));   // wait until it has been received
	UCB0CTL1  |= UCSWRST;            // put the eUSCI into reset mode
    
    return;             
}


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

//#pragma CODE_SECTION(SD_ADC_ISR, ".fram_driver_code")   //comment this line for using ROM's SD14_ADC ISR, uncomment next one
#pragma CODE_SECTION(SD_ADC_ISR, ".sd_14_rom_isr") //comment this line for creating a custom SD14_ADC ISR that will exist in FRAM, bypassing ROM's, uncomment previous
#pragma vector = SD_ADC_VECTOR
__interrupt void SD_ADC_ISR(void)
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

