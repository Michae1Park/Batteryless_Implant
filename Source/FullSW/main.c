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
#include "patch.h"
#include "types.h"

#define MASTER_SLAVE_SELECT         BIT7
#define PORT_I2C_OUT    P1OUT
#define PORT_I2C_DIR    P1DIR
#define PORT_I2C_SEL0    P1SEL0
#define PORT_I2C_SEL1    P1SEL1
#define SDA    BIT0
#define SCL BIT1


#define GCTRL_REG (*((volatile u08_t*)0xF868))
#define SENSORCTRL_REG (*((volatile u08_t*)0xF86A))
#define NUM_PASSES_REG (*((volatile u08_t*)0xF86C))
#define STATUS_REG (*((volatile u08_t*)0xF869))
#define SAMPLED_DATA (*((volatile u16_t*)0xF8B0))
#define MEM_SIZE_REG (*((volatile u16_t*)0xF8A8))

/**************************************************************************************************************************************************
*   Code Space
***************************************************************************************************************************************************
*
*  Please check the lnk_rf430frl152h_ROM_Init.cmd file to customize how much code space is used for logging("FRAM")
*  or for code ("FRAM_CODE").  
*  Also for each new function, please use the following line before the function, to put the code in the correct section:
*  #pragma CODE_SECTION (MyFunction, ".fram_driver_code") //also change MyFunction to the function name
*
**************************************************************************************************************************************************/

//*****************************Summary********************************/

/*This project is meant to simply initialize the FRAM of the device so that
 * it will be operational.  It initializes the FRAM ISR to the correct value
 * to point to the ROM code, which runs the device.
 *
 */

#pragma RETAIN (writeRegister)
#pragma CODE_SECTION (writeRegister, ".fram_driver_code")
void writeRegister(u08_t reg, u08_t data){
//    UCB0CTL1  |= UCSWRST;
//    UCB0TBCNT = 0x0002;
//    UCB0CTL1  &= ~UCSWRST;          // put eUSCI out of reset
//    UCB0CTL1 |= UCTXSTT + UCTR;        // start i2c write operation
//    while(!(UCB0IFG & UCTXIFG0));
//    UCB0TXBUF = reg;
//    //UCB0CTL1 |= UCTXSTT + UCTR;        // start i2c write operation
//    while(!(UCB0IFG & UCTXIFG0));
//    UCB0TXBUF = data;
//    while(!(UCB0IFG & UCBCNTIFG));     // wait until the stop counter
//    UCB0CTL1 |= UCTXSTP;             // send the stop condition
//        while (!(UCB0IFG & UCSTPIFG));
//    //while(!(UCB0IFG & UCBCNTIFG));     // wait until the stop counter
    UCB0CTL1 |= UCSWRST;                // Software reset enabled
    //    UCB0I2CSA  = 0x001D;                // I2C slave address of ISL29023
    //    UCB0CTLW1 = UCASTP_1;
        UCB0TBCNT = 0x0002;
    //    UCBCTL1  &= ~UCSWRST;           // exit eusci reset

        UCB0CTL1  &= ~UCSWRST;           // put eUSCI out of reset
        UCB0CTL1 |= UCTXSTT + UCTR;        // start i2c write operation
        while(!(UCB0IFG & UCTXIFG0));    // send the I2C address
        UCB0TXBUF = reg;                // send the command
        while(!(UCB0IFG & UCTXIFG0));    // wait until the command is sent out
        UCB0TXBUF = data;
        while (!(UCB0IFG & UCBCNTIFG));  // wait until it has been transmitted
        UCB0CTL1 |= UCTXSTP;             // send the stop condition
        while (!(UCB0IFG & UCSTPIFG));   // wait until it has been received
        UCB0CTL1  |= UCSWRST;            // put the eUSCI into reset mode
}

//#pragma RETAIN (readRegister)
#pragma CODE_SECTION (readRegister, ".fram_driver_code")
unsigned short readRegister(u08_t reg){
//    UCB0CTL1  |= UCSWRST;
//    UCB0TBCNT = 0x0001;               //THIS IS HOW MANY BYTES EXPECTED FROM SLAVE
//    UCB0CTL1  &= ~UCSWRST;           // put eUSCI out of reset
//    UCB0CTL1 |= UCTXSTT + UCTR;        // start i2c write operation
//    while(!(UCB0IFG & UCTXIFG0));
//    UCB0TXBUF = reg;
//    while(!(UCB0IFG & UCBCNTIFG));     // wait until the stop counter
//    UCB0CTL1 &= ~UCTR;                 // read operation
//    UCB0CTL1 |= UCTXSTT;             // repeated start
//    while(!(UCB0IFG & UCRXIFG0));    // wait until read data available
//    return UCB0RXBUF;
unsigned short ret;

    UCB0CTL1 |= UCSWRST;                // Software reset enabled
    UCB0TBCNT = 0x0001;

    UCB0CTL1  &= ~UCSWRST;            // put eUSCI out of reset mode
    UCB0CTL1 |= UCTXSTT + UCTR;        // start i2c write operation
    while(!(UCB0IFG & UCTXIFG0));    // wait until transmit is needed
    // RF13MTXF = 1;
    UCB0TXBUF = reg;                // send the command
    while(!(UCB0IFG & UCBCNTIFG));     // wait until the stop counter
    // RF13MTXF = 2;
    UCB0CTL1 &= ~UCTR;                 // read operation
    UCB0CTL1 |= UCTXSTT;             // repeated start
    while(!(UCB0IFG & UCRXIFG0));    // wait until read data available
//     RF13MTXF = 3;
    ret = UCB0RXBUF;            // read the MSB
    UCB0CTLW0 |= UCTXSTP;             // send stop after next byte

    while (!(UCB0IFG & UCSTPIFG));      // Ensure stop condition got sent
    // RF13MTXF = 4;
    UCB0CTL1  |= UCSWRST;            // put I2C in reset mode
    return ret;
}

/*  SetupSD14                                                                                            *
 *  The channel to be sampled (thermistor or reference resistor)                                          *
 *  Function:  This function is setup for sampling either a thermistor or a reference resistor             */
void SetupSD14(unsigned char channel)
{
    // setting: channel to be sampled, the programmable amplifier gain (2x), CIC filter, SD14INTDLY0 needed since CIC filter needs atleast two consecutive samples before producing an accurate result
    // SDRATE at fastest result possible but also not the most accurate, also enabled is the SD14RBEN which is the current source into the thermistor and references resistor
//    SD14CTL1 = SD14RBEN1 + SD14RBEN0 + SD14UNI + SD14GAIN0 + SD14INTDLY0 + 0x1;//channel;
	SD14CTL1 = SD14RBEN1 + SD14RBEN0 + SD14UNI + SD14RATE1 + SD14RATE0 + SD14INTDLY0 + 0x1;//channel;


    //SD14CTL1 = SD14UNI + 0x1;
    //SD14SGL = single conversion, clock from ACLK (64kHz from VLO clock), SD14DIV1 is set for divide by 32 times (SD14 needs a 2kHz clock),
    SD14CTL0 = SD14IE + SD14SGL + SD14DIV1;     // 2 kHz sampling rate, ACLK source, SVSS ground (.125V), interrupt enable

    SD14CTL0 |= SD14EN;               // SD14 module enabled,
    SD14CTL0 |= SD14SC;               // start the conversion
}

u16_t SamplesBuffer[4];
u08_t State;

enum state_type
{
    IDLE_STATE                                      = 1,
    ONE_SHOT_TEMP_REFERENCE_SAMPLE_STATE            = 2,
    ONE_SHOT_TEMP_THERMISTOR_SAMPLE_STATE           = 3
};

enum Channel_Types
{
    ADC0_CHANNEL                        = 0x0,
    INTERNAL_TEMPERATURE_CHANNEL        = 0x1,
    THERMISTOR_ADC2_CHANNEL             = 0x2,
    REFERENCE_ADC1_CHANNEL              = 0x3,
};
//
//#pragma vector=SD_ADC_VECTOR
//interrupt void SD14_ADC (void)
//{
//    switch(__even_in_range(SD14IV,4))
//    {
//        case SD14IV__NONE: // no interrupt pending
//            break;
//        case SD14IV__OV: //SD14MEM overflow - SD14OVIFG
//            SD14CTL0 &= ~SD14OVIFG; // clear the overflow bit
//            break;
//        case SD14IV__RES:
//            SD14CTL0 &= ~SD14IFG;   // clear the data available interrupt
//            if (State == ONE_SHOT_TEMP_REFERENCE_SAMPLE_STATE)
//            {
//                State = ONE_SHOT_TEMP_THERMISTOR_SAMPLE_STATE;
//                SamplesBuffer[0] = SD14MEM0;            // compensation for thermistor current bias error
//                SetupSD14(THERMISTOR_ADC2_CHANNEL);              //setup ADC and start the conversion
//            }
//            else if (State == ONE_SHOT_TEMP_THERMISTOR_SAMPLE_STATE)
//            {
//                SamplesBuffer[1] = SD14MEM0;            // compensation for thermistor current bias error
//                SD14CTL0 &= ~SD14EN; //disable the SD14 until it is restarted if using consecutive readings by the timer
//                State = IDLE_STATE;
//                //conversion completed, data available
//                __bic_SR_register_on_exit(LPM4_bits);      //exit LPM mode after this interrupt
//            }
////            SamplesBuffer[0] = SD14MEM0;            // compensation for thermistor current bias error
////            SetupSD14(INTERNAL_TEMPERATURE_CHANNEL);
////            __bic_SR_register_on_exit(LPM4_bits);
//            break;
//    }
//}




//*****************************FUNCTION PROTOTYPES********************************/
void userCustomCommand();
//********************************************************************************/
//see .cmd file for details - all new firmware must go into fram_driver_code memory section space

/**************************************************************************************************************************************************
*  userCustomCommand
***************************************************************************************************************************************************
*
* Brief : This function is called by the RF stack whenever a custom command by its ID number is transmitted
*
* Param[in] :   None
*
* Param[out]:   None
*
* Return        None
*
* This is an example only, and the user if free to modify as needed.
*
* Operation: Example with TRF7970AEVM
* Use Test tab to send following sequence: 18 02 AA 07 10 10
* 18 - TRF7970AEVM Host command (omit for other readers - not sent out over RF)
* 02 - High speed mode selection (start of actuall RF packet)
* AA - The actual custom command
* 07 - TI Manufacturer ID (need by this IC)
* 01 - Set Error LED to on  (0x00 to be off)
**************************************************************************************************************************************************/
#pragma CODE_SECTION (userCustomCommand, ".fram_driver_code") //see .cmd file for details - all new firmware must go into fram_driver_code memory section space
void userCustomCommand()
{
    __bic_SR_register(GIE);

    u08_t control;

    if( RF13MFIFOFL_L == CRC_LENGTH_IN_BUFFER + DATA_IN_LENGTH){
        control = RF13MRXF_L;
        //RF13MTXF = control;
        //return;

        if(control == 8) {
            RF13MTXF = 0x0;
            __bis_SR_register(GIE);
            return;
        }

        if(control == 0xff)
        {
            State = ONE_SHOT_TEMP_REFERENCE_SAMPLE_STATE;
            SetupSD14(1);
            //RF13MTXF = SamplesBuffer[0];
            RF13MTXF = SD14MEM0;
            __bis_SR_register(GIE);
            return;
        }

        P1OUT = 0x00;
        P1DIR &= ~MASTER_SLAVE_SELECT;

        // Configure P1.0 and P1.1 pins for I2C mode
        PORT_I2C_SEL0 |= SCL + SDA;
        PORT_I2C_SEL1 &= ~(SCL + SDA);

        // configure eUSCI for I2C
        UCB0CTL1  |= UCSWRST;                   // Software reset enabled
        UCB0CTLW0 |= UCMODE_3  + UCMST + UCSYNC + UCTR;      // I2C mode, Master mode, sync, transmitter
        UCB0CTLW0 |= UCSSEL_2;                               // select SMCLK at 2MHz
        UCB0BRW = 20;                                        // 2Mhz / 20 = 100kHz
    //    UCB0I2CSA  = 0x001D;        // slave address of SHT21, initially
        UCB0I2CSA = 0x0018;                // I2C slave address of accelerometer
        UCB0CTLW1 = UCASTP_1;
//        UCB0TBCNT = 0x0001;
//        UCB0CTLW1 = UCASTP_1;

        unsigned short a, b;
//
//        UCB0TBCNT = 0x0001;               //THIS IS HOW MANY BYTES EXPECTED FROM SLAVE
//        UCB0CTL1  &= ~UCSWRST;           // put eUSCI out of reset
//        UCB0CTL1 |= UCTXSTT + UCTR;        // start i2c write operation
//        while(!(UCB0IFG & UCTXIFG0));
//        UCB0TXBUF = x;
//        while(!(UCB0IFG & UCBCNTIFG));     // wait until the stop counter
//        UCB0CTL1 &= ~UCTR;                 // read operation
//        UCB0CTL1 |= UCTXSTT;             // repeated start
//        while(!(UCB0IFG & UCRXIFG0));    // wait until read data available
//        x = UCB0RXBUF;
                //UCB0CTL1 |= UCTXSTP;
                //while (!(UCB0IFG & UCSTPIFG));   // wait until it has been received
//        RF13MTXF = 0xdd;
//        a = readRegister(x);
//        //RF13MTXF = 1;
//        writeRegister(x, control);
//        ///RF13MTXF = 2;
//        b = readRegister(x);
//        //RF13MTXF = 3;

        switch(control){
                case 0:            //set up temp config
                    writeRegister(0x1F, 0xC0);
                    writeRegister(0x23, 0x80);
                    //writeRegister(0x24, 0x40);
                    //writeRegister(0x2E, 0x00);
                    //writeRegister(0x2E, 0x40);
                    break;
                case 1:            //set control reg and turn on
                    //writeRegister(0x2E, 0x40);
                    writeRegister(0x20, 0x2F);
                    break;
                case 2:            //read X acc
                    a = readRegister(0x28);
                    //b = readRegister(0x29);
                    RF13MTXF = a;//  + (b << 8);
                    break;
                case 3:            //read Y acc
                    a = readRegister(0x2A);
                    b = readRegister(0x2B);
                    RF13MTXF = a + (b << 8);
                    break;
                case 4:            //read Z acc
                    a = readRegister(0x2C);
                    b = readRegister(0x2D);
                    RF13MTXF = a + (b << 8);
                    break;
                case 5:
                    a = readRegister(0x2F);
                    RF13MTXF = a;
                    break;
                default:
                    a = readRegister(control);
                    RF13MTXF = a;
                    break;
                }


//        //MMA
//        switch(control){
//        case 0:            //enter standby
//            a = readRegister(0x2A);
//            a &= ~0x1;
//            writeRegister(0x2A, a);
//            break;
//        case 1:            //set scale
//            a = readRegister(0x0E);
//            a &= ~0x3;
//            writeRegister(0x0E, a);
//            break;
//        case 2:         //set output data rate
//            a = readRegister(0x2A);
//            a &= ~0x38;
//            a |= 0x30;
//            writeRegister(0x2A, a);
//            break;
//        case 3:            //portrait/landscape
//            a = readRegister(0x11);
//            a &= ~0x40;
//            writeRegister(0x11, a);
//            break;
//        case 4:            //enter active
//            a = readRegister(0x2A);
//            a |= 0x1;
//            writeRegister(0x2A, a);
//            break;
//        case 5:            //read X acc
//            a = readRegister(0x01);
//            b = readRegister(0x02);
//            RF13MTXF = (a << 4) + (b >> 4);
//            break;
//        case 6:            //read Y acc
//            a = readRegister(0x03);
//            b = readRegister(0x04);
//            RF13MTXF = (a << 4) + (b >> 4);
//            break;
//        case 7:            //read Z acc
//            a = readRegister(0x05);
//            b = readRegister(0x06);
//            RF13MTXF = (a << 4) + (b >> 4);
//            break;
//        }


        UCB0CTL1  |= UCSWRST;            // put the eUSCI into reset mode
        RF13MTXF = 0xAAAA;
        //RF13MTXF = b;
    }
    else {
        RF13MTXF = 0xFFFF;    // an error response
    }
    __bis_SR_register(GIE);
}

/**************************************************************************************************************************************************
*   ErrataFix
***************************************************************************************************************************************************
*
* Brief :       Called by the RF430FRL152H ROM code.
*                Fixes the case where using only one senosr and CIC filter, the ROM code produces inaccurate SD14 results.
*
* Param[in] :   None
*
*
* Param[out]:   None
*
**************************************************************************************************************************************************/\

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

#pragma CODE_SECTION(SD_ADC_ISR, ".fram_driver_code")   //comment this line for using ROM's SD14_ADC ISR, uncomment next one
//#pragma CODE_SECTION(SD_ADC_ISR, ".sd_14_rom_isr") //comment this line for creating a custom SD14_ADC ISR that will exist in FRAM, bypassing ROM's, uncomment previous
#pragma vector = SD_ADC_VECTOR
__interrupt void SD_ADC_ISR(void)
{
    switch(__even_in_range(SD14IV,4))
    {
        case SD14IV__NONE: // no interrupt pending
            break;
        case SD14IV__OV: //SD14MEM overflow - SD14OVIFG
            SD14CTL0 &= ~SD14OVIFG; // clear the overflow bit
            break;
        case SD14IV__RES:
            SD14CTL0 &= ~SD14IFG;   // clear the data available interrupt
            if (State == ONE_SHOT_TEMP_REFERENCE_SAMPLE_STATE)
            {
                State = ONE_SHOT_TEMP_THERMISTOR_SAMPLE_STATE;
                //SamplesBuffer[0] = SD14MEM0;            // compensation for thermistor current bias error
                SetupSD14(THERMISTOR_ADC2_CHANNEL);              //setup ADC and start the conversion
            }
            else if (State == ONE_SHOT_TEMP_THERMISTOR_SAMPLE_STATE)
            {
                //SamplesBuffer[1] = SD14MEM0;            // compensation for thermistor current bias error
                SD14CTL0 &= ~SD14EN; //disable the SD14 until it is restarted if using consecutive readings by the timer
                State = IDLE_STATE;
                //conversion completed, data available
                //__bic_SR_register_on_exit(LPM4_bits);      //exit LPM mode after this interrupt
            }
//            SamplesBuffer[0] = SD14MEM0;            // compensation for thermistor current bias error
//            SetupSD14(INTERNAL_TEMPERATURE_CHANNEL);
//            __bic_SR_register_on_exit(LPM4_bits);
            break;
    }
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
