#include <rf430frl152h.h>

#define MASTER_SLAVE_SELECT         BIT7
#define PORT_I2C_OUT	P1OUT
#define PORT_I2C_DIR	P1DIR
#define PORT_I2C_SEL0	P1SEL0
#define PORT_I2C_SEL1	P1SEL1
#define SDA	BIT0
#define SCL BIT1




int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
   // P1DIR |= 0x1;

    //ROM sets P1OUT = 0x0F, this then consumes some current
        P1OUT = 0x00; // needed to reduce power consumption on RF430FRL152H EVM, since P1.3 is connected to a 2.2K Ohm resistor on the EVM to ground

        P1DIR &= ~MASTER_SLAVE_SELECT; // check if digital sensor mode is selected


        /* For custom digital sensor initialization, keep the previous code as is and change the following as needed.*/

        // Configure P1.0 and P1.1 pins for I2C mode
        PORT_I2C_SEL0 |= SCL + SDA;
        PORT_I2C_SEL1 &= ~(SCL + SDA);

        // configure eUSCI for I2C
        UCB0CTL1  |= UCSWRST;	               // Software reset enabled
        UCB0CTLW0 |= UCMODE_3  + UCMST + UCSYNC + UCTR;  	// I2C mode, Master mode, sync, transmitter
        UCB0CTLW0 |= UCSSEL_2;                           	// select SMCLK at 2MHz
        UCB0BRW = 20;                                    	// 2Mhz / 20 = 100kHz
        UCB0I2CSA  = 0x0018;		// slave address of SHT21, initially

        UCB0TBCNT = 0x0001;
        UCB0CTLW1 = UCASTP_1;
        UCB0CTL1  &= ~UCSWRST;                       	// exit reset mode

        unsigned short x = 0;
        while(1){
        UCB0TBCNT = 0x0001;   			//THIS IS HOW MANY BYTES EXPECTED FROM SLAVE
        UCB0CTL1  &= ~UCSWRST;           // put eUSCI out of reset
        UCB0CTL1 |= UCTXSTT + UCTR;		// start i2c write operation
        while(!(UCB0IFG & UCTXIFG0));
        UCB0TXBUF = x;
        while(!(UCB0IFG & UCBCNTIFG)); 	// wait until the stop counter

        UCB0CTL1 &= ~UCTR; 				// read operation
        UCB0CTL1 |= UCTXSTT; 			// repeated start
        while(!(UCB0IFG & UCRXIFG0));	// wait until read data available
        x = UCB0RXBUF;
        	//UCB0CTL1 |= UCTXSTP;
        	//while (!(UCB0IFG & UCSTPIFG));   // wait until it has been received
        UCB0CTL1  |= UCSWRST;            // put the eUSCI into reset mode

        volatile int i = 20000;					// SW Delay
        		do i--;
        		while(i != 0);
        }
}
