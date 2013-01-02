/*
* Copyright (c) 2011 Wind River Systems, Inc. 
* 
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License version 2 as 
* published by the Free Software Foundation. 
* 
* This program is distributed in the hope that it will be useful, 
* but WITHOUT ANY WARRANTY; without even the implied warranty of 
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
* See the GNU General Public License for more details. 
* 
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 
* Test application for Android Benchmark Platform
* used in acceleration sensor testing for specific
* smarthphone landscape/portait view switch.
* 
* Cristian Axenie - Automotive Development Unit Engineer
* 
* The development included :
* - GPIO config for stepper driver
* - software UART for host-target communication
* - accessory functions to handle/process UART data
* - use DEBUG_ON if flash is bigger than 2K 
* - DEBUG_ON used to test only UART bi-directional functionality
*/
 
 // specific headers
#include 	"stdlib.h"
#include	"stdbool.h"
#include	"msp430g2231.h"


// globals and defines
// #define 	DEBUG_ON
#define		TXD     BIT1    // UART TXD on P1.1
#define		RXD		BIT2	// UART RXD on P1.2

#define     BIT_TIME       104  // 9600 Baud, SubMainCLK=1MHz (1MHz/9600)=104
#define		HALF_BIT_TIME	52	// Time for half a bit.

unsigned char bit_count;		// bit count, used when transmitting a byte
#ifdef DEBUG_ON
unsigned int TX_byte;		    // value sent over UART when calling transmit function
#endif
unsigned int RX_byte;			// value received over the UART once hasRecieved is set
unsigned int RX_buffer[4];		// buffer to store received data (angle value)	
int i=0;						// general purpose counter
int RX_cnt=0;					// RX buffer counter
unsigned int degrees;			// degrees value from RX UART packet 

bool is_receiving;		// Status for when the device is receiving
bool rx_complete;		// Inform when a byte was received received

int steps_done = 0;
#define FULL_CYCLE 200 // steps per full revolution

// function decls
// converts int to ascii representation
char *itoa(int val, int base);
// converts ascii representation to values
unsigned int ascii_to_val(unsigned int ch1, unsigned int ch2, unsigned int ch3);
// implements a basic (pre-measured) delay
void delay();
// energize coils for a defined sequence of steps
void step_motor(int set_pos);
// step the motor for a specific number of steps
void set_position(int steps);
#ifdef DEBUG_ON
// transmit function used in DEBUG mode to receive info
// about the number of steps to run for a certain angle
void transmit(void);
#endif

// funcs defines
// int to ascii conversion using specified radix
char* itoa(int val, int base){
	static char buf[32] = {0};
	int i = 30;
	// determine the corresponding ascii
	for(; val && i ; --i, val /= base)
		// fill the value
		buf[i] = "0123456789abcdef"[val % base];
	return &buf[i+1];
}

// convert received ascii char in value
unsigned int ascii_to_val(unsigned int ch1, unsigned int ch2, unsigned int ch3){
    unsigned int ch;
    if(ch1 <= 0x39) ch1=ch1&0x0f; // test if numeric
    else ch1= (ch1-7) & 0x0f; // if letter
    if(ch2 <= 0x39) ch2=ch2&0x0f; 
    else ch2= (ch2-7) & 0x0f;
    if(ch3 <= 0x39) ch3=ch3&0x0f;
    else ch3= (ch3-7) & 0x0f;
    if(ch1!=0x0A && ch2!=0x0A && ch3!=0x0A){
    	 ch=(ch1<<8)|(ch2<<4)|ch3; // construct the value
    }else if(ch1==0x0A && ch2!=0x0A && ch3!=0x0A){
    			ch=(ch2<<4)|ch3; 
    }else if(ch1!=0x0A && ch2==0x0A && ch3!=0x0A){
    			ch=ch1;	
    }else if(ch1!=0x0A && ch2!=0x0A && ch3==0x0A){
    			ch=(ch1<<4)|ch2;	
    }else{
    		ch=0x00;
    }
    return(ch);
}

// TODO rewrite delay() function using _delay_cycles() intrinsic
void delay(){
 int i;
 	for(i = 0; i < 700; i++)
	{
	// wait here for a while.
	}
}

// setup the motor step sequence
void step_motor(int set_pos){
	// powerup the coils to move the stepper
	  switch (set_pos) {
		  case 0:    
			P1OUT = BIT3 + BIT7;
			break;
		  case 1:    
			P1OUT = BIT3 + BIT4;
			break;
		  case 2:   
			P1OUT = BIT4 + BIT5;
			break;
		  case 3:   
			P1OUT = BIT5 + BIT7;
		  	break;
	  }
	  steps_done++;
}	

// move the stepper a certain amount of steps
void set_position(int steps) {
    int step_number = 0;
    int full_steps=steps;
	// decrement the number of steps, moving one set_position each time:
	while(steps > 0) {
	delay(); // step sequencing
		  // set_position the motor to set_position number 0, 1, 2, or 3
		  step_motor(step_number%4);
		    step_number++;
		    if (step_number == full_steps) {
		      step_number = 0;
		    }
		  // decrement the steps left
		  steps--;
	}
}
// entry point
void main(void)
{
	char *conv;		// conversion value handler
    int rez_conv;	// conversion result 
    int steps;		// extracted steps number
#ifdef DEBUG_ON
    char *s;		// aux vars for UART TX
    int t;
#endif
    
	WDTCTL = WDTPW + WDTHOLD;		// stop WDT by setting password and holding it
  
	BCSCTL1 = CALBC1_1MHZ;			// set basic clock system frequency in ctrl reg
	DCOCTL = CALDCO_1MHZ;			// set the oscillator frequency SMCLK = DCO = 1MHz  
	
	P1OUT = 0x00;					// reset GPIO port 1
#ifdef DEBUG_ON
	P1SEL |= TXD;					// setup GPIO port function for TIMERA use (UART TX)
	P1DIR |= TXD;					// setup GPIO direction 
#endif
	P1IES |= RXD;				// RXD Hi/Lo edge interrupt
	P1IFG &= ~RXD;				// clear RXD (flag) before enabling interrupt
	P1IE |= RXD;				// enable RXD interrupt
  
    P1DIR |= (BIT3 + BIT4 + BIT5 + BIT7); // setup stepper driver pins in GPIO direction reg

	is_receiving = false;			// set initial values
	rx_complete = false;
  
	__bis_SR_register(GIE);			// enable interrupts
    
	while(1)
	{
		if (rx_complete)		// if the device has recieved a value
		{
			rx_complete = false;	// clear the flag
#ifdef DEBUG_ON
			// transmit the received value (degrees) byte by byte
			TX_byte = RX_buffer[0];
			transmit();
			TX_byte = RX_buffer[1];
			transmit();
			TX_byte = RX_buffer[2];
			transmit();
#endif
			// compute the degree value for internal processing
			degrees=ascii_to_val(RX_buffer[0], RX_buffer[1], RX_buffer[2]);
#ifdef DEBUG_ON
			// add line terminators
			TX_byte=0x0A;
			transmit();
			TX_byte=0x0D;
			transmit();
#endif
			// convert extracted value
			conv = itoa(degrees,16);
			// get the int representation 
			rez_conv = atoi(conv);
			// compute the steps to move knowing the stepper resolution
			steps = rez_conv/1.8;
#ifdef DEBUG_ON
			// format string to output
			s=itoa(steps, 10);
			s[3]='\0';
			// sends the corresponding (rounded) number of steps 
			for(t=0;t<=2;t++){
			// according to the input degree value
			if(s[t]!=NULL){
				TX_byte = s[t];
				transmit();
			 }
			}
			// add line terminators
			TX_byte=0x0A;
			transmit();
			TX_byte=0x0D;
			transmit();
			for(i=0;i<3;i++){
				RX_buffer[i]=0x00;
			}	
#endif
		__bic_SR_register(GIE);			// interrupts disabled
		// execute movement
		if(steps==0){ 
				set_position(FULL_CYCLE-steps_done);// return to base position
				steps_done = 0;
		}else{
				set_position(steps); // else step to desired angle
		}
		__bis_SR_register(GIE);			// interrupts enabled
		}
		if (~rx_complete)		// loop again if another value has been received
	  			__bis_SR_register(CPUOFF + GIE);        
			// LPM0, the RX interrupt will wake the processor up. this is done so that it does not
			//	endlessly loop when no value has been received.
	}
}

#ifdef DEBUG_ON
// function transmits character from TX_byte 
void transmit()
{ 
	while(is_receiving);			// wait for RX completion
  	CCTL0 = OUT;				// TXD Idle as logical 1 
  	TACTL = TASSEL_2 + MC_2;	// select SMCLK and use timer in continuous mode

  	bit_count = 0xA;		// load bit counter, 8 bits + ST/SP = 1o bits
  	CCR0 = TAR;				// initialize compare register
  
  	CCR0 += BIT_TIME;			// set time till first bit
  	TX_byte |= 0x100;			// add stop bit to TX_byte (which is logical 1)
  	TX_byte = TX_byte << 1;		// add start bit (which is logical 0)
  
  	CCTL0 =  CCIS0 + OUTMOD0 + CCIE;	// set signal, intial value, enable interrupts
  	while ( CCTL0 & CCIE );				// wait for previous TX completion
}
#endif

// Port 1 interrupt service routine for UART RX
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{  	
	is_receiving = true;		// set receiving flag
	
	P1IE &= ~RXD;			// disable RXD interrupt
	P1IFG &= ~RXD;			// clear RXD IFG (interrupt flag)
	
  	TACTL = TASSEL_2 + MC_2;	// SMCLK select and use timer continuous mode
  	CCR0 = TAR;					// initialize compare register
  	CCR0 += HALF_BIT_TIME;		// set time till first bit
	CCTL0 = OUTMOD1 + CCIE;		// dissable TX and enable interrupts
	
	RX_byte = 0;				// initialize RX_byte
	bit_count = 0x9;			// load bit counter, 8 bits + ST
}

// Timer A0 interrupt service routine for software UART usage
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void){
#ifdef DEBUG_ON
	if(!is_receiving)
	{
		CCR0 += BIT_TIME;			// add Offset to CCR0  
		if ( bit_count == 0)		// if all bits TXed
		{
  			TACTL = TASSEL_2;		// select SMCLK and turn timer off
			CCTL0 &= ~ CCIE ;		// disable interrupt
		}
		else
		{
			CCTL0 |=  OUTMOD2;		// set TX bit to 0
			if (TX_byte & 0x01)
				CCTL0 &= ~ OUTMOD2;	// if it should be 1, set it to 1
			TX_byte = TX_byte >> 1; // extract the value to send
			bit_count --;
		}
	}
#endif	
	if(is_receiving)
	{
		CCR0 += BIT_TIME;				// add Offset to CCR0  
		if ( bit_count == 0)
		{
  			TACTL = TASSEL_2;			// select SMCLK and set timer off (for power consumption)
			CCTL0 &= ~ CCIE ;			// disable interrupt
			
			is_receiving = false;
			
			P1IFG &= ~RXD;				// clear RXD IFG (interrupt flag)
			P1IE |= RXD;				// enabled RXD interrupt
			
			if ( (RX_byte & 0x201) == 0x200)		// validate the start and stop bits are correct and unpack
			{
				RX_byte = RX_byte >> 1;		// remove start bit
				RX_byte &= 0xFF;			// remove stop bit
					// fill in the buffer and adjust counter
					RX_buffer[RX_cnt] = RX_byte;
					RX_cnt++;
					// if reached the maximum dim for the data field, reset counter
					if(RX_byte==0x0a || RX_cnt==4){
				   		rx_complete = true; // set the flag 
				    	RX_cnt = 0; // reset buffer counter
				    }
			}
  			__bic_SR_register_on_exit(CPUOFF);	// enable CPU so the main while loop continues
		}
		else
		{
			if ( (P1IN & RXD) == RXD)		// if bit is set
				RX_byte |= 0x400;			// set the value in the RX byte
				RX_byte = RX_byte >> 1;		// shift the bits down
				bit_count --;				// decrement counter
		}
	}
}
