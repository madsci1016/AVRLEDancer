/*
 * AVRLEDancer.c
 *
 * Created: 1/19/2015 4:51:59 PM
 *  Author: Bill
 *
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdio.h>



/*   

Ok, take 2. This time we are going to try 'chipi-plexing' (Do we really have to call it that?) 
It's a smashup of regular old charlieplexing combined with old x,y row/column scanning. 

Here's the plan. With 8 pins you have one High driver and 7 low drivers for 7 LEDs. We'll call each
group of 7 LEDs a page. There 8 pages of 7 LEDs for a total of 56. We will try to implement BCM for 
brightness control as we are showing a page. 

If would be better to "transpose" the brightness values into bytes the interrupt can write directly to the 
IO register.  I would be a 8 x 7 matrix transpose operation. Then we would have to shift some bits around 
depending on which page we are talking about. 

I couldn't get the 'cool' way of sub matrix transpose to work. Doing it the long way. 
*/



//defines
//#define NUMLEDS 28 //number of LEDs on controller. MAX is 56
#define PAGES 4 //number of 7 LED clusters attached to controller. Max is 8
#define BAUD 57600


//static defines (don't touch)
#define MYUBBR ((F_CPU / (BAUD * 16L)) - 1) //this assumes single speed mode, U2X = 0, else change 16 to 8
#define NUMBYTES PAGES*8 //number of bytes the led driver routine has to go through


//global indexs
uint8_t reg_byte, bit_inx;  //which page are we displaying, where do we interupt
uint8_t rx_page, rx_channel; //what page are we receiving, what channel on that page

uint8_t regvalues[NUMBYTES];  //array holding transposed brightness values for BCM
uint8_t regvalues2[NUMBYTES]; //double buff

//these are the timer values we need to interrupt at
//They are a BCM waveform from MSB to LSB. 
static uint8_t bcmvals[6] = {192,224,240,248,252,254}; 

//used during transpose operation
uint8_t bit; 
uint8_t *p;

//serial state machine
uint8_t rxstate;

//global state flag
volatile uint8_t state;

#define IDLE 0
#define NEWMAIL 1
#define STOPSCAN 2

//some temporaray storage areas
uint8_t temp;
uint16_t longtemp;

/* LED addresses 
xxxxxxLH  0b00000011   0b0000001
xxxxxLxH
xxxxLxxH
xxxLxxxH
xxLxxxxH
xLxxxxxH
LxxxxxxH
xxxxxLHx
xxxxLxHx
xxxLxxHx
xxLxxxHx
xLxxxxHx
LxxxxxHx
xxxxxxHL
xxxxLHxx
xxxLxHxx
xxLxxHxx
xLxxxHxx
LxxxxHxx
xxxxxHxL
xxxxxHLx
 
*/


int main(void)
{
	//setup area
	p = regvalues; //save pointer to this array in p

	//configure timers
	//using timer 0 for plexing LEDs
	TCCR0A = 0;  //not using output pin
	TCCR0B = (1<<CS01)|(1<<CS00); //timer clock div/64
	
	TIMSK = (1<<OCIE0A) | (1<<TOIE0); //compare match A interrupt and overflow interrupt enabled

	/* Set baud rate */
	UBRRH = (unsigned char)(MYUBBR>>8);
	UBRRL = (unsigned char)MYUBBR;
	//if we want to mess with double speed latter
	//UCSRA = (1<<U2X);
	
	/* Enable receiver and transmitter */
	UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN);
	/* Set frame format: 8data, 1stop bit */
	UCSRC = (1<<UCSZ0)|(1<<UCSZ1);
	 //UCSRC = (1 << USBS) | (3 << UCSZ0);	// asynchron 8n1
	/*
	//TESTING
	regvalues[0]=0b00000111;
	regvalues[1]=0b00000001;
	regvalues[2]=0b00000011;
	regvalues[3]=0b00000001;
	regvalues[4]=0b00000011;
	regvalues[5]=0b00000001;
	regvalues[6]=0b00000011;
	regvalues[7]=0b00001001;
	regvalues[8]=0b00000111;
	regvalues[9]=0b00000001;
	regvalues[10]=0b00000011;
	regvalues[11]=0b00000001;
	regvalues[12]=0b00000011;
	regvalues[13]=0b00000001;
	regvalues[14]=0b00000011;
	regvalues[15]=0b00001001;
	regvalues[16]=0b00000111;
	regvalues[17]=0b00000001;
	regvalues[18]=0b00000011;
	regvalues[19]=0b00000001;
	regvalues[20]=0b00000011;
	regvalues[21]=0b00000001;
	regvalues[22]=0b00000011;
	regvalues[23]=0b00001001;
	*/
	//TESTING
	
	
	//set all LEDs on. This also sets our High side driver for each page, bits that will be skiped
	//by the transpose operation later on. 
	for(int i=0; i<PAGES * 8; i++){
		regvalues[i]=0xFF;	
	}
	
	memcpy(regvalues2, regvalues, sizeof(regvalues));
	
	//enable interrupts
	sei();
	
    while(1)
    {
        //TODO:: Please write your application code 

		//We are double buffering the LED values. This fixes a problem with the timer routine
		//interrupting the serial transpose operation and acting on bad data. 
		
		//I used a global flag to mark when there's new data, stop the timer at the next refresh and copy the data over
		
		//we need to check global state and copy buffer when need be. 
		if(state == STOPSCAN){
			state = IDLE;
			memcpy(regvalues2, regvalues, sizeof(regvalues));
			
			//restart scanning timer
			DDRB = regvalues2[reg_byte++];
			PORTB = 1;
	
			//clear interrupt, clear prescailer, start timer
			TIFR = (1<<OCF0A);
			GTCCR = (1<<PSR10);
			TCCR0B = (1<<CS01)|(1<<CS00);
		}			
    }
}

//called when we have a byte come in
ISR(USART_RX_vect) {

 //grab byte
 temp = UDR;
 
 //(re)enable interrupts so PWM code takes priority
 sei();
 
 switch(rxstate){
	case 0: //look for sync
		if (temp == 0x7E){
			rxstate=1;
			//send sync
			while ( !( UCSRA & (1<<UDRE)) ) ;							
			UDR = 0x7E;
			
		}			
	break;
	case 1: //look for my address
		if (temp == 0x80){
			rxstate = 2;
			//setup intial states, first page, first channel (skips 0 as it's the first High driver)
			rx_page = 0;
			rx_channel = 0;
			bit = 1;
			
		}
		else if (temp > 0x80){ //not my address, subtract 1 and forward rest
			rxstate = 4;
			while ( !( UCSRA & (1<<UDRE)) ) ;							
			UDR = --temp;
		}
	break;
	
	
	
	case 2: //peel off my data
		if (temp == 0x7D) //pad byte, drop
			break;
		if (temp == 0x7F)//escape byte
			rxstate = 3;
		else {
			//write brightness value into memory, transposed 
			
			
			//rotate my bitmask 
			//There's no concept of rotation in the C language so doing this the long way
			//instead of using inline assembly.
			bit = (bit << 1) | (bit>>7);
			//set the pointer back to first byte in page
			p = regvalues + rx_page * 8;
			//convert rx_channel to a bit mask
			 rx_channel++;
			//transpose 
			for (uint8_t mask = 0b10000000 ; mask ; mask >>= 1) {
				if (temp & mask) {
				*p++ |= bit;
				} else {
				*p++ &= ~bit;
				} 
			}
			
		
			//see if we need to add one to page
			if(rx_channel == 7){
				rx_page++;
				rx_channel = 0;
				//reset the bit to the starting channel for the next page
				bit = (1<<rx_page);
				//see if we got everything
				if(rx_page == PAGES) {
					state = NEWMAIL;
					rxstate = 4;
					while ( !( UCSRA & (1<<UDRE)) ) ;							
					UDR = 0x80;	
				}					
			}				
		}							
	break;
	case 3: //peel off escaped byte				
		//values[RXLED++] = temp + 0x4E;
		/*if(RXLED == NUMLEDS){
			rxstate = 4;
			while ( !( UCSRA & (1<<UDRE)) ) ;							
			UDR = 0x80;	
		}			
		else
			rxstate = 2;*/
	break;
	
	
	
	
	
	case 4: //forward rest
		if(temp == 0x7E) 
			rxstate = 1;
		while ( !( UCSRA & (1<<UDRE)) ) ;							
		UDR = temp;	
	break;
	 
 	 
	}    
}


//timer compare match interrupt. Turn off led on compare
ISR(TIMER0_COMPA_vect){

	//load next byte into IO register
	DDRB = regvalues2[reg_byte++];
	//load next compare value
	OCR0A = bcmvals[bit_inx++];

}

//timer overflow interrupt. Turn on next LED on overflow
ISR(TIMER0_OVF_vect){
	
	//Port off
	PORTB = 0;
	//stop timer
	TCCR0B = 0;
	//clear count
	TCNT0 = 0;
	//load first compare value
	OCR0A = 128;	
	//reset bit inx	
	bit_inx=0;
	
	//check array counter
	if(reg_byte == NUMBYTES){
		reg_byte = 0;
		//check global state for double buff call
		if (state == NEWMAIL){
			state = STOPSCAN;
			return;
			}
	}			
	
	
	//set IO register for this page
	DDRB = regvalues2[reg_byte++];
	PORTB = 1<<(reg_byte/8);
	
	//clear interrupt, clear prescailer, start timer
	TIFR = (1<<OCF0A);
	GTCCR = (1<<PSR10);
	TCCR0B = (1<<CS01)|(1<<CS00);
	
}