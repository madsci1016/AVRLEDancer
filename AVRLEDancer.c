/*
 * AVRLEDancer.c
 *
 * Created: 1/19/2015 4:51:59 PM
 *  Author: Bill
 */ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdio.h>

//defines
#define NUMLEDS 28 //number of LEDs on controller. MAX is 56
#define BAUD 38400


//static defines (don't touch)
#define MYUBBR ((F_CPU / (BAUD * 16L)) - 1) //this assumes single speed mode, U2X = 0, else change 16 to 8
#define UPSCALE 1

//globals
uint8_t LED; //which LED we are scanning right now
uint8_t RXLED; //which LED we are receiving right now
volatile uint8_t values[NUMLEDS]; //array holding brightness values

uint8_t rxstate;

//addressing arrays
//arrays hold DDRB and PORTB values need to drive the charlieplexed LED array
uint8_t IOdirection[NUMLEDS];
uint8_t IOvalue[NUMLEDS];

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
	
	
	//populate our register value tables dynamically here so code is flexible
	uint8_t count = 0; 
	for(int i=0; count<NUMLEDS; i++){
          temp = (1<<i);
          for(int y=0; count<NUMLEDS && y<7; y++){
			temp = (temp<<1) | (temp>>7); 
			IOdirection[count] = (1<<(i)) | temp;
            IOvalue[count++] = (1<<(i));
          }
     }

	
	//configure timers
	//using timer 0 for plexing LEDs
	//TCCR0A = 0;  //not using output pin
	//TCCR0B = (1<<CS01); //timer clock div/8
	
	//TIMSK = (1<<OCIE0A) | (1<<TOIE0); //compare match A interrupt and overflow interrupt enabled
	
	
	TCCR1A = 0;
	TCCR1B = (1<<CS11); //timer clock div/8
	TIMSK = (1<<OCIE1A)|(1<<OCIE1B); //compare match A and B on timer 1
	OCR1B = (0xFF<<UPSCALE);
	
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
	
	
	//enable interrupts
	sei();
	
    while(1)
    {
        //TODO:: Please write your application code 
		//while ( !( UCSRA & (1<<UDRE)) ) ;							
		//	UDR = 0x7E;
		//_delay_ms(500);	
		//Notes
		//Run my timer based charle code. 
		//service the serial port and act on each byte. 
		
		//Since we will have a high number of channels to drive, and alot of work to do servicing the serial port
		// in conjunction with Charlieplexing. 
		
		//LEDs are all wired on port B. 
		
		//Timer 0 is 8 bit. Sys clock will be 16Mhz. We will run with timer ad div/8 for now 
    }
}

//called when we have a byte come in
ISR(USART_RX_vect) {

 //grab byte
 temp = UDR;
 
 //(re)enable interrupts so PWM code takes priority
 //sei();
 
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
			RXLED = 0;
			
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
		else 
			values[RXLED++] = temp;	
			if(RXLED == NUMLEDS){
				rxstate = 4;
				while ( !( UCSRA & (1<<UDRE)) ) ;							
				UDR = 0x80;		
			}				
	break;
	case 3: //peel off escaped byte				
		values[RXLED++] = temp + 0x4E;
		if(RXLED == NUMLEDS){
			rxstate = 4;
			while ( !( UCSRA & (1<<UDRE)) ) ;							
			UDR = 0x80;	
		}			
		else
			rxstate = 2;
	break;
	case 4: //forward rest
		if(temp == 0x7E) 
			rxstate = 1;
		while ( !( UCSRA & (1<<UDRE)) ) ;							
		UDR = temp;	
	break;
	 
 	 
	}    
}


//turn off on A
ISR(TIMER1_COMPA_vect) {
	
	//turn off LED
	DDRB=0;
	
	
}


//restart for next LED on B
ISR(TIMER1_COMPB_vect){
	
	//turn on next LED
	LED++;
	if(LED>NUMLEDS) LED=0;
	
	//return if LED PWM value is 0
	if(values[LED]==0) {
		//clear count
		TCNT1 = 0;
		return;
	}		
	
	//stop timer
	TCCR1B = 0;
	
	//upscale
	longtemp = ((uint16_t)values[LED]<<UPSCALE);
	
	//load compare value
	OCR1A = longtemp; //spec sheet says we can do this. I'm doubtful
	 
	//clear count
	TCNT1 = 0;
	
	//set direction and IO register for this LED
	DDRB=IOdirection[LED];
	PORTB=IOvalue[LED];
	
	//clear interrupt, clear prescailer, start timer
	TIFR = (1<<OCF1A);
	GTCCR = (1<<PSR10);
	TCCR1B = (1<<CS01);
	
}

/*
//timer compare match interrupt. Turn off led on compare
ISR(TIMER0_COMPA_vect){

	//turn off LED
	DDRB=0;

}

//timer overflow interrupt. Turn on next LED on overflow
ISR(TIMER0_OVF_vect){
	
	//turn on next LED
	LED++;
	if(LED>NUMLEDS) LED=0;
	
	//return if LED PWM value is 0
	if(values[LED]==0) return;
	
	//stop timer
	TCCR0B = 0;
	
	//load compare value
	OCR0A = values[LED];
	
	//clear count
	TCNT0 = 0;
	
	//set direction and IO register for this LED
	DDRB=IOdirection[LED];
	PORTB=IOvalue[LED];
	
	//clear interrupt, clear prescailer, start timer
	TIFR = (1<<OCF0A);
	GTCCR = (1<<PSR10);
	TCCR0B = (1<<CS01);
	
}*/