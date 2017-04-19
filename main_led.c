/**********************************************************************
* Programmer: Paul-Daniel Mayzeles
* Date: Nov 29 2011
* FileName:        main_led.c
* Dependencies:    p24HJ64GP502.h
* Processor:       PIC24H
* Compiler:        MPLAB® C30 v2.01 or higher
*
*Some functions used in this program were modified from the very basic "Flash LED" program distributed by microchip. 
*these functions have been modified from their original states. 
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/*
*displayer stores the values of LED states for a single layer. (16 bits) the high bit of displayer
*corresponds to the #0 LED in the layer matrix.
*
* LED MATRIX
* 0  1  2  3     ---> 
* 4  5  6  7     ---> wires leading to devboard 
* 8  9  10 11    --->
* 12 13 14 15    ---> 
*							
*          0x  0          0           0            0            (hex digits) 
*displayer     0 0 0 0    0 0 0 0     0 0 0  0     0  0  0  0   (binary digits)  
*  LED #       0 1 2 3    4 5 6 7     8 9 10 11    12 13 14 15
*
*
*
*displaya[] is an array that stores an entire state of the cube. Essentially display[a] stores 4 values of displayer,
*where  
*
*displaya[] = { 0x0000,0x0000, 0x0000, 0x0000 } 
*               LAYER0 LAYER1  LAYER2  LAYER3
*
*
**********************************************************************

*/
#include "p24hxxxx.h"						/* generic header for PIC24H family */


/******************************* 

Set device configuration values 

********************************/
#ifdef __PIC24HJ64GP502__
_FOSCSEL(FNOSC_FRC);								/* set oscillator mode for FRC ~ 8 Mhz (111 -- internal FRC) (7.37MHZ)*/
_FOSC(FCKSM_CSDCMD & OSCIOFNC_ON & POSCMD_NONE);	/* clock switching disabled, clock monitor disabled/ use OSCIO pin for RA3 / primary oscillator mode : disabled*/
_FWDT(FWDTEN_OFF);									/* turn off watchdog*/
#elif defined(__dsPIC33FJ64MC802__)
_FOSCSEL(FNOSC_FRC);								
_FOSC(FCKSM_CSDCMD & OSCIOFNC_ON & POSCMD_NONE);
_FWDT(FWDTEN_OFF);									
#endif


#define PERIOD  0x09c						/* sets the default interval flash rate 0x9c4 = 2500 (refresh time) (0xFFFF ~ 61hz)*/
											/*0x09c means the timer is running */
#define FLASH_RATE 100						/* smaller value yields faster rate (1600 ~ 62.5 clocks/min ~ 1Hz (1.05))*/
#define FOREVER 1							/* endless */

#define ClkOut LATAbits.LATA0 				/*ClkOut connected to CLK pin on STP16CP05*/
#define SDout LATAbits.LATA1 				/* Connected to SDIN on STP16CP05*/
#define Latch LATAbits.LATA4                /*Connected to LE/DM1 on STP16CP05*/
#define LED PORTBbits.RB15                  /*connected to the Microstick programmer's user LED. */
#define LED1 LATBbits.LATB15			    /*same as above*/

#define Lyr0 LATBbits.LATB6					/*Controls the pin connected to the base of transistor control for layerN*/
#define Lyr1 LATBbits.LATB7					/*activating any one of these sends power to that layer of the LED cube. */
#define Lyr2 LATBbits.LATB8
#define Lyr3 LATBbits.LATB9




/* function prototypes */
void InitTimer1();							
void wait(); 	
void eraseCube(unsigned int *displaya);
void fillCube(unsigned int *displaya);	
void displayPlus(unsigned int *diaplaya);
void centerCube(unsigned int *diaplaya);
/* globals */
unsigned int Counter = 0;

volatile unsigned int timer_expired;


/********************************* 

	main entry point

*********************************/

int main ( void )
{
 	int layer = 0;
	int j = 0;
	int k = 0; 
	unsigned int latchCounter = 0;	    /*latch counter counts the number of half clock cycles before latch is activated and the 16 serial bits are output on the shift register */ 
	unsigned int displaya[] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};  /*displaya is an array that stores a single entire cube state. 8 bytes = 4 layers. */ 
									    /*for displaya[], */ 
	unsigned int clkCounter = 0; 		/*clkCounter counts the number of clock state switches to time the latch*/
	unsigned int displayer = 0xFFFF; 	/*displayer stores the values of LED states for a single layer. (16 bits) the high bit of displayer*/
										/*corresponds to the #0 LED in the layer matrix.*/
	unsigned int animCounter = 0;		/*this counts the number of repetitions of animation states, to loop animations.*/
	unsigned int  REPEAT_STATE = 3445;  /*repeat state repeats the state n number of times, in order to display a cube state for a certain amount of time. */
										/*25640 = 61 BPM or ~ 1 cube state transition per second. */
										/*94994 = 226bpm */
										/* use formula x = (226*25640)/61 where x is desired state switch rate. */
	unsigned int repeCounter; 			/*repeCounter counts the number of times a specific state is repeated, using repeat_state a a maximum value.*/
	unsigned int displed;				/*displed determines whether a single specific LED is lit during the serial data output.*/
	unsigned int nextAnimCounter = 0;  
	unsigned int measure = 16;          /*measure is the size of a musical measure, in 1/16th notes. */ 

    
	/* 	Initialize ports */
	LATA  = 0x0000; 				/* set latch levels*/
	TRISA = 0x0000; 				/* set IO as outputs*/
	LATB  = 0x0000; 				/* set latch levels*/
	TRISB = 0x0000; 				/* set IO as outputs*/
	
	InitTimer1();					/* start the timer, configure timer bits*/
	Lyr0 = 1;						/*activate the first layer. Mainly used for debugging, has no effect when running at full speed. */
	ClkOut = 1;  					/*set the clock high. (necessary for allowing serial data to be transmitted on rising edge of clock) */

	

/*  endless loop*/
	while (FOREVER)
	{			
		if (timer_expired == 1)									
		{	
			clkCounter++;	        /*increment the clock, latch, and animation repeat counters. */ 
			latchCounter++; 
			repeCounter++;

			

			if(repeCounter >= REPEAT_STATE && layer == 0) /*this is where animations can begin to be built. */ 
			{											  /*documentation for individual animations can be found at the end of this file */ 

				
				if(nextAnimCounter <= (measure*2)) /*falling sheet animation */ 
				{
					for(j = 0; j < 3; j++){
						displaya[j] = displaya[j+1];
						displaya[j+1] = 0x0000; 
					}
				
					if(animCounter == 4)
					{
						displaya[3] = 0xFFFF; 
						animCounter = 0; 
					}
					animCounter++;					

				} 
				if(nextAnimCounter > (measure*2) && nextAnimCounter <= (measure*8) ) 
				{   /*this is similar to the "flash cube" but has four states which appear to make the cube "explode" */ 
					switch(k % 4){
						case 0: 
							fillCube(displaya);
							k++;
							break;
					    case 1:
							displayPlus(displaya);
							k++;
							break;	
						case 2:
							centerCube(displaya);
							k++;
							break;
						case 3:
							eraseCube(displaya);
							k++; 
							break;
						default:
							k = 0; 
							break;
					}
				}else  /* this animates a "flashing cube" display */ 
					if(nextAnimCounter > (measure*8) && nextAnimCounter <= measure*14)
					{
						if(animCounter %2)
						{	
							fillCube(displaya);
							animCounter =0; 

						}else
						{
							eraseCube(displaya);
							animCounter =1;	
						}
				
					}else  /*this animates a series of 4 4-led cubes that "rotate" about themselves. */ 
					if(nextAnimCounter > measure*14 && nextAnimCounter <= (measure*22)){
						switch(animCounter){
						case 0:
							displaya[0] = 0x33cc;
							displaya[1] = 0x33cc;
							displaya[2] = 0xcc33;
							displaya[3] = 0xcc33;
							animCounter++;
							break;
						case 1:
							displaya[0] = 0x6666;
							displaya[1] = 0x6666;
							displaya[2] = 0x6666;
							displaya[3] = 0x6666;
							animCounter++;
							break;
						case 2:
							displaya[0] = 0xcc33;
							displaya[1] = 0xcc33;
							displaya[2] = 0x33cc;
							displaya[3] = 0x33cc;
							animCounter++;
							break;
						
						case 3: 
							displaya[0] = 0x0000;
							displaya[1] = 0xffff;
							displaya[2] = 0xffff;
							displaya[3] = 0x0000;
							animCounter = 0;
							break;
						default:
							break;  
						}
					
							 

					}else if(nextAnimCounter > (measure*22) && nextAnimCounter <= (measure*30))
					{	/* this animation is a box in the center alternating with squares on each edge, in order */	
						switch(animCounter){
						case 0:
							displaya[0] = 0x000f;
							displaya[1] = 0x0009;
							displaya[2] = 0x0009;
							displaya[3] = 0x000f;
							animCounter++;
							break;
						case 1: 
							centerCube(displaya);
							animCounter++;
							break;	
						case 2:
							displaya[0] = 0x8888;
							displaya[1] = 0x8008;
							displaya[2] = 0x8008;
							displaya[3] = 0x8888;							

							animCounter++;
							break;
						case 3: 
							centerCube(displaya);
							animCounter++;
							break;
						case 4:
							displaya[0] = 0xf000;
							displaya[1] = 0x9000;
							displaya[2] = 0x9000;
							displaya[3] = 0xf000;

							animCounter++;
							break;
						case 5: 
							centerCube(displaya);
							animCounter++;
							break;						
						case 6: 
							displaya[0] = 0x1111;
							displaya[1] = 0x1001;
							displaya[2] = 0x1001;
							displaya[3] = 0x1111;
							animCounter++;
							break;
						case 7: 
							centerCube(displaya);
							animCounter = 0;
							break;
						default:
							break;  
						}						 
					
					}else if(nextAnimCounter > measure*30)
						nextAnimCounter = 29;
			

				nextAnimCounter++;   /*animcounter counts the time until the next animation is displayed */ 
				repeCounter = 0;
			}
			
			
			

			if (Latch ==1)  /*we reset the latch on the next clock tick */ 
			{//	LED1 = 0;
				Latch = 0; 
			}

			if (ClkOut == 1) /*we check to see if the clock is rising in order to perform the next serial output. */ 
			{				 /*if the clock is falling, we do not register the serial output, and if we do not */
							/*distinguish at all, then we double the rate at which serial data is transferred and */
							/*obtain erratic results. */ 
					displed = displayer % 2;
					displayer = displayer / 2; 		
					
					if(displed == 1)
						SDout = 1; 
					else if (displed == 0)
						SDout = 0; 
			}	
			


			/*Lyr0 = ~Lyr0; */ 
			ClkOut = ~ClkOut;   /* toggle the clock signal. this is done once per timer interrupt. */
		
			if(latchCounter == 33)  /* 34 half-cycles is 17 "clock" cycles */
			{
				Latch = 1;			/*activate the latch to display the state of the cube array */ 
				latchCounter = 0;   /*reset the latch counter to prepare for another 16 bits of data to the shift register. */ 
			 
				
				switch(layer){      /* this switch activates one layer at a time, in the order l0, l1, l2, l3, then back to l0.*/ 
					case 0:
						Lyr3 = 0;
						Lyr0 = 1;
					break;
					case 1:
						Lyr0 = 0;
						Lyr1 = 1;
					break;
					case 2:
						Lyr1 = 0;
						Lyr2 = 1;
					break;
					case 3:
						Lyr2 = 0;
						Lyr3 = 1;
					break;
					default:
					break; 
				} 

				if(layer < 3)		/* 4 layer LED cube */ 
					layer ++;       /*increment the layer that we wish to display. */ 
				else 
					layer = 0;     /*if we're at layer 3, reset to layer 0 to finish a single display state. */ 
					
				
				displayer = displaya[layer]; 
				  
			}
			timer_expired = 0;    /*finally, reset the timer interrupt so we can start a new clock cycle. */ 
			Counter = 0; 
		}
	
	}
	
}
/*---------------------------------------------------------------------
  Function Name: wait
  Description:   waits a small time (half a ms or so) for an instruction to execute so
				 that changes made to the ports can register on the clock cycle. 
  Inputs:        None
  Returns:       None
-----------------------------------------------------------------------*/

void wait( void ) 
{
	int i;
	for (i=0; i < 10000; i++ )
	{
		Nop();
	}
	
} 


/*---------------------------------------------------------------------
  Function Name: InitTimer1
  Description:   Initialize Timer1 for 1 second intervals
  Inputs:        None
  Returns:       None
-----------------------------------------------------------------------*/
void InitTimer1( void )
{
	
	OSCTUN = 0x0016;				/* multiply internal FRC clock (approx 7.37mHz by 23*0.375% to get 8.00mHz. yields approx 5% faster than 8MHz. */ 
	
	T1CON = 0;						/* ensure Timer 1 is in reset state */
 	
	IFS0bits.T1IF = 0;				/* reset Timer 1 interrupt flag */
	IPC0bits.T1IP = 4;				/* set Timer1 interrupt priority level to 4 */
 	IEC0bits.T1IE = 1;				/* enable Timer 1 interrupt */
	
	PR1 = PERIOD;					/* set Timer 1 period register (2500d) */
	T1CONbits.TCKPS1 = 0x0;			/* select Timer1 Input Clock Prescale - 3(div 256, debug) 0(div 1) */ 
	T1CONbits.TCS = 0;			 	/* select external timer clock = FRC clock = fosc/2  = 4mhz*/
	T1CONbits.TON = 1;			 	/* enable Timer 1 and start the count */ 
	
}


/*---------------------------------------------------------------------
  Function Name: _T1Interrupt
  Description:   Timer1 Interrupt Handler
  Inputs:        None
  Returns:       None
-----------------------------------------------------------------------*/
void __attribute__((interrupt, auto_psv)) _T1Interrupt( void )
{

	
	timer_expired = 1;				/* flag */
	Counter++;						/* keep a running counter */
 	IFS0bits.T1IF = 0;				/* reset timer interrupt flag	*/

}	
	



/* ALTERNATING WEIRD flashy thing. 


				LED1 = ~LED1; 
				//load next displaya; 
					for(j = 0; j < 3; j++){
						displaya[j] = displaya[j+1];
					}
					if(animCounter % 2) 
						displaya[3] = 0x5a5a;
					else
						displaya[3] = 0xa5a5;
				
				animCounter++; 

				if(animCounter == 4)
				{
					displaya[0] = 0xA5A5;
					displaya[1] = 0x5A5A;
					displaya[2] = 0xA5A5;
					displaya[3] = 0x5A5A;
					animCounter = 0; 

				}
					
				repeCounter = 0;


*/ 




/*EVERY OTHER LED FLASHING 

				LED1 = ~LED1; 
				//load next displaya; 
					
				
				if(animCounter == 1)
				{
					displaya[0] = 0xA5A5;
					displaya[1] = 0x5A5A;
					displaya[2] = 0xA5A5;
					displaya[3] = 0x5A5A;
					animCounter =0;
				}else
				{
					displaya[0] = 0x5A5A;
					displaya[1] = 0xA5A5;
					displaya[2] = 0x5A5A;
					displaya[3] = 0xA5A5;
					animCounter =1; 
				}
				

*/ 

/* 4 box- alternating 


				if(animCounter == 1)
				{
					displaya[0] = 0x33cc;
					displaya[1] = 0x33cc;
					displaya[2] = 0xcc33;
					displaya[3] = 0xcc33;
					animCounter =0;
				}else
				{
					displaya[0] = 0xcc33;
					displaya[1] = 0xcc33;
					displaya[2] = 0x33cc;
					displaya[3] = 0x33cc;
					animCounter =1; 
				} 	

*/ 


/* falling sheet  (change animCounter to get "faster" fall: 
2 gives "alternating sheet"
3 gives "fast falling sheet"
4 gives "one falling sheet" 													

				
				for(j = 0; j < 3; j++){
						displaya[j] = displaya[j+1];
						displaya[j+1] = 0x0000; 
					}
				
				if(animCounter == 3)
				{
					displaya[3] = 0xFFFF; 
					animCounter = 0; 
				}
				animCounter++; */ 


/*

flashcube (all on, all off)

				if(animCounter == 1)
				{
					eraseCube(displaya);
					animCounter =0;
				}else
				{
					fillCube(displaya);
					animCounter =1; 
				}



*/

/* fill cube with 1s

*/	

void fillCube(unsigned int *displaya){
					displaya[0] = 0xffff;
					displaya[1] = 0xffff;
					displaya[2] = 0xffff;
					displaya[3] = 0xffff;
}


/*
fill cube with 0's
*/ 

void eraseCube(unsigned int *displaya){
					displaya[0] = 0x0000;
					displaya[1] = 0x0000;
					displaya[2] = 0x0000;
					displaya[3] = 0x0000;

}


/*
displays a "plus sign" in 3d
*/
void displayPlus(unsigned int *displaya){
					displaya[0] = 0x0660;
					displaya[1] = 0x6ff6;
					displaya[2] = 0x6ff6;
					displaya[3] = 0x0660;
}

/* displays only the centermost 4 leds */ 
void centerCube(unsigned int *displaya){

					displaya[0] = 0x0000;
					displaya[1] = 0x0660;
					displaya[2] = 0x0660;
					displaya[3] = 0x0000;

}
