#include <stdio.h>
#include <stdlib.h>
#include <at89lp51rd2.h>

// ~C51~ 

// definitions for motor components
#define DESIRED_VOLTAGE 1.2
#define MINIMUM 25 
#define SLOW_SPEED 50
#define MEDIUM_SPEED 75
#define TURBO_SPEED 100
#define FORWARD 1
#define BACKWARD 0
#define OFF 0
#define ON 1
#define NORMAL_ORIENTATION 1
#define REVERSE_ORIENTATION -1
#define LEFT_TURN_SIG P1_0
#define RIGHT_TURN_SIG P3_6


//initialization definitions
#define CLK 22118400L
#define BAUD 115200L
#define BRG_VAL (0x100-(CLK/(32L*BAUD)))

//timer 0 will interrupt every 100ms
#define FREQ 10000L
#define TIMER0_RELOAD_VALUE (65536L-(CLK/(12L*FREQ)))

//command definitions
#define MOVE_CLOSER 0B_0101_0101
#define MOVE_FURTHER 0B_1010_1010
#define ONE_EIGHTY 0B_0011_0011
#define PARALLEL_PARK 0B_1100_1100
#define THREE_POINT_TURN 0B_0110_0110
#define FIGURE_EIGHT 0B_1001_1001

//volatile variables for the interrupt
//Names are based on Normal Orientation
volatile int pwmcount;
volatile int LeftMotor_Forward;		
volatile int LeftMotor_Backward;	
volatile int RightMotor_Forward;	
volatile int RightMotor_Backward;

/**
====================================================================
							Initialization
====================================================================
**/


unsigned char _c51_external_startup(void)
{
	// Configure ports as a bidirectional with internal pull-ups.
	P0M0=0;	P0M1=0;
	P1M0=0;	P1M1=0;
	P2M0=0;	P2M1=0;
	P3M0=0;	P3M1=0;
	AUXR=0B_0001_0001; // 1152 bytes of internal XDATA, P4.4 is a general purpose I/O
	P4M0=0;	P4M1=0;
    
    //User built-in baud rate generator instead of timer to generate the clock
    //for the serial port
 
    PCON|=0x80;
	SCON = 0x52;
    BDRCON=0;
    BRL=BRG_VAL;
    BDRCON=BRR|TBCK|RBCK|SPD;
    
    // Initialize timer 0 for ISR 'pwmcounter()' below
	TR0=0; // Stop timer 0
	TMOD=0B_00010001; // 16-bit timer for timer 0 and 1
	// Use the autoreload feature available in the AT89LP51RB2
	TH0=RH0=TIMER0_RELOAD_VALUE/0x100;
	TL0=RL0=TIMER0_RELOAD_VALUE%0x100;
	TR0=1; // Start timer 0 (bit 4 in TCON)
	ET0=1; // Enable timer 0 interrupt
	ET0 = 0; // Disable timer 0 -Only set it when we want it to move
	EA=1;  
    pwmcount=0;
    RIGHT_TURN_SIG = OFF;
    LEFT_TURN_SIG = OFF;
    
    return 0;
}


/**
====================================================================
					SPI Functions
====================================================================
**/


void SPIWrite(unsigned char value)
{
	SPSTA&=(~SPIF); // Clear the SPIF flag in SPSTA
	SPDAT=value;
	while((SPSTA & SPIF)!=SPIF); //Wait for transmission to end
}

// Read 10 bits from the MCP3004 ADC converter
unsigned int GetADC(unsigned char channel)
{
	unsigned int adc;

	// initialize the SPI port to read the MCP3004 ADC attached to it.
	SPCON&=(~SPEN); // Disable SPI
	SPCON=MSTR|CPOL|CPHA|SPR1|SPR0|SSDIS;
	SPCON|=SPEN; // Enable SPI
	
	P1_4=0; // Activate the MCP3004 ADC.
	SPIWrite(channel|0x18);	// Send start bit, single/diff* bit, D2, D1, and D0 bits.
	for(adc=0; adc<10; adc++); // Wait for S/H to setup
	SPIWrite(0x55); // Read bits 9 down to 4
	adc=((SPDAT&0x3f)*0x100);
	SPIWrite(0x55);// Read bits 3 down to 0
	P1_4=1; // Deactivate the MCP3004 ADC.
	adc+=(SPDAT&0xf0); // SPDR contains the low part of the result. 
	adc>>=4;
		
	return adc;
}

float voltage (unsigned char channel)
{
	return ( (GetADC(channel)*4.84)/1023.0 ); // VCC=4.84V (measured)	
}


/**
====================================================================
					Bit Bang Functions
====================================================================
**/

void wait_one_and_half_bit_time(){
			_asm	
		mov R2, #3
	N2:	mov R1, #150
	N3:	mov R0, #150
	N4:	djnz R0, N4
	    djnz R1, N3
	    djnz R2, N2
	    ret
    _endasm;
}


void wait_bit_time (){

		_asm	
		mov R2, #2
	Q2:	mov R1, #150
	Q3:	mov R0, #150
	Q4:	djnz R0, Q4
	    djnz R1, Q3
	    djnz R2, Q2
	    ret
    _endasm;

}

unsigned char rx_byte ( int min ){
	unsigned char j, val;
	int v;
		
	val=0;
	wait_one_and_half_bit_time();
	
	for(j=0; j<8; j++)
	{
		v=GetADC(0);
		val|=(v>min)?(0x01<<j):0x00;
		wait_bit_time();
	}

	wait_one_and_half_bit_time();
	return val;
}


/**
====================================================================
						Different Delays
====================================================================
**/
// For a 22.1184MHz crystal one machine cycle takes 12/22.1184MHz=0.5425347us

void startUpDelay(){
	_asm	
	
	    mov R2, #40
	A3:	mov R1, #248
	A2:	mov R0, #184
	A1:	djnz R0, A1 
	    djnz R1, A2 
	    djnz R2, A3 
	    ret
    _endasm;
}

void rotateDelay(){
	_asm	
	
	    mov R2, #16
	H3:	mov R1, #232
	H2:	mov R0, #184
	H1:	djnz R0, H1 
	    djnz R1, H2 
	    djnz R2, H3 
	    ret
    _endasm;
}

void SlightDelay(){
	_asm	
	    mov R2, #2
	E3:	mov R1, #248
	E2:	mov R0, #184
	E1:	djnz R0, E1 
	    djnz R1, E2 
	    djnz R2, E3 
	    ret
    _endasm;
}


void motorDelay(){
	_asm	
	    mov R2, #2
	L3:	mov R1, #248
	L2:	mov R0, #184
	L1:	djnz R0, L1 
	    djnz R1, L2 
	    djnz R2, L3 
	    ret
    _endasm;
}

void motorsDelay(){
	_asm	
	   mov R2, #4
	M3:	mov R1, #248
	M2:	mov R0, #184
	M1:	djnz R0, M1 
	    djnz R1, M2 
	    djnz R2, M3 
	    ret
    _endasm;
}

void fortyFive(){
	_asm	
	    mov R2, #5
	K3:	mov R1, #253
	K2:	mov R0, #184
	K1:	djnz R0, K1 
	    djnz R1, K2 
	    djnz R2, K3
	    ret
    _endasm;
}

void distDelay(){
	_asm	
	    mov R2, #27
	Y3:	mov R1, #250
	Y2:	mov R0, #184
	Y1:	djnz R0, Y1 
	    djnz R1, Y2 
	    djnz R2, Y3 
	    ret
    _endasm;
}

void threeQuartSec(){
	_asm	
	    mov R2, #5
	W3:	mov R1, #248
	W2:	mov R0, #184
	W1:	djnz R0, W1
	    djnz R1, W2 
	    djnz R2, W3 
	    ret
    _endasm;
}

void arcTurnDelay(){
	_asm	
	    mov R2, #22
	O3:	mov R1, #241
	O2:	mov R0, #184
	O1:	djnz R0, O1 
	    djnz R1, O2
	    djnz R2, O3
	    ret
    _endasm;
}

void threePtBackDelay(){
	_asm	
	    mov R2, #21
	Z3:	mov R1, #248
	Z2:	mov R0, #184
	Z1:	djnz R0, Z1 
	    djnz R1, Z2
	    djnz R2, Z3 
	    ret
    _endasm;
}


/**
====================================================================
					Motor Initialization Commands
====================================================================
**/

void init_backwards(int orientation){
	if(orientation == NORMAL_ORIENTATION){
		LeftMotor_Forward = OFF;			RightMotor_Forward = OFF;
		LeftMotor_Backward = MEDIUM_SPEED; 	RightMotor_Backward = MEDIUM_SPEED;		
	}
	if(orientation == REVERSE_ORIENTATION){
		LeftMotor_Forward = MEDIUM_SPEED;	RightMotor_Forward = MEDIUM_SPEED;
		LeftMotor_Backward = OFF; 			RightMotor_Backward = OFF;
	}
}

void init_Fastbackwards(int orientation){
	if(orientation == NORMAL_ORIENTATION){
		LeftMotor_Forward = OFF;			RightMotor_Forward = OFF;
		LeftMotor_Backward = TURBO_SPEED; 	RightMotor_Backward = TURBO_SPEED;		
	}
	if(orientation == REVERSE_ORIENTATION){
		LeftMotor_Forward = TURBO_SPEED;	RightMotor_Forward = TURBO_SPEED;
		LeftMotor_Backward = OFF; 			RightMotor_Backward = OFF;
	}
}

void init_forwards(int orientation){
	if(orientation == NORMAL_ORIENTATION){
		LeftMotor_Forward = MEDIUM_SPEED;	RightMotor_Forward = MEDIUM_SPEED;
		LeftMotor_Backward = OFF; 			RightMotor_Backward = OFF;		
	}
	if(orientation == REVERSE_ORIENTATION){
		LeftMotor_Forward = OFF;			RightMotor_Forward = OFF;
		LeftMotor_Backward = MEDIUM_SPEED; 	RightMotor_Backward = MEDIUM_SPEED;
	}
}

void init_45_arcTurn(int orientation){
	if(orientation == NORMAL_ORIENTATION){
		LeftMotor_Forward = SLOW_SPEED;			RightMotor_Forward = TURBO_SPEED;
		LeftMotor_Backward = OFF;				RightMotor_Backward = OFF;	
	}
	if(orientation == REVERSE_ORIENTATION){
		LeftMotor_Forward = OFF;				RightMotor_Forward = OFF;
		LeftMotor_Backward = TURBO_SPEED; 		RightMotor_Backward = SLOW_SPEED;
	}
}

void init_180(){
	LeftMotor_Forward = MEDIUM_SPEED;		RightMotor_Forward = OFF;
	RightMotor_Backward = MEDIUM_SPEED;		LeftMotor_Backward = OFF;
}

void init_45_CW(){
	LeftMotor_Forward = MEDIUM_SPEED;		RightMotor_Forward = OFF;
	LeftMotor_Backward = OFF; 				RightMotor_Backward = MEDIUM_SPEED;
}

void init_45_CCW(){
	LeftMotor_Forward = OFF;				RightMotor_Forward = MEDIUM_SPEED;
	LeftMotor_Backward = MEDIUM_SPEED;  	RightMotor_Backward = OFF;
}

void init_figEight_14(){
	LeftMotor_Forward = SLOW_SPEED;			RightMotor_Forward = TURBO_SPEED;
	LeftMotor_Backward = OFF; 				RightMotor_Backward = OFF;	
}

void init_figEight_23(){
	LeftMotor_Forward = TURBO_SPEED;		RightMotor_Forward = SLOW_SPEED;
	LeftMotor_Backward = OFF; 				RightMotor_Backward = OFF;	
}

void init_Stop(){
	P0_2 = 0;							P0_3 = 0;
	P1_3 = 0;							P1_2 = 0;
}

/**
====================================================================
						Different Commands
====================================================================
**/

float move_closer( float distance ){                                                     
	distance = distance * (2.0/3.0);
	
	return distance;
}


float move_further( float distance ){                                                  
	
	distance = distance * (3.0/2.0);
	
	return distance;
}

int rotate_180(int orientation){

	init_180(); 
	ET0 = ON;	rotateDelay(); 	ET0 = OFF;
	init_Stop();   
	
	orientation *= -1;
	return orientation;
}

void parallel_park(int orientation){

	unsigned char receivedData = 5;
	float receive;
	unsigned int min = MINIMUM;								
	
	init_45_CCW();
	ET0 = ON;	fortyFive();   	ET0 = OFF;	init_Stop();	SlightDelay();	
	
	if(orientation == NORMAL_ORIENTATION)
		RIGHT_TURN_SIG = ON;
	else
		LEFT_TURN_SIG = ON;
		
	init_backwards(orientation);
	ET0 = ON;	distDelay();	ET0 = OFF;	init_Stop();	SlightDelay();
	LEFT_TURN_SIG = OFF;		RIGHT_TURN_SIG = OFF;
	
	init_45_CW();
	ET0 = ON;	fortyFive();	ET0 = OFF;	init_Stop();	SlightDelay();
	
	
	while(1){
		receive = GetADC(0);
			if( receive <= MINIMUM ){
				receivedData = rx_byte(min);
				if (receivedData == PARALLEL_PARK)
					break;
			}
	}
	

	init_45_CCW();
	ET0 = ON;	fortyFive();	ET0 = OFF;	init_Stop();	SlightDelay();
	
	if(orientation == NORMAL_ORIENTATION)
		LEFT_TURN_SIG = ON;
	else
		RIGHT_TURN_SIG = ON;
	
	init_forwards(orientation);	
	ET0 = ON;	distDelay();	ET0 = OFF;	init_Stop();	SlightDelay();
	LEFT_TURN_SIG = OFF;		RIGHT_TURN_SIG = OFF;
	
	init_45_CW();
	ET0 = ON;	fortyFive();	ET0 = OFF;	init_Stop();	SlightDelay();
}

int three_point_turn(int orientation){

	
	init_forwards(orientation);
	ET0 = ON;	threeQuartSec();	ET0 = OFF;	init_Stop();	
			
	init_45_arcTurn(orientation);
	
	if(orientation == NORMAL_ORIENTATION)
		LEFT_TURN_SIG = ON;
	else
		RIGHT_TURN_SIG = ON;
		
	ET0 = ON;	arcTurnDelay();		ET0 = OFF;	init_Stop();	SlightDelay();
	LEFT_TURN_SIG = OFF;		RIGHT_TURN_SIG = OFF;
	
	
	init_Fastbackwards(orientation);
	ET0 = ON;	threePtBackDelay();	ET0 = OFF;	init_Stop();	SlightDelay();
	
	init_45_arcTurn(orientation);
	
	if(orientation == NORMAL_ORIENTATION)
		LEFT_TURN_SIG = ON;
	else
		RIGHT_TURN_SIG = ON;
		
	ET0 = ON;	arcTurnDelay();		ET0 = OFF;	init_Stop();
	LEFT_TURN_SIG = OFF;			RIGHT_TURN_SIG = OFF;
	
	init_forwards(orientation);
	ET0 = ON;	threeQuartSec();	ET0 = OFF;	init_Stop();
	
	orientation *= -1;
	return orientation;
}

void figure_Eight(){

	init_figEight_14();
	ET0 = ON;	LEFT_TURN_SIG = ON;
	arcTurnDelay();		arcTurnDelay();	
	ET0 = OFF;	LEFT_TURN_SIG = OFF;

	init_figEight_23();
	ET0 = ON;	RIGHT_TURN_SIG = ON;
	arcTurnDelay(); 	arcTurnDelay();		arcTurnDelay(); 	arcTurnDelay();				
	ET0 = OFF;	RIGHT_TURN_SIG = OFF;

	init_figEight_14();
	ET0 = ON;	LEFT_TURN_SIG = ON;
	arcTurnDelay();		arcTurnDelay();	
	ET0 = OFF;	LEFT_TURN_SIG = OFF;
	
	init_Stop();
}


	float distance = DESIRED_VOLTAGE;	
	int pwm_val = TURBO_SPEED;		
	int min = MINIMUM;
	

/**
====================================================================
							Interrupt
====================================================================
**/

// timer 0 overflows: 100 us.
void pwmcounter (void) interrupt 1
{
	if(++pwmcount>99) pwmcount=0;
	P1_3=(LeftMotor_Forward>pwmcount)?1:0;
	P1_2=(LeftMotor_Backward>pwmcount)?1:0;
	P0_2=(RightMotor_Forward>pwmcount)?1:0; 
	P0_3=(RightMotor_Backward>pwmcount)?1:0; 
}


/**
====================================================================
						Adjust Distance Function
====================================================================
**/

void moveMotor1( int pwm_val, int direction ){                                    
	RightMotor_Forward = OFF;	
	RightMotor_Backward = OFF;

	if( direction == FORWARD){
		LeftMotor_Forward = pwm_val;
		LeftMotor_Backward = OFF;
	}
	if( direction == BACKWARD){
		LeftMotor_Forward = OFF;
		LeftMotor_Backward = pwm_val;
	}
	ET0 = ON;	
	motorDelay();
	ET0 = OFF;
	init_Stop();
}


void moveMotor2( int pwm_val, int direction ){                                   

	LeftMotor_Forward = OFF;	
	LeftMotor_Backward = OFF;

	if( direction == FORWARD){
		RightMotor_Forward = pwm_val;
		RightMotor_Backward = OFF;
	}
	if( direction == BACKWARD){
		RightMotor_Forward = OFF;
		RightMotor_Backward = pwm_val;
	}
	ET0 = ON;	
	motorDelay();
	ET0 = OFF;
	init_Stop();
}

void moveMotors( int pwm_val, int direction ){                                   

	if( direction == FORWARD){
		RightMotor_Forward = pwm_val;
		RightMotor_Backward = OFF;
		LeftMotor_Forward = pwm_val;
		LeftMotor_Backward = OFF;
	}
	if( direction == BACKWARD){
		RightMotor_Forward = OFF;
		RightMotor_Backward = pwm_val;
		LeftMotor_Forward = OFF;
		LeftMotor_Backward = pwm_val;
	}
	ET0 = ON;	
	motorsDelay();
	ET0 = OFF;
	init_Stop();
}

void adjust_Distance(float distance, int pwm_val, int orientation)
{		

	float receive;
	float bufferL = 0.15;
	float bufferT = 0.1;

	float distance0 = voltage(0);		
	float distance1 = voltage(1);

	float tempDistance = distance0-distance1;
	
		
	while( tempDistance < (-bufferL) ){
		
		receive = GetADC(0);

		if( receive <= MINIMUM ){
			break;
		}
		
		RightMotor_Forward = OFF;	
		RightMotor_Backward = OFF;
	
		if(orientation == NORMAL_ORIENTATION){                                                
			LeftMotor_Forward = pwm_val;
			LeftMotor_Backward = OFF;
		}
		else{
			LeftMotor_Forward = OFF;
			LeftMotor_Backward = pwm_val;
		}
		ET0 = ON;	
		
		distance0 = voltage(0);
		distance1 = voltage(1);
		
		tempDistance = distance0-distance1;
	}
	
	ET0 = OFF;
	init_Stop();	

	while( tempDistance > bufferL ){
		
				
		receive = GetADC(0);

		if( receive <= MINIMUM ){
			break;
		}
		
		LeftMotor_Forward = OFF;
		LeftMotor_Backward = OFF;
	
		if(orientation == NORMAL_ORIENTATION){                                                
			RightMotor_Forward = pwm_val;
			RightMotor_Backward = OFF;
		}
		else{
			RightMotor_Forward = OFF;
			RightMotor_Backward = pwm_val;
		}
		ET0 = ON;	
		
		distance0 = voltage(0);
		distance1 = voltage(1);
		
		tempDistance = distance0-distance1;
	}
	
	ET0 = OFF;
	init_Stop();
	
	while( distance0 > distance+bufferT){
	
				
		receive = GetADC(0);

		if( receive <= MINIMUM ){
			break;
		}
	
		if(orientation == NORMAL_ORIENTATION){                                                
			RightMotor_Forward = OFF;
			RightMotor_Backward = pwm_val;
			LeftMotor_Forward = OFF;
			LeftMotor_Backward = pwm_val;
		}
		else{
			RightMotor_Forward = pwm_val;
			RightMotor_Backward = OFF;
			LeftMotor_Forward = pwm_val;
			LeftMotor_Backward = OFF;
		}
		ET0 = ON;
		
		distance0 = voltage(0);
		
		printf("distance0 is %u\n", distance0);
	}
	
	ET0 = OFF;
	init_Stop();
	
	while( distance0 < distance - bufferT ){
	
				
		receive = GetADC(0);

		if( receive <= MINIMUM ){
			break;
		}
	
		if(orientation == NORMAL_ORIENTATION){                                                
			RightMotor_Forward = pwm_val;
			RightMotor_Backward = OFF;
			LeftMotor_Forward = pwm_val;
			LeftMotor_Backward = OFF;
		}
		else{
			RightMotor_Forward = OFF;
			RightMotor_Backward = pwm_val;
			LeftMotor_Forward = OFF;
			LeftMotor_Backward = pwm_val;
		}
		ET0 = ON;
		
		distance0 = voltage(0);
		
		printf("distance0 is %u\n", distance0);
	}
	
	ET0 = OFF;
	init_Stop();
	
}


/**
====================================================================
							Main Funcion
====================================================================
**/

void main ()
{

	float distance = DESIRED_VOLTAGE;	
	int pwm_val = TURBO_SPEED;	
	int direction = FORWARD;				
	int orientation = NORMAL_ORIENTATION;	
	
	unsigned int min = MINIMUM;						
	unsigned char receivedData;
	
	startUpDelay();

	while(1)
	{
	
		float receive = GetADC(0);

		if( receive <= MINIMUM ){
			receivedData = rx_byte(min);
			
			
			if( receivedData == MOVE_CLOSER)                                                
			distance = move_closer(distance);
	
			else if (receivedData == MOVE_FURTHER)
			distance = move_further(distance);	
						
			else if (receivedData == ONE_EIGHTY)
				orientation = rotate_180(orientation);
			else if (receivedData == PARALLEL_PARK)
				 parallel_park(orientation);
			else if (receivedData == THREE_POINT_TURN)
				orientation = three_point_turn(orientation);
			else if (receivedData == FIGURE_EIGHT)
				figure_Eight();	
		}
	
		if (GetADC(2) < 400){
			P2_0 = 0;
			P2_1 = 0;
		}
		else{
			P2_0 = 1;
			P2_1 = 1;
		}
		
		adjust_Distance(distance, pwm_val, orientation);
	}
}