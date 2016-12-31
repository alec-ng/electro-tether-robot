#include <stdio.h>
#include <at89lp51rd2.h>

// ~C51~ 
 
#define CLK 22118400L
#define BAUD 115200L
#define BRG_VAL (0x100-(CLK/(32L*BAUD)))

#define MOVE_CLOSER 0B_0101_0101
#define MOVE_FURTHER 0B_1010_1010
#define ONE_EIGHTY 0B_0011_0011
#define PARALLEL_PARK 0B_1100_1100
#define THREE_POINT_TURN 0B_0110_0110
#define FIGURE_EIGHT 0B_1001_1001

//We want timer 0 to interrupt every 30 microseconds ((1/30000Hz)=30)
#define FREQ 30000L
#define TIMER0_RELOAD_VALUE (65536L-(CLK/(12L*FREQ)))

//These variables are used in the ISR
int onOff;

unsigned char _c51_external_startup(void)
{
	// Configure ports as a bidirectional with internal pull-ups.
	P0M0=0;	P0M1=0B_0100_0001;
	P1M0=0;	P1M1=0;
	P2M0=0;	P2M1=0;
	P3M0=0;	P3M1=0;
	AUXR=0B_0001_0001; // 1152 bytes of internal XDATA, P4.4 is a general purpose I/O
	P4M0=0;	P4M1=0;
    
    // Initialize the serial port and baud rate generator
    PCON|=0x80;
	SCON = 0x52;
    BDRCON=0;
    BRL=BRG_VAL;
    BDRCON=BRR|TBCK|RBCK|SPD;
	
	// Initialize timer 0 for ISR 'pwmcounter()' below
	TR0=0; // Stop timer 0
	TMOD=0x01; // 16-bit timer
	// Use the autoreload feature available in the AT89LP51RB2
	// WARNING: There was an error in at89lp51rd2.h that prevents the
	// autoreload feature to work.  Please download a newer at89lp51rd2.h
	// file and copy it to the crosside\call51\include folder.
	TH0=RH0=TIMER0_RELOAD_VALUE/0x100;
	TL0=RL0=TIMER0_RELOAD_VALUE%0x100;
	TR0=1; // Start timer 0 (bit 4 in TCON)
	ET0=1; // Enable timer 0 interrupt
	EA=1;  // Enable global interrupts
	
	onOff = 0;
    
    return 0;
}

void waitOneBit (int val)
{
	if (val == 0){
		_asm	
		;For a 22.1184MHz crystal one machine cycle 
		;takes 12/22.1184MHz=0.5425347us
		mov R2, #2
	L2:	mov R1, #150
	L3:	mov R0, #150
	L4:	djnz R0, L4 ; 2 machine cycles-> 2*0.5425347us*184=200us
	    djnz R1, L3 ; 200us*250=0.05s
	    djnz R2, L2
	    ret
    _endasm;
    }
    else if (val == 1){
   		_asm	
		;For a 22.1184MHz crystal one machine cycle 
		;takes 12/22.1184MHz=0.5425347us
		mov R1, #150
	B1:	mov R0, #150
	B2:	djnz R0, B2 ; 2 machine cycles-> 2*0.5425347us*184=200us
	    djnz R1, B1 ; 200us*250=0.05s
	    ret
    _endasm;
    }
}

void sendMessage(char dat){
	int i;
	EA = 0;
	waitOneBit(0);
	for(i = 0; i < 8; i++){
		EA = dat >> 7;
		dat = dat << 1;
		waitOneBit(EA);
	}
	EA = 0;
	waitOneBit(0);
	waitOneBit(0);
	EA = 1;
	return;
}


// Interrupt 1 is for timer 0.  This function is executed every time
// timer 0 overflows: 100 us.
void outputFreq (void) interrupt 1
{
	if (onOff == 0){
		onOff = 1;
		P0_0 = 1;
		P0_6 = 0;
	}
	else if (onOff == 1){
		onOff = 0;
		P0_0 = 0;
		P0_6 = 1;
	}
}

void main (void)
{	

	while(1){
		if (P1_4 == 1)
			sendMessage(MOVE_CLOSER);
		else if (P1_5 == 1)
			sendMessage(MOVE_FURTHER);
		else if (P1_6 == 1)
			sendMessage(ONE_EIGHTY);
		else if (P1_7 == 1)
			sendMessage(PARALLEL_PARK);
		else if (P4_1 == 1)
			sendMessage(THREE_POINT_TURN);
		else if (P3_2 == 1)
			sendMessage(FIGURE_EIGHT);
	}
}
