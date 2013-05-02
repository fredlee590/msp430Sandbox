/*
this main makes sure that we can use the intended method (reed switch to act as
input to use in clip detection. PIN and port interrupts
*/
#include "msp430.h"
#define WTH_DELAY 5000

__interrupt void P2_ISR( void );
void P2InterruptSetupAndLoop( void );

int main( void )
{
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;
  P2InterruptSetupAndLoop();
}

void P2InterruptSetupAndLoop( void )
{
  // configure clocks (VLO) <- also a bad idea, but hey
  DCOCTL = CALDCO_1MHZ; // not using DCO clock for VLO test, so arbitrary
  BCSCTL1 = XT2OFF;     // select low power options
  BCSCTL2 = SELM_3 | SELS; // master clock = VLO;
                                    // small clock = VLO too;
                                    // master divider = 1;
  BCSCTL3 = LFXT1S_2 | XCAP_1;      // low frequency clock is VLO;
                                    // capacitance set to 6 pf (not sure why)
  
  // configure to turn LEDs on
  P1DIR = 0x03;
  P1OUT = 0x00;
  
  // P2 input and turn on low-to-high interrupt for P2.0
  P2DIR = 0x00;
  P2IE = 0x01;
  P2IES = 0x00;
  
  __bis_SR_register(/*LPM3_bits + */GIE);       // /*Enter LPM3, */interrupts enabled
  
  // always check P2IN. if high at P2.1, turn on second LED
  while(1)
  {
    if(P2IN & 0x02)
      P1OUT |= 0x02;
    else
      P1OUT &= ~0x02;
  }
}

// Port 2 interrupt when goes low to high
#pragma vector=PORT2_VECTOR
__interrupt void P2_ISR( void )
{
  volatile unsigned int i;
  // toggle LED
  P1OUT ^= 0x01;
  for(i = 0; i < WTH_DELAY; i++);
  P2IFG = 0x00;
}