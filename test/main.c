/*
this main makes sure that we can use the intended method (reed switch to act as
input to use in clip detection. PIN and port interrupts
*/
#include "msp430.h"

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
  // configure to turn LEDs on
  P1DIR = 0x03;
  P1OUT = 0x00;
  
  // P2 input and turn on low-to-high interrupt for P2.0
  P2DIR = 0x00;
  P2IE = 0x01;
  P2IES = 0x00;
  
  __bis_SR_register(LPM3_bits + GIE);       // /*Enter LPM3, */interrupts enabled
  
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
  // toggle LED
  P1OUT ^= 0x01;
  P2IFG = 0x00;
}